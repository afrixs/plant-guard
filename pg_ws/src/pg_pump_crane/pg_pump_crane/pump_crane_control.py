
import rclpy
from rclpy.time import Duration, Time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, Bool
# from pg_msgs.srv import GetJobs, EditJob
from pg_msgs.msg import MeasurementHeader
from pg_pump_crane_msgs.action import PerformJob
from pg_msgs.msg import Float64Stamped, BoolStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from pg_pump_crane.pump_crane import PumpCrane
import portalocker
from pathlib import Path
from threading import Lock, Thread, Condition

class PumpCraneControl(Node):
  def __init__(self):
    super().__init__('pump_crane_control')

    self.angle_pub = self.create_publisher(Float64Stamped, 'angle', 1) #QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    self.pump_pub = self.create_publisher(BoolStamped, 'pump', 1)
    # TODO: pump_pub

    self.pump_crane = PumpCrane(False, stepperPosCallback=lambda pos: self.stepper_pos_callback(pos), pumpCallback=lambda pumping: self.pump_callback(pumping))
    self.pump_crane.setKeyboardListening(True)
    self.pump_crane.cleanupHW()

    self.movement_dir_sub = self.create_subscription(Int32, 'movement_dir_cmd', self.movement_dir_cmd_callback, 1)
    self.pump_sub = self.create_subscription(Bool, 'pump_cmd', self.pump_cmd_callback, 1)

    # self.get_jobs_server = self.create_service(GetJobs, 'get_jobs', self.get_jobs_callback)
    # self.edit_job_server = self.create_service(EditJob, 'edit_job', self.edit_job_callback)

    self.perform_job_server = ActionServer(
      self,
      PerformJob,
      'perform_job',
      self.perform_job_callback,
      goal_callback=self.perform_job_goal_callback,
      cancel_callback=self.perform_job_cancel_callback,
      callback_group=MutuallyExclusiveCallbackGroup()
    )

    self.pump_crane_lock = Lock()
    self.canceled_jobs = []  # cannot be a set() because UUIDs are not hashable
    self.jobs_queue = []
    self.jobs_queue_condition = Condition()
    self.current_job_goal_handle = None
    self.current_job_goal_handle_lock = Lock()

    self.watchdog_timer = self.create_timer(0.5, self.watchdog_callback)
    self.last_pump_time = None
    self.last_movement_time = None

  def destroy_node(self):
    self.pump_crane.setKeyboardListening(False)
    self.pump_crane.stopPump()
    self.pump_crane.setMovementDir(0)
    self.pump_crane.cleanup()
    try:
      self.pump_crane_lock.release()
    except:
      pass
    super().destroy_node()

  def watchdog_callback(self):
    if self.last_pump_time is not None and self.get_clock().now() - self.last_pump_time > Duration(seconds=0.5):
      self.get_logger().warn("Connection lost, stopping pump")
      self.pump_crane.stopPump()
      self.last_pump_time = None
      if self.last_movement_time is None:
        self.pump_crane.cleanupHW()
        self.pump_crane_lock.release()
    if self.last_movement_time is not None and self.get_clock().now() - self.last_movement_time > Duration(seconds=0.5):
      self.get_logger().warn("Connection lost, stopping movement")
      self.pump_crane.setMovementDir(0)
      self.last_movement_time = None
      if self.last_pump_time is None:
        self.pump_crane.cleanupHW()
        self.pump_crane_lock.release()

  def movement_dir_cmd_callback(self, msg: Int32):
    was_moving = self.last_movement_time is not None
    self.last_movement_time = self.get_clock().now() if msg.data != 0 else None
    if self.last_movement_time is not None and not was_moving and self.last_pump_time is None:
      self.pump_crane_lock.acquire()
      self.pump_crane.initHW()
    self.pump_crane.setMovementDir(msg.data)
    if self.last_movement_time is None and was_moving and self.last_pump_time is None:
      self.pump_crane.cleanupHW()
      self.pump_crane_lock.release()

  def pump_cmd_callback(self, msg: Bool):
    was_pumping = self.last_pump_time is not None
    if msg.data:
      self.last_pump_time = self.get_clock().now()
      if not was_pumping and self.last_movement_time is None:
        self.pump_crane_lock.acquire()
        self.pump_crane.initHW()
      self.pump_crane.startPump()
    else:
      self.pump_crane.stopPump()
      self.last_pump_time = None
      if was_pumping and self.last_movement_time is None:
        self.pump_crane.cleanupHW()
        self.pump_crane_lock.release()

  def perform_job_goal_callback(self, goal: PerformJob.Goal):
    self.get_logger().info(f"Received PerformJob goal: ({goal.job.angle_from}˚ <-> {goal.job.angle_to}°) x {goal.job.pump_duration} s")
    return GoalResponse.ACCEPT

  def perform_job_cancel_callback(self, goal_handle):
    self.get_logger().info(f"PerformJob goal {goal_handle.goal_id} canceling")
    with self.jobs_queue_condition:
      if goal_handle.is_active:
        self.canceled_jobs.append(goal_handle.goal_id)
        self.jobs_queue_condition.notify_all()
    return CancelResponse.ACCEPT

  def perform_job_callback(self, goal_handle):
    goal: PerformJob.Goal = goal_handle.request
    result = PerformJob.Result()

    with self.jobs_queue_condition:
      self.jobs_queue.append(goal_handle.goal_id)
      if len(self.jobs_queue) > 1:
        self.get_logger().info(f"PerformJob goal {goal_handle.goal_id} queued")
        feedback = PerformJob.Feedback()
        feedback.stage = PerformJob.Feedback.STAGE_QUEUED
        goal_handle.publish_feedback(feedback)

        while rclpy.ok() and not (goal_handle.is_cancel_requested or goal_handle.goal_id in self.canceled_jobs) and self.jobs_queue[0] != goal_handle.goal_id:
          self.jobs_queue_condition.wait()

    if rclpy.ok() and not goal_handle.is_cancel_requested:
      self.get_logger().info(f"Executing PerformJob goal {goal_handle.goal_id}...")
      self.pump_crane_lock.acquire()
      self.pump_crane.initHW()

      with self.current_job_goal_handle_lock:
        self.current_job_goal_handle = goal_handle
        feedback = PerformJob.Feedback()
        feedback.stage = PerformJob.Feedback.STAGE_APPROACHING
        goal_handle.publish_feedback(feedback)

      self.pump_crane.allowMovement()
      water_thread = Thread(
        target=self.pump_crane.water,
        args=(goal.job.angle_from, goal.job.angle_to, goal.job.pump_duration, False),
        daemon=True)
      water_thread.start()

      while rclpy.ok() and not goal_handle.is_cancel_requested and water_thread.is_alive():
        # feedback = PerformJob.Feedback()
        # feedback.current_duration = (self.get_clock().now() - start).nanoseconds / 1e9
        # goal_handle.publish_feedback(feedback)
        self.get_clock().sleep_for(Duration(seconds=0.1))
      if water_thread.is_alive():
        self.pump_crane.terminateMovement()
      water_thread.join()

      with self.current_job_goal_handle_lock:
        self.current_job_goal_handle = None
      self.pump_crane.cleanupHW()
      self.pump_crane_lock.release()

    with self.jobs_queue_condition:
      self.jobs_queue.remove(goal_handle.goal_id)
      self.jobs_queue_condition.notify_all()

      # must be inside the lock to prevent adding the uuid to canceled_jobs after the job is done
      try:
        self.canceled_jobs.remove(goal_handle.goal_id)
      except ValueError:
        pass
      if goal_handle.is_cancel_requested:
        goal_handle.canceled()
      else:
        goal_handle.succeed()
    return result

  def stepper_pos_callback(self, pos):
    self.angle_pub.publish(Float64Stamped(header=MeasurementHeader(stamp=self.get_clock().now().to_msg()), data=pos))

  def pump_callback(self, pumping):
    self.get_logger().info("pumping: " + str(pumping))
    self.pump_pub.publish(BoolStamped(header=MeasurementHeader(stamp=self.get_clock().now().to_msg()), data=pumping))
    if pumping:
      with self.current_job_goal_handle_lock:
        if self.current_job_goal_handle is not None and self.current_job_goal_handle.is_active:
          feedback = PerformJob.Feedback()
          feedback.stage = PerformJob.Feedback.STAGE_WATERING
          self.current_job_goal_handle.publish_feedback(feedback)

def main(args=None):
  rclpy.init(args=args)
  node = PumpCraneControl()
  executor = MultiThreadedExecutor(12)
  executor.add_node(node)
  try:
    node.get_logger().info('Beginning client, shut down with CTRL-C')
    executor.spin()
  except KeyboardInterrupt:
    node.get_logger().info('Keyboard interrupt, shutting down.\n')
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
