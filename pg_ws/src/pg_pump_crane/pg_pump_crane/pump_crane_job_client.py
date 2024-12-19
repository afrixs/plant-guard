import signal

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from pg_pump_crane_msgs.action import PerformJob
from bagtube_msgs.action import RecordBag
import os
import yaml
from sys import argv
from pathlib import Path
import threading

class PumpCraneJobClient(Node):
  def __init__(self):
    super().__init__('pump_crane_job_client')
    self.declare_parameter('job_name', '')
    self.job_client = ActionClient(self, PerformJob, 'perform_job')
    self.record_client = ActionClient(self, RecordBag, '/record_bag')
    self.record_handle = None
    self.recording_started = False

  def start_job(self):
    self.get_logger().info('Starting job')
    goal_msg = PerformJob.Goal()
    self.job_name = self.get_parameter('job_name').get_parameter_value().string_value
    with open(os.path.join(Path.home(), 'plant_guard', 'jobs.yaml')) as file:
      jobsData = yaml.full_load(file)
    devicePath = self.get_namespace().split('/')
    for p in devicePath:
      if p != '':
        jobsData = jobsData[p]
    job = jobsData[self.job_name]
    goal_msg.job.angle_from = float(job["angle_from"])
    goal_msg.job.angle_to = float(job["angle_to"])
    goal_msg.job.pump_duration = float(job["pump_dur"])
    self.get_logger().info(f'Waiting for job server...')
    self.job_client.wait_for_server()
    self.get_logger().info(f'Waiting for record server...')
    self.record_client.wait_for_server()

    rclpy.uninstall_signal_handlers()
    signal.signal(signal.SIGINT,
                  lambda sign, frame:
                    self.get_logger().error('Received signal (Ctrl+C) but ignoring it since the action '
                                            'would not be properly canceled otherwise (TODO: fixme).'))
    self.get_logger().info('Note: signal (Ctrl+C) handling disabled since the action '
                           'would not be properly canceled otherwise (TODO: fixme).')

    self.get_logger().info('Sending job goal ' + self.job_name + '...')
    self.job_handle = self.get_goal_handle(self.job_client, goal_msg, feedback_callback=self.feedback_callback)
    self.get_logger().info('Received job goal handle')
    return self.job_handle.get_result_async()

  def get_goal_handle(self, client, goal_msg, feedback_callback=None):
    handle_future = client.send_goal_async(goal_msg, feedback_callback=feedback_callback)
    rclpy.spin_until_future_complete(self, handle_future)
    if handle_future.exception() is not None:
      return None
    return handle_future.result()

  def finalize_job(self):
    if self.record_handle is not None:
      cancel_future = self.record_handle.cancel_goal_async()
      self.get_logger().info('Waiting for record cancel response...')
      rclpy.spin_until_future_complete(self, cancel_future)
      self.get_logger().info('Waiting for record future result...')
      rclpy.spin_until_future_complete(self, self.record_future)
      self.get_logger().info('Recording finished')

  def feedback_callback(self, feedback_msg: PerformJob.Impl.FeedbackMessage):
    self.get_logger().info(f'Feedback: {feedback_msg}')
    if feedback_msg.feedback.stage != PerformJob.Feedback.STAGE_QUEUED and not self.recording_started:
      self.recording_started = True
      self.get_logger().info('Sending record goal ' + self.job_name + '...')
      record_goal = RecordBag.Goal()
      record_goal.name = self.job_name
      self.record_goal_future = self.record_client.send_goal_async(record_goal)
      self.record_goal_future.add_done_callback(self.record_goal_callback)

  def record_goal_callback(self, future):
    if future.exception() is not None:
      self.get_logger().info('Failed to send record goal: ' + str(future.exception()))
      return
    self.record_handle = future.result()
    self.get_logger().info('Received record goal handle')
    self.record_future = self.record_handle.get_result_async()

def main(args=None):
  rclpy.init(args=args)
  node = PumpCraneJobClient()
  future = node.start_job()
  node.get_logger().info('Spinning until job finished')
  rclpy.spin_until_future_complete(node, future)
  node.get_logger().info('Job finished')
  node.finalize_job()

  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
