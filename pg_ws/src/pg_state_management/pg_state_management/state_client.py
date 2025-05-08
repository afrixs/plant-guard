import sys

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from pg_msgs.srv import EditDevice, GetDeviceList, GetState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import yaml
import re
from pg_job_management.jobs_common import *

class StateClient(Node):
  def __init__(self, expression=''):
    super().__init__('state_client')
    self.sync_service_callback_group = MutuallyExclusiveCallbackGroup()
    self.init_timer = self.create_timer(0.0, self.init_timer_callback)
    self.response = ''
    self.expression = self.declare_parameter('expression', '').get_parameter_value().string_value
    self.get_logger().info(self.expression)

  def init_timer_callback(self):
    self.init_timer.cancel()
    get_state_client = self.create_client(GetState, '/get_state', callback_group=self.sync_service_callback_group)
    if get_state_client.wait_for_service(timeout_sec=3.0):
      client_request = GetState.Request()
      client_request.expression = self.expression
      state_result: GetState.Response = get_state_client.call(client_request)
      if state_result is not None and not state_result.error_message:
        self.response = state_result.state
        self.get_logger().info(self.response)
      else:
        self.get_logger().error(f"get_state service call failed: {(state_result.error_message if state_result else 'NO_RESPONSE')}")
    else:
      self.get_logger().error(f"get_state service not available")
    rclpy.shutdown()

def main(args=None):
  rclpy.init(args=args)
  node = StateClient()
  executor = MultiThreadedExecutor(2)
  executor.add_node(node)
  try:
    executor.spin()
  except KeyboardInterrupt:
    node.get_logger().info('Keyboard interrupt, shutting down.\n')
  finally:
    node.destroy_node()
  sys.stdout.write(node.response)
  sys.exit(0 if node.response.lower() == 'true' else 1)

if __name__ == '__main__':
  main()
