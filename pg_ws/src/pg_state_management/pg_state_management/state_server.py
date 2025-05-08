import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from pg_msgs.srv import EditDevice, GetDeviceList, GetState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import yaml
import re
from pg_job_management.jobs_common import *

class StateServer(Node):
  def __init__(self):
    super().__init__('state_server')
    self.get_logger().info("starting StateServer")

    self.declare_parameter('is_state_montor', False)
    self.is_state_monitor = self.get_parameter('is_state_montor').get_parameter_value().bool_value

    self.state = OverallState()
    self.device_interfaces = {}
    self.sync_service_callback_group = MutuallyExclusiveCallbackGroup()

    self.init_timer = self.create_timer(0.0, self.init_timer_callback)

  def init_timer_callback(self):
    self.init_timer.cancel()
    if self.is_state_monitor:
      get_state_client = self.create_client(GetState, '/get_state', callback_group=self.sync_service_callback_group)
      if get_state_client.wait_for_service(timeout_sec=3.0):
        client_request = GetState.Request()
        state_result: GetState.Response = get_state_client.call(client_request)
        if state_result is not None and not state_result.error_message:
          with self.state as state:
            state.clear()
            state.update(yaml.safe_load(state_result.state))
        else:
          self.get_logger().error(f"get_state service call failed: {(state_result.error_message if state_result else 'NO_RESPONSE')}")
      else:
        self.get_logger().error(f"get_state service not available")

    get_devices_client = self.create_client(GetDeviceList, '/get_device_list', callback_group=self.sync_service_callback_group)
    if get_devices_client.wait_for_service(timeout_sec=3.0):
      client_request = GetDeviceList.Request()
      devices_result: GetDeviceList.Response = get_devices_client.call(client_request)
      if devices_result is not None and not devices_result.error_message:
        for device in devices_result.devices:
          edit_req = EditDevice.Request()
          edit_req.device = device
          edit_req.operation = EditDevice.Request.ADD
          self.edit_device(edit_req)
      else:
        self.get_logger().error(f"get_device_list service call failed: {devices_result.error_message if devices_result else 'NO RESPONSE'}")
    else:
      self.get_logger().error(f"get_device_list service not available")

    self.get_state_service = self.create_service(GetState, 'get_state', self.get_state_callback)
    self.edit_device_service = self.create_service(EditDevice, 'edit_device', self.edit_device_callback)
    self.get_logger().info("StateServer started")

  def get_state_callback(self, request: GetState.Request, response: GetState.Response):
    with self.state as state:
      if request.expression:
        try:
          state_children = {k: v for k, v in state.items()}
          result = eval(request.expression, {}, state_children)
          if result is DotMap:
            response.state = yaml.dump(result.toDict(), default_flow_style=True)
          elif result is dict:
            response.state = yaml.dump(result, default_flow_style=True)
          else:
            response.state = str(result)

        except Exception as e:
          response.error_message = 'Error: ' + str(e)
      else:
        response.state = yaml.dump(state.toDict(), default_flow_style=True)
    return response

  def edit_device_callback(self, request: EditDevice.Request, response: EditDevice.Response):
    if re.fullmatch(ALLOWED_NAMING_PATTERN, request.device.name) is None:
      response.message = "requested device name not matching pattern " + ALLOWED_NAMING_PATTERN
      return response
    if request.operation != EditDevice.Request.MODIFY and request.operation != EditDevice.Request.ADD_OR_MODIFY and request.name_before_renaming:
      response.message = "name_before_renaming should be empty for this operation (used only for renaming)"
      return response
    if request.name_before_renaming and re.fullmatch(ALLOWED_NAMING_PATTERN, request.name_before_renaming) is None:
      response.message = "name_before_renaming not empty and not matching pattern " + ALLOWED_NAMING_PATTERN
      return response
    if not request.name_before_renaming:
      request.name_before_renaming = request.device.name

    error = self.edit_device(request)
    if error:
      response.message = error
      return response
    return response

  def edit_device(self, request: EditDevice.Request) -> str | None:
    if (request.operation == EditDevice.Request.ADD or request.name_before_renaming != request.device.name) and request.device.name in self.device_interfaces:
      return "device with this name already exists"
    if (request.operation == EditDevice.Request.MODIFY or request.operation == EditDevice.Request.DELETE) and request.name_before_renaming not in self.device_interfaces:
      return "device not found"
    old_interface: DeviceInterface | None = self.device_interfaces.get(request.name_before_renaming, None)
    if old_interface:
      old_interface.stop_updating_state()
      del self.device_interfaces[request.name_before_renaming]
    if request.operation != EditDevice.Request.DELETE:
      interface = get_device_interface(request.device.type)
      if interface is None:
        return "unsupported device type"
      new_device_config = interface.create_device_config_dict(b''.join(request.device.config_msg_data))
      interface.start_updating_state(request.device.name, new_device_config, self, self.state)
      self.device_interfaces[request.device.name] = interface

def main(args=None):
  rclpy.init(args=args)
  node = StateServer()
  executor = MultiThreadedExecutor(2)
  executor.add_node(node)
  try:
    node.get_logger().info('Beginning client, shut down with CTRL-C')
    executor.spin()
  except KeyboardInterrupt:
    node.get_logger().info('Keyboard interrupt, shutting down.\n')
  finally:
    node.destroy_node()

if __name__ == '__main__':
  main()
