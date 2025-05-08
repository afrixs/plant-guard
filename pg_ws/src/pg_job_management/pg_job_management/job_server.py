import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from pg_msgs.srv import EditJob, GetJobList, EditDevice, GetDeviceList, GetState
from pg_msgs.msg import Job, Device

from crontab import CronTab, CronSlices
import yaml
import getpass
import re
import glob

from .jobs_common import *

class JobServerException(Exception):
  pass
class JobServer(Node):
  def __init__(self):
    super().__init__('job_server')
    os.makedirs(PG_HOME, exist_ok=True)

    self.device_interfaces = {}
    try:
      with open(JOBS_YAML_PATH) as file:
        jobs_data = yaml.full_load(file)
    except:
      jobs_data = {}
    if jobs_data is None:
      jobs_data = {}
    for device in jobs_data:
      interface = get_device_interface(jobs_data[device]["__type"])
      if interface is None:
        self.get_logger().error("unsupported device type %s for device %s"%(jobs_data[device]["__type"], device))
        rclpy.shutdown()
        return
      if not interface.start_device(device, jobs_data[device]["__config"], self):
        self.get_logger().error("failed to start device %s"%device)
        rclpy.shutdown()
        return
      self.device_interfaces[device] = interface

    self.sync_service_callback_group = MutuallyExclusiveCallbackGroup()
    self.get_state_client = self.create_client(GetState, '/get_state', callback_group=self.sync_service_callback_group)

    self.get_job_list_service = self.create_service(GetJobList, 'get_job_list', self.get_job_list_callback)
    self.edit_job_service = self.create_service(EditJob, 'edit_job', self.edit_job_callback)
    self.get_device_list_service = self.create_service(GetDeviceList, 'get_device_list', self.get_device_list_callback)
    self.edit_device_service = self.create_service(EditDevice, 'edit_device', self.edit_device_callback)
    self.get_logger().info("job server started")

  def destroy_node(self):
    self.get_logger().info("stopping devices")
    for (device, interface) in self.device_interfaces.items():
      if not interface.stop_device():
        self.get_logger().error("failed to stop device %s"%device)
    super().destroy_node()

  def edit_job_callback(self, request: EditJob.Request, response: EditJob.Response):
    if re.fullmatch(ALLOWED_NAMING_PATTERN, request.job.name) is None:
      response.message = "requested job name not matching pattern " + ALLOWED_NAMING_PATTERN
      return response
    if re.fullmatch(ALLOWED_NAMING_PATTERN, request.job.device.name) is None:
      response.message = "requested device name empty or not matching pattern " + ALLOWED_NAMING_PATTERN
      return response
    if request.operation != EditJob.Request.MODIFY and request.operation != EditJob.Request.ADD_OR_MODIFY and request.name_before_renaming:
      response.message = "name_before_renaming should be empty for this operation (used only for renaming)"
      return response
    if request.name_before_renaming and re.fullmatch(ALLOWED_NAMING_PATTERN, request.name_before_renaming) is None:
      response.message = "name_before_renaming not empty and not matching pattern " + ALLOWED_NAMING_PATTERN
      return response
    if not request.name_before_renaming:
      request.name_before_renaming = request.job.name

    cron = CronTab(user=getpass.getuser())
    try:
      with open(JOBS_YAML_PATH) as file:
        jobs_data = yaml.full_load(file)
    except:
      jobs_data = {}
    if jobs_data is None:
      jobs_data = {}

    message_suffix = ""
    if request.operation == EditJob.Request.ADD or request.operation == EditJob.Request.MODIFY or request.operation == EditJob.Request.ADD_OR_MODIFY:
      if not CronSlices.is_valid(request.job.crontab_schedule):
        response.message = "invalid schedule"
        return response
      interface = self.device_interfaces.get(request.job.device.name, None)
      if interface is None or request.job.device.name not in jobs_data:
        response.message = "use edit_device service to create the device first"
        return response
      if request.job.device.type != jobs_data[request.job.device.name]["__type"]:
        response.message = "device type mismatch (" + request.job.device.type + " != " + jobs_data[request.job.device.name]["__type"] + ")"
        return response

      if request.operation == EditJob.Request.ADD or request.name_before_renaming != request.job.name:
        if request.job.name in jobs_data[request.job.device.name]:
          response.message = "job data with this name already exist"
          return response
        if list(cron.find_comment(JOB_CRON_PREFIX + request.job.device.name + "." + request.job.name)):
          response.message = "job schedule with this name already exists"
          return response
      if request.operation == EditJob.Request.MODIFY or request.operation == EditJob.Request.ADD_OR_MODIFY:
        schedules_found = cron.remove_all(comment=JOB_CRON_PREFIX + request.job.device.name + "." + request.name_before_renaming)
        if request.operation == EditJob.Request.MODIFY and schedules_found == 0:
          response.message = "job schedule not found"
          return response
        if request.name_before_renaming in jobs_data[request.job.device.name]:
          del jobs_data[request.job.device.name][request.name_before_renaming]
        elif request.operation == EditJob.Request.MODIFY:
          response.message = "job data not found"
          return response

      job_data = interface.create_job_config_dict(b''.join(request.job.config_msg_data))
      if job_data is None:
        response.message = "device does not support jobs or job config not valid"
        return response
      if request.job.condition:
        job_data['__condition'] = request.job.condition
      jobs_data[request.job.device.name][request.job.name] = job_data
      try:
        job_command = self.create_job_command(request.job.name, job_data, interface)
      except JobServerException as e:
        response.message = str(e)
        return response

      if request.job.condition:
        if self.get_state_client.wait_for_service(timeout_sec=3.0):
          client_request = GetState.Request()
          client_request.expression = request.job.condition
          state_result: GetState.Response = self.get_state_client.call(client_request)
          if state_result is not None and not state_result.error_message:
            message_suffix = ' Job condition currently returns: ' + state_result.state
          else:
            message_suffix = ' Job condition check failed: ' + (state_result.error_message if state_result else 'NO_RESPONSE')
        else:
            message_suffix = ' Job condition check failed: service not available'

      job = cron.new(command=job_command, comment=JOB_CRON_PREFIX + request.job.device.name + "." + request.job.name)
      job.setall(CronSlices(request.job.crontab_schedule))
      cron.write()
    elif request.operation == EditJob.Request.DELETE:
      if request.job.device.name not in jobs_data or request.job.name not in jobs_data[request.job.device.name]:
        response.message = "job data not found"
      else:
        del jobs_data[request.job.device.name][request.job.name]
      if jobs_data[request.job.device.name] == {'__type': request.job.device.type}: # no jobs left
        del jobs_data[request.job.device.name]
      if not cron.remove_all(comment=JOB_CRON_PREFIX + request.job.device.name + "." + request.job.name):
        response.message = (response.message + ", " if response.message else "") + "job schedule not found"
      if response.message:
        return response
      cron.write()

    with open(JOBS_YAML_PATH, 'w') as file:
      yaml.dump(jobs_data, file)
    response.success = True
    response.message = "Job updated." + message_suffix
    return response

  def get_job_list_callback(self, request: GetJobList.Request, response: GetJobList.Response):
    if request.name and re.fullmatch(ALLOWED_NAMING_PATTERN, request.name) is None:
      response.error_message = "requested job name not empty and not matching pattern " + ALLOWED_NAMING_PATTERN
      return response
    if request.device_name and re.fullmatch(ALLOWED_NAMING_PATTERN, request.device_name) is None:
      response.error_message = "requested device name not empty and not matching pattern " + ALLOWED_NAMING_PATTERN
      return response

    cron = CronTab(user=getpass.getuser())
    try:
      with open(JOBS_YAML_PATH) as file:
        jobs_data = yaml.full_load(file)
    except:
      jobs_data = {}
    if jobs_data is None:
      jobs_data = {}

    job_regex = "^" + JOB_CRON_PREFIX.replace(".", "\\.") + (request.device_name if request.device_name else ".*") + "\\." + (request.name if request.name else ".*") + "$"
    for job in cron.find_comment(re.compile(job_regex)):
      device_end = job.comment.rfind(".")
      device_start = job.comment.find(".") + 1
      name = job.comment[device_end + 1:]
      device = job.comment[device_start:device_end]
      interface = self.device_interfaces.get(device, None)
      if interface is None:
        response.error_message = "device interface is not running"
        return response
      try:
        device_type = jobs_data[device]["__type"]
        job_data = jobs_data[device][name]
        job_msg = Job()
        job_msg.device.name = device
        job_msg.device.type = device_type
        job_msg.name = name
        job_msg.crontab_schedule = job.slices.render()
        job_msg.condition = job_data.get('__condition', '')

        job_msg.config_msg_data = interface.create_job_config_msg_data(job_data)
        response.jobs.append(job_msg)
      except yaml.YAMLError:
        self.get_logger().error("YAML error (%s.%s)"%(device, name))

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

    try:
      with open(JOBS_YAML_PATH) as file:
        jobs_data = yaml.full_load(file)
    except:
      jobs_data = {}
    if jobs_data is None:
      jobs_data = {}

    message_suffix = ""
    if request.operation == EditDevice.Request.ADD or request.operation == EditDevice.Request.MODIFY or request.operation == EditDevice.Request.ADD_OR_MODIFY:
      if request.name_before_renaming in jobs_data and request.device.type != jobs_data[request.name_before_renaming]["__type"]:
        response.message = ("device type mismatch (" + request.device.type + " != " + jobs_data[request.name_before_renaming]["__type"] +
                            "). To change device type, delete it first and create a new one.")
        return response

      if (request.operation == EditDevice.Request.ADD or request.name_before_renaming != request.device.name) and request.device.name in jobs_data:
        response.message = "device with this name already exists"
        return response
      if request.operation == EditDevice.Request.MODIFY and request.name_before_renaming not in jobs_data:
        response.message = "device data not found"
        return response
      old_interface = self.device_interfaces.get(request.name_before_renaming, None)
      interface = self.device_interfaces.get(request.device.name, None)
      creating_interface = interface is None
      if creating_interface:
        interface = get_device_interface(request.device.type)
        if interface is None:
          response.message = "unsupported device type"
          return response

      cron = None
      if request.name_before_renaming != request.device.name:
        jobs_data[request.device.name] = jobs_data[request.name_before_renaming]
        del jobs_data[request.name_before_renaming]

        cron = CronTab(user=getpass.getuser())
        for job in cron.find_comment(re.compile("^" + JOB_CRON_PREFIX.replace(".", "\\.") + request.name_before_renaming + "\\.")):
          job.set_comment(job.comment.replace("." + request.name_before_renaming + ".", "." + request.device.name + ".", 1))
          job_name = job.comment[job.comment.find("." + request.device.name + ".") + len(request.device.name) + 2:]
          job_data = jobs_data[request.device.name].get(job_name, {})
          try:
            job.set_command(self.create_job_command(job_name, job_data, interface))
          except JobServerException as e:
            response.message = str(e)
            return response

      new_device_config = interface.create_device_config_dict(b''.join(request.device.config_msg_data))
      if new_device_config is None:
        response.message = "failed to create device configuration"
        return response
      print(jobs_data)
      if request.device.name not in jobs_data:
        jobs_data[request.device.name] = {'__type': request.device.type, '__config': new_device_config}
      else:
        jobs_data[request.device.name]['__config'] = new_device_config

      if cron is not None:  # device name changed
        old_interface.stop_device()
        cron.write()
        if not interface.start_device(request.device.name, new_device_config, self):
          response.message = "failed to start device"
          return response
      elif creating_interface:
        if not interface.start_device(request.device.name, new_device_config, self):
          response.message = "failed to start device"
          return response
      else:
        interface.update_config(new_device_config)
      # TODO: update zenoh and bagpipe
    elif request.operation == EditDevice.Request.DELETE:
      if request.device.name not in jobs_data:
        response.message = "device data not found"
      else:
        del jobs_data[request.device.name]
      cron = CronTab(user=getpass.getuser())
      for job in cron.find_comment(re.compile("^" + JOB_CRON_PREFIX.replace(".", "\\.") + request.device.name + "\\.")):
        cron.remove(job)
      interface = self.device_interfaces.get(request.device.name, None)
      if interface is not None:
        if not interface.stop_device():
          response.message = "failed to stop device"
          return response
        del self.device_interfaces[request.device.name]
      cron.write()
      # TODO: update zenoh and bagpipe

    with open(JOBS_YAML_PATH, 'w') as file:
      yaml.dump(jobs_data, file)
    response.success = True
    response.message = "device updated" + message_suffix
    return response

  def get_device_list_callback(self, request: GetDeviceList.Request, response: GetDeviceList.Response):
    if request.name and re.fullmatch(ALLOWED_NAMING_PATTERN, request.name) is None:
      response.error_message = "requested device name not empty and not matching pattern " + ALLOWED_NAMING_PATTERN
      return response
    if request.type and re.fullmatch(ALLOWED_NAMING_PATTERN, request.type) is None:
      response.error_message = "requested device type not empty and not matching pattern " + ALLOWED_NAMING_PATTERN
      return response

    try:
      with open(JOBS_YAML_PATH) as file:
        jobs_data = yaml.full_load(file)
    except:
      jobs_data = {}
    if jobs_data is None:
      jobs_data = {}

    for (device_name, device_data) in jobs_data.items():
      if (request.name and device_name != request.name) or (request.type and device_data["__type"] != request.type):
        continue
      device = Device()
      device.name = device_name
      device.type = device_data["__type"]
      interface = self.device_interfaces.get(device_name, None)
      if interface is not None:
        device.config_msg_data = interface.create_device_config_msg_data()
      else:
        self.get_logger().error("device interface is not running")
        continue
      response.devices.append(device)

    return response

  def create_job_command(self, job_name: str, job_data: dict, interface: DeviceInterface) -> str:
    job_command_pure = interface.create_job_command(job_name, job_data)
    if job_command_pure is None:
      raise JobServerException("device does not support job scheduling")
    if re.search(JOB_COMMAND_FORBIDDEN_SEQUENCE, job_command_pure):
      raise JobServerException("job command contains forbidden sequence " + JOB_COMMAND_FORBIDDEN_SEQUENCE)
    condition_cmd = ''
    condition: str | None = job_data.get('__condition', None)
    if condition:
      condition_rosparametrized = re.sub(r'\\\\', '\\\\x5C', condition)
      condition_rosparametrized = re.sub(r'(["\'])', '\\\\$1', condition_rosparametrized)
      condition_cmd = 'r2pg && ros2 run pg_state_management state_client --ros-args -p expression:=$\'' + condition_rosparametrized + '\' && '

    return 'bash -i -c "' + condition_cmd + job_command_pure + '" 2>>~/plant_guard/err.log 1>>~/plant_guard/out.log'

def main(args=None):
  rclpy.init(args=args)
  node = JobServer()
  if not rclpy.ok():
    node.destroy_node()
    return
  executor = MultiThreadedExecutor(2)
  executor.add_node(node)
  try:
    executor.spin()
  except KeyboardInterrupt:
    node.get_logger().info('Keyboard interrupt, shutting down.\n')
  finally:
    node.destroy_node()
    os.system("screen -ls | grep '(Detached)' | awk '{ print $1 }' |  xargs -i@ screen -S @ -X quit")  # remove clutter that possibly remained in case of errors

if __name__ == '__main__':
  main()
