import rclpy
from rclpy.node import Node
from pg_msgs.srv import EditJob, GetJobList
from pg_msgs.msg import Job, Device

from crontab import CronTab, CronSlices
import yaml
import getpass
import re
import glob

from .jobs_common import *

class JobServer(Node):
  def __init__(self):
    super().__init__('job_server')
    os.makedirs(PG_HOME, exist_ok=True)
    self.get_job_list_service = self.create_service(GetJobList, 'get_job_list', self.get_job_list_callback)
    self.edit_job_service = self.create_service(EditJob, 'edit_job', self.edit_job_callback)

  def edit_job_callback(self, request: EditJob.Request, response: EditJob.Response):
    if not request.job.name or re.fullmatch(ALLOWED_NAMING_PATTERN, request.job.name) is None:
      response.message = "requested job name empty or not matching pattern " + ALLOWED_NAMING_PATTERN
      return response
    if not request.job.device.name or re.fullmatch(ALLOWED_NAMING_PATTERN, request.job.device.name) is None:
      response.message = "requested device name empty or not matching pattern " + ALLOWED_NAMING_PATTERN
      return response
    if request.command != EditJob.Request.MODIFY and request.command != EditJob.Request.ADD_OR_MODIFY and request.name_before_renaming:
      response.message = "name_before_renaming should be empty for this command (used only for renaming)"
      return response
    if re.fullmatch(ALLOWED_NAMING_PATTERN, request.name_before_renaming) is None:
      response.message = "name_before_renaming not matching pattern " + ALLOWED_NAMING_PATTERN
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

    if request.command == EditJob.Request.ADD or request.command == EditJob.Request.MODIFY or request.command == EditJob.Request.ADD_OR_MODIFY:
      if not CronSlices.is_valid(request.job.crontab_schedule):
        response.message = "invalid schedule"
        return response
      if request.job.device.type not in DEVICE_TYPES: # add more types here
        response.message = "unsupported device type"
        return response
      if request.job.device.name not in jobs_data:
        jobs_data[request.job.device.name] = {'__type': request.job.device.type}
      if request.job.device.type != jobs_data[request.job.device.name]["__type"]:
        response.message = "device type mismatch (" + request.job.device.type + " != " + jobs_data[request.job.device.name]["__type"] + ")"
        return response

      if request.command == EditJob.Request.ADD or request.name_before_renaming != request.job.name:
        if request.job.name in jobs_data[request.job.device.name]:
          response.message = "job data with this name already exist"
          return response
        if list(cron.find_comment(JOB_CRON_PREFIX + request.job.device.name + "." + request.job.name)):
          response.message = "job schedule with this name already exists"
          return response
      if request.command == EditJob.Request.MODIFY or request.command == EditJob.Request.ADD_OR_MODIFY:
        schedules_found = cron.remove_all(comment=JOB_CRON_PREFIX + request.job.device.name + "." + request.name_before_renaming)
        if request.command == EditJob.Request.MODIFY and schedules_found == 0:
          response.message = "job schedule not found"
          return response
        if request.name_before_renaming in jobs_data[request.job.device.name]:
          del jobs_data[request.job.device.name][request.name_before_renaming]
        elif request.command == EditJob.Request.MODIFY:
          response.message = "job data not found"
          return response

      if request.job.device.type == Device.TYPE_PUMP_CRANE:
        jobs_data[request.job.device.name][request.job.name] = {
          'angle_from': request.job.pump_crane_job.angle_from,
          'angle_to': request.job.pump_crane_job.angle_to,
          'pump_dur': request.job.pump_crane_job.pump_duration
        }
        job_command = 'bash -i -c "r2pg && ros2 run pg_pump_crane pump_crane_job_client --ros-args -r __ns:=/'+request.job.device.name+' -p job_name:='+request.job.name+\
                      '" 2>>~/plant_guard/err.log 1>>~/plant_guard/out.log'

      job = cron.new(command=job_command, comment=JOB_CRON_PREFIX + request.job.device.name + "." + request.job.name)
      job.setall(CronSlices(request.job.crontab_schedule))
      cron.write()
    elif request.command == EditJob.Request.DELETE:
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
    response.message = "job updated"
    return response

  def get_job_list_callback(self, request: GetJobList.Request, response: GetJobList.Response):
    if re.fullmatch(ALLOWED_NAMING_PATTERN, request.name) is None:
      response.error_message = "requested job name not matching pattern " + ALLOWED_NAMING_PATTERN
      return response
    if re.fullmatch(ALLOWED_NAMING_PATTERN, request.device) is None:
      response.error_message = "requested device name not matching pattern " + ALLOWED_NAMING_PATTERN
      return response

    cron = CronTab(user=getpass.getuser())
    try:
      with open(JOBS_YAML_PATH) as file:
        jobs_data = yaml.full_load(file)
    except:
      jobs_data = {}
    if jobs_data is None:
      jobs_data = {}

    job_regex = "^" + JOB_CRON_PREFIX.replace(".", "\\.") + (request.device if request.device else ".*") + "\\." + (request.name if request.name else ".*") + "$"
    for job in cron.find_comment(re.compile(job_regex)):
      device_end = job.comment.rfind(".")
      device_start = job.comment.find(".") + 1
      name = job.comment[device_end + 1:]
      device = job.comment[device_start:device_end]
      try:
        device_type = jobs_data[device]["__type"]
        job_data = jobs_data[device][name]
        job_msg = Job()
        job_msg.device.name = device
        job_msg.device.type = device_type
        job_msg.name = name
        job_msg.crontab_schedule = job.slices.render()
        if device_type == Device.TYPE_PUMP_CRANE:
          job_msg.pump_crane_job.angle_from = job_data["angle_from"]
          job_msg.pump_crane_job.angle_to = job_data["angle_to"]
          job_msg.pump_crane_job.pump_duration = job_data["pump_dur"]
        response.jobs.append(job_msg)
      except yaml.YAMLError:
        self.get_logger().error("YAML error (%s.%s)"%(device, name))

    return response

def main(args=None):
  rclpy.init(args=args)
  node = JobServer()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
