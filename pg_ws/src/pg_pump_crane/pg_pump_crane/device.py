from pg_job_management.jobs_common import DeviceInterface as DeviceInterfaceBase
from rclpy.serialization import deserialize_message, serialize_message
from pg_pump_crane_msgs.msg import Job, Config

class DeviceInterface(DeviceInterfaceBase):
  def __init__(self):
    self.startup_process = None
    self.output_redir = None
    self.device_name = None
    self.config_dict = None

  def create_device_config_dict(self, config_msg_data: bytes) -> dict:
    # TODO: create device config dictionary
    return {}

  def start_device(self, device_name: str, config_dict: dict, _node) -> bool:
    self.config_dict = config_dict
    self.device_name = device_name
    self.startup_process, self.output_redir = self.run_remote_process(
      device_name, 'r2pg && ros2 run pg_pump_crane pump_crane_control --ros-args -r __ns:=/' + device_name)
    return True

  def update_config(self, config_dict: dict):
    self.config_dict = config_dict

  def create_device_config_msg_data(self):
    return serialize_message(Config())

  def stop_device(self) -> bool:
    self.stop_remote_process(self.device_name)
    if self.startup_process is not None:
      self.startup_process.wait()
    if self.output_redir is not None:
      self.output_redir.join()
    return True

  def create_job_config_dict(self, job_config_msg_data: bytes) -> dict:
    job_config_msg: Job = deserialize_message(job_config_msg_data, Job)
    return {
      'angle_from': job_config_msg.angle_from,
      'angle_to': job_config_msg.angle_to,
      'pump_dur': job_config_msg.pump_duration
    }

  def create_job_config_msg_data(self, job_config_dict: dict) -> bytes:
    job_config_msg = Job()
    job_config_msg.angle_from = job_config_dict['angle_from']
    job_config_msg.angle_to = job_config_dict['angle_to']
    job_config_msg.pump_duration = job_config_dict['pump_dur']
    return serialize_message(job_config_msg)

  def create_job_command(self, job_name: str) -> str:
    # create job command
    return 'r2pg && ros2 run pg_pump_crane pump_crane_job_client --ros-args -r __ns:=/'+self.device_name+' -p job_name:='+job_name