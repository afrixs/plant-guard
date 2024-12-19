from pg_job_management.jobs_common import DeviceInterface as DeviceInterfaceBase
from rclpy.serialization import deserialize_message, serialize_message
from pg_moisture_sensor_msgs.msg import Job, Config, Sensor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

class DeviceInterface(DeviceInterfaceBase):
  def __init__(self):
    self.startup_process = None
    self.output_redir = None
    self.device_name = None
    self.config_dict = None

  def create_device_config_dict(self, config_msg_data: bytes) -> dict:
    config_msg: Config = deserialize_message(config_msg_data, Config)
    return {
      'sensors': [{ 'measure_name': sensor.measure_name,
                    'zero_moisture_value': sensor.zero_moisture_value,
                    'full_moisture_value': sensor.full_moisture_value} for sensor in config_msg.sensors]
    }

  def start_device(self, device_name: str, config_dict: dict, node: Node) -> bool:
    self.config_dict = config_dict
    self.device_name = device_name
    self.startup_process, self.output_redir = self.run_remote_process(
      device_name, 'r2pg && ros2 run pg_moisture_sensor moisture_sensor --ros-args -r __ns:=/' + device_name)
    self.config_pub = node.create_publisher(Config, device_name + '/config', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    self.update_config(self.config_dict)
    return True

  def update_config(self, config_dict: dict):
    self.config_dict = config_dict
    self.config_pub.publish(self.create_device_config_msg_data())

  def create_device_config_msg_data(self):
    config_msg = Config()
    for sensor in self.config_dict['sensors']:
      sensor_msg = Sensor()
      sensor_msg.measure_name = sensor['measure_name']
      sensor_msg.zero_moisture_value = sensor['zero_moisture_value']
      sensor_msg.full_moisture_value = sensor['full_moisture_value']
      config_msg.sensors.append(sensor_msg)
    return serialize_message(config_msg)

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
      'measurement_duration': job_config_msg.measurement_duration,
      'termination_condition': job_config_msg.termination_condition
    }

  def create_job_config_msg_data(self, job_config_dict: dict) -> bytes:
    job_config_msg = Job()
    job_config_msg.measurement_duration = job_config_dict['measurement_duration']
    job_config_msg.termination_condition = job_config_dict['termination_condition']
    return serialize_message(job_config_msg)

  def create_job_command(self, job_name: str) -> str:
    return None  # measurement job not implemented yet