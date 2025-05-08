from pg_job_management.jobs_common import DeviceInterface as DeviceInterfaceBase, OverallState
from rclpy.serialization import deserialize_message, serialize_message
from pg_moisture_sensor_msgs.msg import Job, Config, Sensor, ConfigStamped
from pg_msgs.msg import Float64ArrayStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

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
    self.startup_process, self.output_redir = self.run_external_process(
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

  def create_job_command(self, job_name: str, job_config_dict: dict) -> str | None:
    return None  # measurement job not implemented yet

  def start_updating_state(self, device_name: str, config_dict: dict, node: Node, state: OverallState):
    super().start_updating_state(device_name, config_dict, node, state)
    self.config_sub = node.create_subscription(ConfigStamped, device_name + '/config_stamped', self.config_callback, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    self.data_sub = node.create_subscription(Float64ArrayStamped, device_name + '/moistures', self.moistures_callback, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

  def stop_updating_state(self):
    self.config_sub.destroy()
    self.data_sub.destroy()

  def config_callback(self, config: ConfigStamped):
    self.config_callback_general(config, self.update_moistures)

  def moistures_callback(self, msg: Float64ArrayStamped):
    self.measurement_callback_general(msg, self.update_moistures)

  def update_moistures(self):
    config = self.active_config
    moistures = self.last_measurement.data
    moisture_i = 0
    with self.state as state:
      for sensor in config.config.sensors:
        if sensor.measure_name:
          if (moisture_i >= len(moistures)):
            self.logger.error(f"Moisture sensor {sensor.measure_name} not found ({self.device_name} is configured only for {len(moistures)} sensors)", throttle_duration_sec=3)
            break
          state.setdefault('plants', {}).setdefault(sensor.measure_name, {})['moisture'] = moistures[moisture_i]
          moisture_i += 1
