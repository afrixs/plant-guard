import math

import rclpy
from rclpy.timer import Timer
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_srvs.srv import SetBool
from pg_msgs.msg import Float64ArrayStamped
from pg_moisture_sensor_msgs.msg import Config, ConfigStamped
from pg_moisture_sensor.ADS1256 import ADS1256
import RPi.GPIO as GPIO

class MoistureSensor(Node):
  def __init__(self):
    super().__init__('moisture_sensor')
    self.declare_parameter('live_measuring_period', 0.1)
    self.declare_parameter('idle_measuring_period', 600.0)

    self.ADC: ADS1256 = None
    self.live_measuring_timer: Timer = None

    self.config: ConfigStamped | None = None

    self.moistures_pub = self.create_publisher(Float64ArrayStamped, 'moistures', QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    self.config_pub = self.create_publisher(ConfigStamped, 'config_stamped', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    self.config_sub = self.create_subscription(Config, 'config', self.config_callback, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    self.enable_live_measuring_service = self.create_service(SetBool, 'enable_live_measuring', self.enable_live_measuring_callback)

    self.idle_measuring_timer = self.create_timer(self.get_parameter('idle_measuring_period').value, self.measure_idle)
    self.enable_live_measuring(True)

  def destroy_node(self):
    self.cleanupHW()
    super().destroy_node()

  def initHW(self):
    self.ADC = ADS1256()
    self.ADC.ADS1256_init()

  def cleanupHW(self):
    self.ADC = None
    GPIO.cleanup()

  def enable_live_measuring_callback(self, request: SetBool.Request, response: SetBool.Response):
    self.enable_live_measuring(request.data)
    response.success = True
    return response

  def enable_live_measuring(self, enable: bool):
    if enable:
      if self.live_measuring_timer is None:
        self.initHW()
        self.live_measuring_timer = self.create_timer(self.get_parameter('live_measuring_period').value, self.measure)
    else:
      if self.live_measuring_timer is not None:
        self.live_measuring_timer.cancel()
        self.live_measuring_timer = None
        self.cleanupHW()

  def measure_idle(self):
    if self.live_measuring_timer is None:
      self.initHW()
      self.measure()
      self.cleanupHW()

  def measure(self):
    if self.config is None:
        return
    vals = self.ADC.ADS1256_GetAll()
    msg = Float64ArrayStamped()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.config_id = self.config.header.config_id
    for i, sensor in enumerate(self.config.config.sensors):
      if not sensor.measure_name:
        continue
      if i >= len(vals):
        self.get_logger().error(f"Sensor {i} {sensor.measure_name} not found (ADS1256 supports only {len(vals)} sensors)", throttle_duration_sec=3)
        break
      if sensor.zero_moisture_value == sensor.full_moisture_value or not math.isfinite(sensor.zero_moisture_value) or not math.isfinite(sensor.full_moisture_value):
        msg.data.append(float(vals[i]))
      else:
        msg.data.append((vals[i] - sensor.zero_moisture_value) / (sensor.full_moisture_value - sensor.zero_moisture_value))
    self.moistures_pub.publish(msg)

  def config_callback(self, msg: Config):
    if self.config is None:
      self.config = ConfigStamped()
    self.config.header.stamp = self.get_clock().now().to_msg()
    self.config.header.config_id += 1
    self.config.config = msg
    self.config_pub.publish(self.config)
    self.measure_idle()

def main(args=None):
  rclpy.init(args=args)

  node = MoistureSensor()

  rclpy.spin(node)

  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
