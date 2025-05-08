import importlib
import os
import sys
import traceback
from pathlib import Path
import pty
import select
import errno
import subprocess
import threading
from rclpy.node import Node
from rclpy.time import Time
from collections import deque
import yaml
from dotmap import DotMap

JOB_CRON_PREFIX = "plant_guard."
PG_HOME = os.path.join(Path.home(), 'plant_guard')
JOBS_YAML_PATH = os.path.join(PG_HOME, 'jobs.yaml')
STATE_YAML_PATH = os.path.join(PG_HOME, 'state.yaml')
ALLOWED_NAMING_PATTERN = "(?!^__)([a-zA-Z_][a-zA-Z0-9_]*)"
JOB_COMMAND_FORBIDDEN_SEQUENCE = r'(?<!\\)"'

class OverallState:
  def __init__(self):
    self._state_lock = threading.Lock()
    try:
      with open(STATE_YAML_PATH) as file:
        self._state = DotMap(yaml.full_load(file))
    except FileNotFoundError:
      self._state = DotMap()

  def __enter__(self) -> DotMap:
    self._state_lock.acquire()
    return self._state

  def __exit__(self, exc_type, exc_val, exc_tb):
    with open(STATE_YAML_PATH, 'w') as file:  # TODO: don't write after every update?
      yaml.dump(self._state.toDict(), file)
    self._state_lock.release()

class DeviceInterface:
  # methods to be overriden
  def create_device_config_dict(self, config_msg_data: bytes) -> dict | None:
    # create device config dictionary
    return None

  def create_device_config_msg_data(self):
    # convert device config dictionary to message data
    return None

  def start_device(self, device_name: str, config_dict: dict, node: Node) -> bool:
    # start device node
    return False

  def update_config(self, config_dict: dict):
    # update device config
    pass

  def stop_device(self) -> bool:
    # stop device node
    return False

  def create_job_config_dict(self, job_config_msg_data: bytes) -> dict | None:
    # create job config dictionary
    return None

  def create_job_config_msg_data(self, job_config_dict: dict) -> bytes | None:
    # convert job config dictionary to message data
    return None

  def create_job_command(self, job_name: str, job_config_dict: dict) -> str | None:
    # create job command
    return None

  def start_updating_state(self, device_name: str, config_dict: dict, node: Node, state: OverallState):
    self.logger = node.get_logger()
    self.name = device_name
    self.state = state
    self.active_config = None
    self.last_measurement = None
    self.config_queue = deque()

  def stop_updating_state(self):
    pass

  def config_callback_general(self, config, measurement_callback):
    if self.last_measurement and Time.from_msg(self.last_measurement.header.stamp) >= Time.from_msg(config.header.stamp):
      self.active_config = config
      self.config_queue.clear()
      if self.last_measurement.header.config_id == self.active_config.header.config_id:
        measurement_callback()
    else:
      self.config_queue.append(config)

  def measurement_callback_general(self, msg, measurement_callback):
    self.last_measurement = msg
    while self.config_queue and Time.from_msg(msg.header.stamp) >= Time.from_msg(self.config_queue[0].header.stamp):
      self.active_config = self.config_queue.popleft()
    if (self.active_config and self.last_measurement.header.config_id == self.active_config.header.config_id and
            Time.from_msg(self.last_measurement.header.stamp) >= Time.from_msg(self.active_config.header.stamp)):
      measurement_callback()

# helper methods
  def run_external_process(self, process_name: str, command: str):
    mo, so = pty.openpty()  # provide tty to enable line-buffering
    me, se = pty.openpty()
    p = subprocess.Popen('{ outer_stdout=$(readlink -f /proc/self/fd/3); } 3>&1 && { outer_stderr=$(readlink -f /proc/self/fd/4); } 4>&2 && '
                         'screen -DmS ' + process_name + ' bash -i -c "' + command +
                         ' 1>$outer_stdout 2>$outer_stderr"', shell=True, stdout=so, stderr=se)
    for fd in [so, se]:
      os.close(fd)

    def redirect_output(p, mo, me):
      readable = [mo, me]
      try:
        while readable:
          ready, _, _ = select.select(readable, [], [])
          for fd in ready:
            try:
              data = os.read(fd, 512)
            except OSError as e:
              if e.errno != errno.EIO:
                raise
              # EIO means EOF on some systems
              readable.remove(fd)
            else:
              if not data: # EOF
                readable.remove(fd)
              print(data.decode('utf-8'), file=sys.stderr if fd is me else sys.stdout, end='', flush=True)
      finally:
        for fd in [mo, me]:
          os.close(fd)
        if p.poll() is None:
          p.kill()
        p.wait()

    t = threading.Thread(target=redirect_output, args=(p, mo, me))
    t.start()

    return p, t

  def stop_remote_process(self, process_name: str):
    os.system('screen -S ' + process_name + ' -X quit')

def get_device_interface(device_type) -> DeviceInterface | None:
  try:
    dt = importlib.import_module("pg_" + device_type + ".device").DeviceInterface
    if not issubclass(dt, DeviceInterface):
      return None
    return dt()
  except ImportError:
    traceback.print_exc()
    return None
