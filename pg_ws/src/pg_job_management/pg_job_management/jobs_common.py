import importlib
import os
import sys
from pathlib import Path
import pty
import select
import errno
import subprocess
import threading
from rclpy.node import Node

JOB_CRON_PREFIX = "plant_guard."
PG_HOME = os.path.join(Path.home(), 'plant_guard')
JOBS_YAML_PATH = os.path.join(PG_HOME, 'jobs.yaml')
ALLOWED_NAMING_PATTERN = "(?!^__)([a-zA-Z_][a-zA-Z0-9_]*)"
JOB_COMMAND_FORBIDDEN_SEQUENCE = r'(?<!\\)"'

class DeviceInterface:
  # methods to be overriden
  def create_device_config_dict(self, config_msg_data: bytes) -> dict:
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

  def create_job_config_dict(self, job_config_msg_data: bytes) -> dict:
    # create job config dictionary
    return None

  def create_job_config_msg_data(self, job_config_dict: dict) -> bytes:
    # convert job config dictionary to message data
    return None

  def create_job_command(self, job_name: str) -> str:
    # create job command
    return None

  # helper methods
  def run_remote_process(self, process_name: str, command: str):
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

def get_device_interface(device_type) -> DeviceInterface:
  try:
    dt = importlib.import_module("pg_" + device_type + ".device").DeviceInterface
    if not issubclass(dt, DeviceInterface):
      return None
    return dt()
  except ImportError:
    return None
