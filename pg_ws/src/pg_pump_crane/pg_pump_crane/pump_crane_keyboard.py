#!/usr/bin/env python3
import pg_msgs.msg

#pip3 install python_crontab pyyaml portalocker
from pump_crane import PumpCrane
from crontab import CronTab, CronSlices
import yaml
import portalocker
from pathlib import Path
import sys,tty,termios
import getpass
import re
import time
from pg_job_management.jobs_common import *
from pg_msgs.msg import Device
# from pynput import keyboard

pumpStartTime = time.time()
pumpingDuration = 0
angleFrom = -1
angleTo = -1
pumpRunning = False
DEVICE_NAME = "pump_crane"
JOB_PREFIX = JOB_CRON_PREFIX+DEVICE_NAME+"."

pumpCrane = None

def on_press(k):
  global pumpStartTime
  global pumpingDuration
  global angleFrom
  global angleTo
  global pumpRunning
  # if key == keyboard.Key.esc:
  # if k == 'esc':
  #   if keyboardListening:
  #     print("Stopping keyboard control. Press Enter to start keyboard control again")
  #     keyboardListening = False
  #   else:
  #     print("Stopping keyboard listening")
  #     return False
  # if key == keyboard.Key.enter:
  if k == 'return':
    if angleTo == -1 and angleFrom == -1:
      print("first press 'f' or 't' to set angleFrom or angleTo")
      return True
    if pumpingDuration == 0:
      print("first press and hold 'p' to set pumping duration")
      return True
    print("Enter a name of the new job")
    name = input()
    try:
      with open(JOBS_YAML_PATH) as file:
        jobsData = yaml.full_load(file)
    except:
      jobsData = {}
    if jobsData is None:
      jobsData = {}
    if DEVICE_NAME not in jobsData:
      jobsData[DEVICE_NAME] = {'__type': Device.TYPE_PUMP_CRANE}
    jobsData[DEVICE_NAME][name] = {
        'angle_from': angleFrom if angleFrom != -1 else angleTo,
        'angle_to': angleTo if angleTo != -1 else angleFrom,
        'pump_dur': pumpingDuration
      }
    with open(JOBS_YAML_PATH, 'w') as file:
      yaml.dump(jobsData, file)
    
    print("Enter schedule\nminute hour day_of_month month day_of_week\nexample (every friday at 6AM and 6PM): 0 6,18 * * Fri)")
    schedule = input()
    while not CronSlices.is_valid(schedule):
      print("Invalid schedule")
      schedule = input()
    cron = CronTab(user=getpass.getuser())
    # job = cron.new(command='python3 ~/plant_guard/plant_guard_job.py '+name, comment=JOB_PREFIX+name)
    job = cron.new(command='bash -i -c "r2pg && ros2 run pg_pump_crane pump_crane_job_client --ros-args -r __ns:=/'+DEVICE_NAME+' -p job_name:='+name+
                           '" 2>>~/plant_guard/err.log 1>>~/plant_guard/out.log', comment=JOB_PREFIX + name)
    job.setall(schedule)
    cron.write()
    print("Added job "+name+"  "+schedule+"  "+str(jobsData[DEVICE_NAME][name]))

  # try:
  #   k = key.char  # single-char keys
  # except:
  #   k = key.name  # other keys
  if k == 'l':
    print("Plant guard jobs:")
    cron = CronTab(user=getpass.getuser())
    try:
      with open(JOBS_YAML_PATH) as file:
        jobsData = yaml.full_load(file)
    except:
      jobsData = {}
    if jobsData is None:
      jobsData = {}
    for job in cron.find_comment(re.compile("^" + JOB_PREFIX.replace(".", "\\.") + ".*$")):
      name = job.comment[len(JOB_PREFIX):]
      print(name+"  "+str(job.slices)+"  "+(str(jobsData[DEVICE_NAME][name]) if DEVICE_NAME in jobsData and name in jobsData[DEVICE_NAME] else "No data!"))
  if k == 'd':
    print("Enter the name of the job to be deleted")
    name = input()
    cron = CronTab(user=getpass.getuser())
    if cron.remove_all(comment=JOB_PREFIX+name) == 0:
      print("No job found")
    else:
      print("Deleted job")
    cron.write()

    try:
      with open(JOBS_YAML_PATH) as file:
        jobsData = yaml.full_load(file)
    except:
      jobsData = {}
    if jobsData is None:
      jobsData = {}
    if DEVICE_NAME in jobsData and name in jobsData[DEVICE_NAME]:
      del jobsData[DEVICE_NAME][name]
      with open(JOBS_YAML_PATH, 'w') as file:
        yaml.dump(jobsData, file)
      print("Deleted data")
    else:
      print("No data found")
  if k == 'x':
    cron = CronTab(user=getpass.getuser())
    if cron.remove_all(comment=re.compile("^" + JOB_PREFIX.replace(".", "\\.") + ".*$")) == 0:
      print("No job found for " + DEVICE_NAME)
    else:
      print("Deleted all jobs for " + DEVICE_NAME)
    cron.write()
    try:
      with open(JOBS_YAML_PATH) as file:
        jobsData = yaml.full_load(file)
    except:
      jobsData = {}
    if jobsData is None:
      jobsData = {}
    if DEVICE_NAME in jobsData:
      del jobsData[DEVICE_NAME]
      with open(JOBS_YAML_PATH, 'w') as file:
        yaml.dump(jobsData, file)
      print("Deleted all job data for " + DEVICE_NAME)
    else:
      print("No job data found for " + DEVICE_NAME)

  if k == 'left':
    pumpCrane.setMovementDir(0 if pumpCrane.getMovementDir() == 1 else 1)
  if k == 'right':
    pumpCrane.setMovementDir(0 if pumpCrane.getMovementDir() == -1 else -1)
  if k == 'p':
    if pumpRunning:
      pumpRunning = False
      pumpCrane.stopPump()
      pumpingDuration = time.time() - pumpStartTime
      print("Pumping time: " + str(pumpingDuration))
    else:
      print("Starting pump")
      pumpRunning = True
      pumpCrane.startPump()
      pumpStartTime = time.time()
  if k == 'f':
    angleFrom = pumpCrane.getCurrentAngle()
    print("Angle from: " + str(angleFrom))
  if k == 't':
    angleTo = pumpCrane.getCurrentAngle()
    print("Angle to: " + str(angleTo))
  if k == 'h':
    print("left - move left\nright - move right\nl - list jobs\nx - delete all jobs\nd - delete job\nf - set angleFrom\nt - set angleTo\np - start/stop pump\nenter - save job\nesc/q - quit")


  return True

# def on_release(key):
#   global pumpingDuration
#   global pumpRunning
#   try:
#     k = key.char  # single-char keys
#   except:
#     k = key.name  # other keys
#   if (k == 'left' and getKeyboardDir() == 1) or (k == 'right' and getKeyboardDir() == -1):
#     setKeyboardDir(0)
#     print("Angle: " + str(getCurrentAngleInternalScale()))
#   if k == 'p' and pumpRunning:
#     pumpRunning = False
#     stopPump()
#     pumpingDuration = time.time() - pumpStartTime
#     print("Pumping time: " + str(pumpingDuration))
#   if key == keyboard.Key.esc:
#     return False
#   if k == 'k':
#     print("Keyboard test: on release")

#   return True

def getkey():
  old_settings = termios.tcgetattr(sys.stdin)
  tty.setcbreak(sys.stdin.fileno())
  try:
    while True:
      b = os.read(sys.stdin.fileno(), 3).decode()
      if len(b) == 3:
        k = ord(b[2])
      else:
        k = ord(b)
      key_mapping = {
          127: 'backspace',
          10: 'return',
          32: 'space',
          9: 'tab',
          27: 'esc',
          65: 'up',
          66: 'down',
          67: 'right',
          68: 'left'
      }
      return key_mapping.get(k, chr(k))
  finally:
      termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

# def on_press_dummy(key):
#   try:
#     k = key.char  # single-char keys
#   except:
#     k = key.name  # other keys
#   print(k + " pressed")
#   return True

if __name__ == '__main__':
  with portalocker.TemporaryFileLock(str(Path.home())+'/plant_guard/.lock', timeout=1800, fail_when_locked=False):
    pumpCrane = PumpCrane(False)

    # listener = keyboard.Listener(on_release=on_release)
    # listener.start()
    pumpCrane.setKeyboardListening(True)
    on_press('h')
    try:
      while True:
        k = getkey()
        if k == 'esc' or k == 'q':
          quit()
        else:
          on_press(k)
    except (KeyboardInterrupt, SystemExit):
      os.system('stty sane')
      print('stopping.')
    pumpCrane.setKeyboardListening(False)
    # listener.join()
    pumpCrane.cleanup()
