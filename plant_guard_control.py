#!/usr/bin/env python3

#pip3 install python_crontab pyyaml portalocker
from plant_guard import *
from crontab import CronTab, CronSlices
import yaml
import portalocker
from pathlib import Path
import sys,tty,os,termios
import getpass
import re
# from pynput import keyboard

pumpStartTime = time.time()
pumpingDuration = 0
angleFrom = -1
angleTo = -1
pumpRunning = False

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
      with open(str(Path.home())+'/plant-guard/jobs.yaml') as file:
        jobsData = yaml.full_load(file)
    except:
      jobsData = {}
    if jobsData is None:
      jobsData = {}
    jobsData[name] = {
        'angle_from': angleFrom if angleFrom != -1 else angleTo,
        'angle_to': angleTo if angleTo != -1 else angleFrom,
        'pump_dur': pumpingDuration
      }
    with open(str(Path.home())+'/plant-guard/jobs.yaml', 'w') as file:
      yaml.dump(jobsData, file)
    
    print("Enter schedule\nminute hour day_of_month month day_of_week\nexample (every friday at 6AM and 6PM): 0 6,18 * * Fri)")
    schedule = input()
    while not CronSlices.is_valid(schedule):
      print("Invalid schedule")
      schedule = input()
    cron = CronTab(user=getpass.getuser())
    job = cron.new(command='python3 ~/plant-guard/plant_guard_job.py '+name, comment='plant-guard-'+name)
    job.setall(schedule)
    cron.write()
    print("Added job "+name+"  "+schedule+"  "+str(jobsData[name]))

  # try:
  #   k = key.char  # single-char keys
  # except:
  #   k = key.name  # other keys
  if k == 'l':
    print("Plant-guard jobs:")
    cron = CronTab(user=getpass.getuser())
    try:
      with open(str(Path.home())+'/plant-guard/jobs.yaml') as file:
        jobsData = yaml.full_load(file)
    except:
      jobsData = {}
    if jobsData is None:
      jobsData = {}
    for job in cron.find_comment(re.compile("plant-guard-*")):
      name = job.comment[len("plant-guard-"):]
      print(name+"  "+str(job.slices)+"  "+(str(jobsData[name]) if name in jobsData else "No data!"))
  if k == 'd':
    print("Enter the name of the job to be deleted")
    name = input()
    cron = CronTab(user=getpass.getuser())
    if cron.remove_all(comment="plant-guard-"+name) == 0:
      print("No job found")
    else:
      print("Deleted job")
    cron.write()

    try:
      with open(str(Path.home())+'/plant-guard/jobs.yaml') as file:
        jobsData = yaml.full_load(file)
    except:
      jobsData = {}
    if jobsData is None:
      jobsData = {}
    if name in jobsData:
      del jobsData[name]
      with open(str(Path.home())+'/plant-guard/jobs.yaml', 'w') as file:
        yaml.dump(jobsData, file)
      print("Deleted data")
    else:
      print("No data found")
  if k == 'x':
    cron = CronTab(user=getpass.getuser())
    if cron.remove_all(comment=re.compile("plant-guard-*")) == 0:
      print("No job found")
    else:
      print("Deleted all jobs")
    cron.write()
    with open(str(Path.home())+'/plant-guard/jobs.yaml', 'w') as file:
      pass
    print("Deleted job data")

  if k == 'left':
    setKeyboardDir(0 if getKeyboardDir() == 1 else 1)
  if k == 'right':
    setKeyboardDir(0 if getKeyboardDir() == -1 else -1)
  if k == 'p':
    if pumpRunning:
      pumpRunning = False
      stopPump()
      pumpingDuration = time.time() - pumpStartTime
      print("Pumping time: " + str(pumpingDuration))
    else:
      print("Starting pump")
      pumpRunning = True
      startPump()
      pumpStartTime = time.time()

  if k == 'f':
    angleFrom = getCurrentAngle()
    print("Angle from: " + str(angleFrom))
  if k == 't':
    angleTo = getCurrentAngle()
    print("Angle to: " + str(angleTo))

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
#     print("Angle: " + str(getCurrentAngle()))
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
  with portalocker.TemporaryFileLock(str(Path.home())+'/plant-guard/.lock', timeout=1800, fail_when_locked=False):
    init(False)

    # listener = keyboard.Listener(on_release=on_release)
    # listener.start()
    setKeyboardListening(True)
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
    setKeyboardListening(False)
    # listener.join()
    cleanup()
