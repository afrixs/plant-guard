import RPi.GPIO as GPIO
import time
from threading import Lock, Thread
from pathlib import Path

lock = Lock()
keyboardListening = False
keyboardDir = 0
movementTerminated = False
rikt = []

PUMP_PIN = 5
PUMP_REV_PIN = 6

def init(exitOnLoadFailure):
  GPIO.setmode(GPIO.BCM)
  initPump()
  initStepper(exitOnLoadFailure)

def cleanup():
  GPIO.cleanup()

def initPump():
  GPIO.setup(PUMP_PIN, GPIO.OUT)
  GPIO.output(PUMP_PIN, GPIO.LOW)
  GPIO.setup(PUMP_REV_PIN, GPIO.OUT)
  GPIO.output(PUMP_REV_PIN, GPIO.LOW)
  pass

def startPump():
  print("pump: 1")
  GPIO.output(PUMP_PIN, GPIO.HIGH)

def stopPump():
  print("pump: 0")
  GPIO.output(PUMP_PIN, GPIO.LOW)


STEPPER_PINS = [24, 23, 22, 27]
STEPS = [[1,0,0,0],[1,1,0,0],[0,1,0,0],[0,1,1,0],[0,0,1,0],[0,0,1,1],[0,0,0,1],[1,0,0,1]]
currentStep = 0
currentAngle = 0
fullRot = 512*8

stepperSleepDur = 0.005

def computeChecksum():
  return 0xFFFFFFFF - (currentStep + currentAngle)

def saveFilePath(backup):
  return str(Path.home())+'/plant-guard/save'+('_backup.txt' if backup else '.txt')

def readSavedData(backup):
  global currentStep
  global currentAngle
  try:
    with open(saveFilePath(backup)) as file:
      currentStep = int(file.readline())
      currentAngle = int(file.readline())
      checksum = int(file.readline())
      if checksum != computeChecksum():
        raise
  except:
    return False
  return True

def writeSavedData(backup):
  with open(saveFilePath(backup), 'w') as file:
    file.writelines([str(currentStep)+'\n', str(currentAngle)+'\n', str(computeChecksum())+'\n'])

def initStepper(exitOnLoadFailure):
  if not readSavedData(False) and not readSavedData(True):
    if exitOnLoadFailure:
      print("ERROR: No saved state file found")
      exit(-1)
    else:
      print("WARNING: No saved state file found. This is ok if running for the first time")

  for p in STEPPER_PINS:
    GPIO.setup(p, GPIO.OUT)
  updateStepper()

def updateStepper():
  # print(STEPS[currentStep])
  writeSavedData(False)
  writeSavedData(True)
  for (p, v) in zip(STEPPER_PINS, STEPS[currentStep]):
    GPIO.output(p, GPIO.HIGH if v == 1 else GPIO.LOW)

def degAngle(deg):
  return round(deg*512.0/45.0)

def rotateSteps(stepsCnt):
  global currentStep
  global currentAngle
  with lock:
    for _ in range(abs(stepsCnt)):
      if stepsCnt > 0:
        currentStep = (currentStep+1)%len(STEPS)
        currentAngle = (currentAngle+1)%fullRot
      else:
        currentStep = (currentStep - 1 + len(STEPS))%len(STEPS)
        currentAngle = (currentAngle - 1 + fullRot)%fullRot
      updateStepper()
      time.sleep(stepperSleepDur)
      if movementTerminated:
        break

def rotateTo(angle):
  global currentAngle

  while angle >= fullRot:
    angle -= fullRot
  while angle < 0:
    angle += fullRot
  
  with lock:
    stepsCnt = angle - currentAngle
  while stepsCnt > fullRot/2: # 'if' should be good enough...
    stepsCnt -= fullRot
  while stepsCnt < -fullRot/2: # 'if' should be good enough...
    stepsCnt += fullRot
  #don't rotate through 360 degrees because of hose and (future) cables
  if currentAngle <= fullRot/2 and angle > fullRot/2 and stepsCnt > 0:
    stepsCnt -= fullRot
  elif currentAngle > fullRot/2 and angle <= fullRot/2 and stepsCnt < 0:
    stepsCnt += fullRot
  
  rotateSteps(stepsCnt)

def patrol(angle1, angle2):
  if angle1 == angle2:
    return
  while not movementTerminated:
    rotateTo(angle2)
    if not movementTerminated:
      rotateTo(angle1)

def water(angle1, angle2, dur):
  global movementTerminated
  rotateTo(angle1)
  startPump()
  movementTerminated = False
  t = Thread(target=patrol, args=(angle1, angle2))
  t.start()
  time.sleep(dur)
  movementTerminated = True
  t.join()
  movementTerminated = False
  stopPump()
  time.sleep(1.0)

def setKeyboardListening(kl):
  global keyboardListening
  global rikt
  if keyboardListening != kl:
    keyboardListening = kl
    if kl:
      rikt = Thread(target=rotateIfKey)
      rikt.start()
    else:
      rikt.join()

def setKeyboardDir(kd):
  global keyboardDir
  keyboardDir = kd
def getKeyboardDir():
  return keyboardDir

def getCurrentAngle():
  return currentAngle

def rotateIfKey():
  global currentStep
  global currentAngle
  while keyboardListening:
    if keyboardDir != 0:
      with lock:
        if keyboardDir > 0:
          currentStep = (currentStep+1)%len(STEPS)
          currentAngle = (currentAngle+1)%fullRot
        else:
          currentStep = (currentStep - 1 + len(STEPS))%len(STEPS)
          currentAngle = (currentAngle - 1 + fullRot)%fullRot
        updateStepper()
    time.sleep(stepperSleepDur)