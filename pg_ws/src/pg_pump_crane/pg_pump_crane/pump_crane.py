import RPi.GPIO as GPIO
import time
import math
from threading import Lock, Thread, Event
from pathlib import Path

class PumpCrane:
  lock = Lock()
  keyboardListening = False
  movementDir = 0
  movementTerminated = Event()
  rikt = []

  PUMP_PIN = 5
  PUMP_REV_PIN = 6

  STEPPER_PINS = [24, 23, 22, 27]
  STEPS = [[1,0,0,0],[1,1,0,0],[0,1,0,0],[0,1,1,0],[0,0,1,0],[0,0,1,1],[0,0,0,1],[1,0,0,1]]
  currentStep = 0
  currentAngle = 0
  fullRot = 512*8

  stepperSleepDur = 0.005

  def __init__(self, exitOnLoadFailure, stepperPosCallback=lambda pos: None, pumpCallback=lambda pumping: None):
    self.stepperPosCallback = stepperPosCallback
    self.pumpCallback = pumpCallback
    if not self.readSavedData(False) and not self.readSavedData(True):
      if exitOnLoadFailure:
        print("ERROR: No saved state file found")
        exit(-1)
      else:
        print("WARNING: No saved state file found. This is ok if running for the first time")
    self.initHW()

  def cleanup(self):
    self.cleanupHW()

  def initHW(self):
    GPIO.setmode(GPIO.BCM)
    self.initPump()
    self.initStepper()

  def cleanupHW(self):
    GPIO.cleanup()

  def initPump(self):
    GPIO.setup(self.PUMP_PIN, GPIO.OUT)
    GPIO.output(self.PUMP_PIN, GPIO.LOW)
    GPIO.setup(self.PUMP_REV_PIN, GPIO.OUT)
    GPIO.output(self.PUMP_REV_PIN, GPIO.LOW)
    pass

  def startPump(self):
    GPIO.output(self.PUMP_PIN, GPIO.HIGH)
    self.pumpCallback(True)

  def stopPump(self):
    GPIO.output(self.PUMP_PIN, GPIO.LOW)
    self.pumpCallback(False)

  def computeChecksum(self):
    return 0xFFFFFFFF - (self.currentStep + self.currentAngle)

  def saveFilePath(self, backup):
    return str(Path.home())+'/plant_guard/save'+('_backup.txt' if backup else '.txt')

  def readSavedData(self, backup):
    try:
      with open(self.saveFilePath(backup)) as file:
        self.currentStep = int(file.readline())
        self.currentAngle = int(file.readline())
        checksum = int(file.readline())
        if checksum != self.computeChecksum():
          raise
    except:
      return False
    return True

  def writeSavedData(self, backup):
    with open(self.saveFilePath(backup), 'w') as file:
      file.writelines([str(self.currentStep)+'\n', str(self.currentAngle)+'\n', str(self.computeChecksum())+'\n'])

  def initStepper(self):
    for p in self.STEPPER_PINS:
      GPIO.setup(p, GPIO.OUT)
    self.updateStepper()

  def updateStepper(self):
    self.writeSavedData(False)
    self.writeSavedData(True)
    self.stepperPosCallback(self.getCurrentAngle())
    for (p, v) in zip(self.STEPPER_PINS, self.STEPS[self.currentStep]):
      GPIO.output(p, GPIO.HIGH if v == 1 else GPIO.LOW)

  def degAngle(self, deg):
    return round(deg*512.0/45.0)

  def rotateSteps(self, stepsCnt):
    with self.lock:
      for _ in range(abs(stepsCnt)):
        if stepsCnt > 0:
          self.currentStep = (self.currentStep+1)%len(self.STEPS)
          self.currentAngle = (self.currentAngle+1)%self.fullRot
        else:
          self.currentStep = (self.currentStep - 1 + len(self.STEPS))%len(self.STEPS)
          self.currentAngle = (self.currentAngle - 1 + self.fullRot)%self.fullRot
        self.updateStepper()
        time.sleep(self.stepperSleepDur)
        if self.movementTerminated.is_set():
          break

  def rotateTo(self, angle):
    while angle >= self.fullRot:
      angle -= self.fullRot
    while angle < 0:
      angle += self.fullRot

    with self.lock:
      stepsCnt = angle - self.currentAngle
    while stepsCnt > self.fullRot/2:  # 'if' is good enough since angle and currentAngle are in [0, 2pi)
      stepsCnt -= self.fullRot
    while stepsCnt < -self.fullRot/2:  # 'if' is good enough since angle and currentAngle are in [0, 2pi)
      stepsCnt += self.fullRot
    # don't rotate through 360 degrees because of hose and (future) cables
    if self.currentAngle <= self.fullRot/2 and angle > self.fullRot/2 and stepsCnt > 0:
      stepsCnt -= self.fullRot
    elif self.currentAngle > self.fullRot/2 and angle <= self.fullRot/2 and stepsCnt < 0:
      stepsCnt += self.fullRot

    self.rotateSteps(stepsCnt)

  def patrol(self, angle1, angle2):
    if angle1 == angle2:
      return
    while not self.movementTerminated.is_set():
      self.rotateTo(angle2)
      if not self.movementTerminated.is_set():
        self.rotateTo(angle1)

  def allowMovement(self):
    self.movementTerminated.clear()

  def terminateMovement(self):
    self.movementTerminated.set()

  def water(self, angle1, angle2, dur, auto_allow=True):
    if auto_allow:
      self.movementTerminated.clear()
    angle1 = self.toInternalAngleScale(angle1)
    angle2 = self.toInternalAngleScale(angle2)
    print("watering from "+str(angle1)+" to "+str(angle2)+" dur "+str(dur))
    self.rotateTo(angle1)
    if not self.movementTerminated.is_set():
      self.startPump()
      t = Thread(target=self.patrol, args=(angle1, angle2))
      t.start()
      self.movementTerminated.wait(dur)
      self.movementTerminated.set()
      t.join()
    self.movementTerminated.clear()
    self.stopPump()
    time.sleep(1.0)

  def setKeyboardListening(self, kl):
    if self.keyboardListening != kl:
      self.keyboardListening = kl
      if kl:
        self.rikt = Thread(target=self.rotateIfKey)
        self.rikt.start()
      else:
        self.rikt.join()

  def setMovementDir(self, kd):
    self.movementDir = kd
  def getMovementDir(self):
    return self.movementDir

  def getCurrentAngleInternalScale(self):
    return self.currentAngle

  def getCurrentAngle(self):
    return (self.currentAngle - (self.fullRot if self.currentAngle > self.fullRot/2 else 0.0))/self.fullRot*(math.pi*2)

  def toInternalAngleScale(self, angle):
    a = round(angle/(math.pi*2)*self.fullRot)
    return a + (self.fullRot if a < 0 else 0)

  def rotateIfKey(self):
    while self.keyboardListening:
      if self.movementDir != 0:
        with self.lock:
          if self.movementDir > 0:
            self.currentStep = (self.currentStep+1)%len(self.STEPS)
            self.currentAngle = (self.currentAngle+1)%self.fullRot
          else:
            self.currentStep = (self.currentStep - 1 + len(self.STEPS))%len(self.STEPS)
            self.currentAngle = (self.currentAngle - 1 + self.fullRot)%self.fullRot
          self.updateStepper()
      time.sleep(self.stepperSleepDur)