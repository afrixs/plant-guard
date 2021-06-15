#!/usr/bin/env python3

from plant_guard import *
import portalocker
import yaml
from sys import argv
from pathlib import Path

if __name__ == '__main__':
  with portalocker.TemporaryFileLock(str(Path.home())+'/plant-guard/.lock', timeout=1800, fail_when_locked=False):
    init(True)
    try:
      with open(str(Path.home())+'/plant-guard/jobs.yaml') as file:
        jobsData = yaml.full_load(file)
      job = jobsData[argv[1]]
      angleFrom = int(job["angle_from"])
      angleTo = int(job["angle_to"])
      pumpDuration = float(job["pump_dur"])
    except:
      print("Error loading job")
      exit(1)
    print("watering from "+str(angleFrom)+" to "+str(angleTo)+" dur "+str(pumpDuration))
    water(angleFrom, angleTo, pumpDuration)
    cleanup()
    
