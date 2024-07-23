#!/usr/bin/env python3

from pump_crane import PumpCrane
import portalocker
import yaml
from sys import argv
from pathlib import Path
import os

if __name__ == '__main__':
  with portalocker.TemporaryFileLock(str(Path.home())+'/plant_guard/.lock', timeout=1800, fail_when_locked=False):
    pumpCrane = PumpCrane(True)
    try:
      with open(os.path.join(Path.home(), 'plant_guard', 'jobs.yaml')) as file:
        jobsData = yaml.full_load(file)
      job = jobsData[argv[1]]
      angleFrom = float(job["angle_from"])
      angleTo = float(job["angle_to"])
      pumpDuration = float(job["pump_dur"])
    except:
      print("Error loading job")
      exit(1)
    print("watering from "+str(angleFrom)+" to "+str(angleTo)+" dur "+str(pumpDuration))
    pumpCrane.water(angleFrom, angleTo, pumpDuration)
    pumpCrane.cleanup()
    
