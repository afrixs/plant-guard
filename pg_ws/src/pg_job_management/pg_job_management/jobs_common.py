import os
from pathlib import Path
from pg_msgs.msg import Device

JOB_CRON_PREFIX = "plant_guard."
PG_HOME = os.path.join(Path.home(), 'plant_guard')
JOBS_YAML_PATH = os.path.join(PG_HOME, 'jobs.yaml')
ALLOWED_NAMING_PATTERN = "(?!^__)[a-zA-Z0-9_ -]*"
DEVICE_TYPES = {Device.TYPE_PUMP_CRANE}
