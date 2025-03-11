# Plant guard

[![Click to watch](https://img.youtube.com/vi/59vS7RXK-lc/0.jpg)](https://www.youtube.com/watch?v=59vS7RXK-lc "Click to watch")

A simple plant watering machine.
Used parts:
  - Raspberry Pi 3 B+
  - Submersible Low Noise Motor Pump 3-6V
  - Motor driver l293d
  - Stepper motor 28BYJ-48 with driver
  - Silicon hose
  - Bucket
  - [3d-printed parts](models)
  - dupont cables, screws, ...

No sensors are used for now, you will need to set appropriate timing for the frequency and duration of the watering (Keep in mind the watering will be slower when the bucket is emptier).

Usage:
  1. Assemble using the datasheets of the components (GPIO pins for the pump driver are 5 (->IN1) and 6 (->IN2), GPIO pins for the stepper motor driver are 24, 23, 22, 27 (-> IN1, IN2, IN3, IN4)) and mount onto the bucket
  2. `ssh pi@raspberrypi.local`
  3. `git clone https://github.com/afrixs/plant-guard.git`
  4. `pip3 install python_crontab pyyaml portalocker`
  5. Set the arm to point directly outwards from the bucket
  6. `python3 plant-guard/pg_ws/src/pg_pump_crane/pg_pump_crane/pump_crane_keyboard.py`
  7. Use arrow keys to move the arm. Repeat the last arrow key stroke to stop. When the arm is above the plant being configured use 'f' key to set `angle_from`. Optionally move the arm a little bit and use 't' key to set `angle_to`. The arm will be moving between `angle_from` and `angle_to` during the watering so that water does not fall to a single spot only.
  8. Use 'p' key to start/stop the pump. Duration of pumping will be stored to `pump_dur`.
  9. Use 'Enter' key to create a watering job. Enter a name for the job and a crontab time entry
  10. The watering job is scheduled. Repeat 6.-8. for another plant
  11. Use 'l' key to list all jobs, 'd' key to delete a single job or 'x' key to delete all jobs
  12. Use 'q' or 'Esc' key to quit the job configuration client. Lock for controlling motors is released and scheduled jobs are free to run
  13. You can now use Ctrl+D to logout from ssh

Note: You may edit the jobs afterwards using `crontab -e` or `nano ~/plant_guard/jobs.yaml`.

Important note: don't turn the stepper motor manually after the first start or the configuration will be broken. If you need (or accidentally happen) to turn it, use `rm ~/plant_guard/save.txt; rm ~/plant_guard/save_backup.txt` and follow steps 5. - 12. to reconfigure (or turn the motor back to its original position, hoping the difference will not be critical).

Important note: Also when leaving the device to operate without your attendance for a longer time, prepare the place in a way that no firm or big leaves capable of blocking the movement of the arm will grow in its way.

First test was successful: Here are images of our plants before and after leaving the apartment for 9 days. I think they liked it

![t = 0](docs/photo_t_0days.jpg)
![t = 9 days](docs/photo_t_8days.jpg)

sudo raspi-config -> System Options -> Boot / Auto Login -> Console Autologin
sudo nano /etc/dphys-swapfile -> CONF_SWAPSIZE=4096, CONF_MAXSWAP=4096
sudo dphys-swapfile setup
mkdir .plant_guard
mkdir .ros
sudo ln -s /home/pi/.ros /root/.ros
sudo ln -s /home/pi/.plant_guard /root/.plant_guard
sudo ln -s /var/spool/cron/crontabs /root/.plant_guard_cron
tmp (TODO): nano .plant_guard/jobs.yaml
# add following 3 lines
pump_crane:
  __config: {}
  __type: pump_crane

sudo ./docker-build.bash
# or to run in background:
sudo apt install screen
./docker-build.bash 2>>/root/build_docker.err 1>>/root/build_docker.out"
sudo tail -f /root/build_docker.err
# sudo screen -S plant_guard_build -X quit
Note: `docker-*.bash` must be run with `sudo`

sudo apt install screen
sudo crontab -e -> @reboot screen -DmS plant_guard bash -i -c "/home/pi/plant-guard/docker-run.bash 1>>/root/plant_guard_control_out.log 2>>/root/plant_guard_control_err.log"
sudo screen -S plant_guard -X quit

server:
```
sudo yum update
sudo yum install docker
sudo systemctl enable docker
sudo crontab -e -> @reboot sudo docker run --init --rm --net=host -p 7447:7447/tcp -p 8000:8000/tcp eclipse/zenoh:0.5.0-beta.9
```