#!/bin/sh
### BEGIN INIT INFO
# Provides: RaspberryPilot
# Required-Start: $all
# Required-Stop: $all
# Default-Start:  2 3 4 5
# Default-Stop: 0 1 6
# Description: RaspberryPilot
### END INIT INFO

sleep 10;
sudo  /home/pi/TestCode/FlyControler/RaspberryPilot &
