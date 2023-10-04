#!/bin/sh
### BEGIN INIT INFO
# Provides:          sys_rdy
# Required-Start:    
# Required-Stop:
# Default-Start:     2 3 4 5
# Default-Stop:
# Short-Description: System ready
# Description:       This script turn on System Status LED when system
#		     boot finish.
### END INIT INFO

echo none > /sys/class/leds/System\ Status/trigger
echo 255 > /sys/class/leds/System\ Status/brightness

cd /home/root
../kongo &

: exit 0
