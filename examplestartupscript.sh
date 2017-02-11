#!/bin/sh -e
# rc.local

_IP=$(hostname -I) || true
if [ "$_IP" ]; then
  printf "My IP address is %s\n" "$_IP"
fi

sudo ifdown wlan0
sudo ifup eth0

python3 /home/pi/seamonsters_git/CompetitionBot2017/grip/pi_script.py &
python3 /home/pi/seamonsters_git/CompetitionBot2017/grip/plain_camera.py &

exit 0