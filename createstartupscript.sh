#!/bin/sh

echo -e "#!/bin/sh -e\n_IP=$(hostname -I) || true\nif [ "$_IP" ]; then\n  printf "My IP address is %s\n" "$_IP"\nfi\n\n" > /etc/rc.local
echo -e "sudo ifdown wlan0\nsudo ifup eth0\n" >> /etc/rc.local
echo -e "python3 /home/pi/seamonsters_git/CompetitionBot2017/grip/filter_test_contours.py peg_filter_real &\n" >> /etc/rc.local
echo -e "python3 /home/pi/seamonsters_git/CompetitionBot2017/grip/plain_camera.py &\nexit 0" >> /etc/rc.local
