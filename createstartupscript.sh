#!/bin/sh

echo "#!/bin/sh -e" > /etc/rc.local
echo "# rc.local\n" >> /etc/rc.local
echo "_IP=$(hostname -I) || true" >> /etc/rc.local
echo "if [ \"$_IP\" ]; then" >> /etc/rc.local
echo "  printf \"My IP address is %s\\n\" \"$_IP\"" >> /etc/rc.local
echo "fi" >> /etc/rc.local
echo "sudo ifdown wlan0" >> /etc/rc.local
echo "sudo ifup eth0\n" >> /etc/rc.local
echo "python3 /home/pi/seamonsters_git/CompetitionBot2017/grip/filter_test_contours.py peg_filter_real &" >> /etc/rc.local
echo "python3 /home/pi/seamonsters_git/CompetitionBot2017/grip/plain_camera.py &" >> /etc/rc.local
echo "exit 0" >> /etc/rc.local
