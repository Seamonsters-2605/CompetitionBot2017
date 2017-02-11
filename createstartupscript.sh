#!/bin/sh

echo "#!/bin/sh -e\n" > /etc/rc.local
echo "# rc.local\n\n" >> /etc/rc.local
echo "_IP=$(hostname -I) || true\n" >> /etc/rc.local
echo "if [ "$_IP" ]; then\n" >> /etc/rc.local
echo "  printf "My IP address is %s\n" "$_IP"\n" >> /etc/rc.local
echo "fi\n\n" >> /etc/rc.local
echo "sudo ifdown wlan0\n" >> /etc/rc.local
echo "sudo ifup eth0\n\n" >> /etc/rc.local
echo "python3 /home/pi/seamonsters_git/CompetitionBot2017/grip/filter_test_contours.py peg_filter_real &\n" >> /etc/rc.local
echo "python3 /home/pi/seamonsters_git/CompetitionBot2017/grip/plain_camera.py &\n" >> /etc/rc.local
echo "exit 0" >> /etc/rc.local
