#!/bin/bash
echo "Remapping RPLIDAR to /dev/rplidar..."
sudo cp rplidar.rules /etc/udev/rules.d/
sudo service udev reload
sudo sleep 2
sudo service udev restart
echo "finish"
