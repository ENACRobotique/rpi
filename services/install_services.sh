#!/bin/bash

[ $UID != 0 ] && echo A lancer avec sudo && exit 42

sp="/home/pi/rpi2024/services"

rm /etc/systemd/system/robot_*
rm /etc/systemd/system/pigpiod.service

for f in \
robot_bridge \
robot_lidar_driver \
robot_lidar_clients \
robot_lcd \
robot_record \
robot_start \
robot_strat \
robot_vl53 \
robot_IO \
# robot_optitrack \
robot_aruco ; do 
    ln -s $sp/$f.service 		/etc/systemd/system/$f.service
done

ln -s $sp/pigpiod.service                  /etc/systemd/system/pigpiod.service

systemctl enable robot_start.service

systemctl daemon-reload

