#!/bin/bash

mkdir -p ~/.config/systemd/user/

rm ~/.config/systemd/user/robot_*

# autre services:
# robot_optitrack

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
robot_joystick \
robot_aruco ; do 
    ln -s ~/rpi2024/services/$f.service ~/.config/systemd/user/$f.service
done


systemctl --user enable robot_start.service

systemctl --user daemon-reload

# start user.default target at startup
loginctl enable-linger $(whoami)

sudo rm /etc/systemd/system/pigpiod.service
sudo ln -s ~/rpi2024/services/pigpiod.service /etc/systemd/system/pigpiod.service
sudo systemctl daemon-reload

