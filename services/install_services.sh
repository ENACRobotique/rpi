#!/bin/bash

mkdir -p ~/.config/systemd/user/

rm ~/.config/systemd/user/robot_*


for f in \
robot_bridge \
robot_lidar_driver \
robot_lidar_amalgameur \
robot_lidar_loca \
robot_strat \
robot_vl53 \
robot_IO \
robot_joystick \
robot_ui \
robot_tirette \
robot_aruco2 \
robot_aruco \
robot_UCD \
robot_ekf ; do 
    ln -s ~/rpi/services/$f.service ~/.config/systemd/user/$f.service
done

ln -s ~/rpi/services/robot_start.target ~/.config/systemd/user/robot_start.target

systemctl --user enable robot_start.target

systemctl --user daemon-reload

# start user.default target at startup
loginctl enable-linger $(whoami)
