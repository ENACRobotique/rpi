#!/bin/bash

sp="/home/pi/rpi2024/services"

sudo rm /etc/systemd/system/robot_*

sudo ln -s $sp/robot_bridge.service 		/etc/systemd/system/robot_bridge.service
sudo ln -s $sp/robot_lidar_driver.service 	/etc/systemd/system/robot_lidar_driver.service
sudo ln -s $sp/robot_lidar_clients.service 	/etc/systemd/system/robot_lidar_clients.service
sudo ln -s $sp/robot_lcd.service 		/etc/systemd/system/robot_lcd.service
#sudo ln -s $sp/robot_lidar_fusion.service 	/etc/systemd/system/robot_lidar_fusion.service
#sudo ln -s $sp/robot_lidar_loca.service 	/etc/systemd/system/robot_lidar_loca.service
#sudo ln -s $sp/robot_optitrack.service 		/etc/systemd/system/robot_optitrack.service
sudo ln -s $sp/robot_record.service 		/etc/systemd/system/robot_record.service
sudo ln -s $sp/robot_start.service 		/etc/systemd/system/robot_start.service
sudo ln -s $sp/robot_strat.service 		/etc/systemd/system/robot_strat.service
sudo ln -s $sp/robot_vl53.service 		/etc/systemd/system/robot_vl53.service
sudo ln -s $sp/robot_IO.service 		/etc/systemd/system/robot_IO.service
sudo ln -s $sp/robot_aruco.service 		/etc/systemd/system/robot_aruco.service

#sudo systemctl enable robot_bridge.service
#sudo systemctl enable robot_lidar_driver.service
#sudo systemctl enable robot_lidar_fusion.service
#sudo systemctl enable robot_lidar_loca.service
#sudo systemctl enable robot_optitrack.service
#sudo systemctl enable robot_record.service
sudo systemctl enable robot_start.service
#sudo systemctl enable robot_strat.service
#sudo systemctl enable robot_vl53.service
#sudo systemctl enable robot_IO.service
#sudo systemctl enable robot_aruco.service

sudo systemctl daemon-reload

