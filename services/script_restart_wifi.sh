#!/bin/bash

echo "Arrêt des services + Redemarrage du wifi..."
systemctl stop 'robot_*'
echo $(date) >> /home/pi/rpi2024/services/wifi_restarts.log
#netplan apply
sleep 2
systemctl start robot_start.service
echo "Wifi redemarré et service start lancé"
