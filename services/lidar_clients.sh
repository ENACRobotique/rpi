#!/bin/bash

sp_old="/home/pi/rpi2024/src/old_lidar"
sp="/home/pi/rpi2024/src/lidar"

#python3 $sp_old/ecal_loca_lidar.py &
#python3 $sp_old/ecal_pos_fusion.py &
python3 $sp/amalgameur.py &
