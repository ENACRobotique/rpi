#!/usr/bin/python3
import ld06_driver as lidar
import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher

import sys
import generated.lidar_data_pb2 as lidar_data

if __name__ == '__main__':
    #init ecal publisher 
    ecal_core.initialize(sys.argv, "eCAL_LD06_driver")

    pub = ProtoPublisher("lidar_data", lidar_data.Lidar)

    def publish_reading(angles, distances, quality):
        #once the program finished to read a full circle reading from lidar, publlish it to eCAL with protobuf format
        lidar_msg = lidar_data.Lidar()
        lidar_msg.nb_pts = len(distances)
        lidar_msg.angles.extend(angles)
        lidar_msg.distances.extend(distances)
        lidar_msg.quality.extend(quality)
        pub.send(lidar_msg, ecal_core.getmicroseconds()[1])

    driver = lidar.Driver(publish_reading, '/dev/ttyUSB0')
    driver.scan()
