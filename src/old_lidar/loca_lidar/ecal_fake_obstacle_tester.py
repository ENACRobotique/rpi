import ld06_driver.ld06_driver as lidar
import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '../..')) # Avoids ModuleNotFoundError when finding generated folder
import generated.lidar_data_pb2 as lidar_data

pub = ProtoPublisher("obstacles_wrt_table", lidar_data.Obstacles)

