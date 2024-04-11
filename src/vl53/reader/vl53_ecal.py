import socket
import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
from ecal.core.subscriber import ProtoSubscriber
import time
import sys
sys.path.append('/home/pi/rpi2024/generated/')
import lidar_data_pb2 as lidar_pb


UDP_IP = "127.0.0.1"
UDP_PORT = 2000

ecal_core.initialize(sys.argv, "vl53bridge")
dist_pub = ProtoPublisher("vl53", lidar_pb.Lidar)
time.sleep(1)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    dist = [int.from_bytes(data[offset:offset+2], 'little') for offset in range(0, len(data), 2)]
    msg = lidar_pb.Lidar(angles=[i for i in range(64)], distances=dist, nb_pts=64)
    dist_pub.send(msg)

    print("distances:", dist)



