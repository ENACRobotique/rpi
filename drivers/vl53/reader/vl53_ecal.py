#!/usr/bin/python3
import socket
import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
from ecal.core.subscriber import ProtoSubscriber
import time
import sys
sys.path.append('/home/pi/rpi2024/generated/')
import lidar_data_pb2 as lidar_pb
import subprocess
import os
from dataclasses import dataclass
from enum import Enum
from threading import Thread
import fcntl
import os

PORTS = [2001, 2002, 2003, 2004]

UDP_IP = "127.0.0.1"

READER_PATH = os.path.abspath("reader")


@dataclass
class VL53:
    nb: int
    port: int
    socket: socket.socket
    pub: ProtoPublisher
    terminate: bool


def non_block_readline(output) -> bytes:
    fd = output.fileno()
    fl = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
    try:
        return output.readline().strip(b'\n')
    except:
        return b''



def handle_sensor(s: VL53):
    while not s.terminate:
        data, addr = s.socket.recvfrom(1024) # buffer size is 1024 bytes
        dist = [int.from_bytes(data[offset:offset+2], 'little') for offset in range(0, len(data), 2)]
        msg = lidar_pb.Lidar(angles=[i for i in range(64)], distances=dist, nb_pts=64)
        s.pub.send(msg)


if __name__ == '__main__':
    print("VL53 Ecal")
    ecal_core.initialize(sys.argv, "vl53bridge")

    p = subprocess.Popen([READER_PATH], stdout=subprocess.PIPE)

    sensors: list[VL53] = []

    for i, port in enumerate(PORTS):
        pub = ProtoPublisher(f"vl53_{i+1}", lidar_pb.Lidar)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, port))
        sensors.append(VL53(i, port, sock, pub, False))

    
    print("launching threads...")
    for s in sensors:
        thread = Thread(target=handle_sensor, args=(s,))
        print(f"launching sensor {s.nb} thread.")
        thread.start()

    while True:
        time.sleep(2)

#reader_proc.terminate()

