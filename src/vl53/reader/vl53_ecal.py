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
import gpiozero
from dataclasses import dataclass
from enum import Enum
from threading import Thread
import fcntl
import os

BASE_PORT = 2000
GPIOS = [26, 16, 17, 18]

UDP_IP = "127.0.0.1"
INIT_TIMEOUT =10

READER_PATH = os.path.abspath("vl53_reader")



class SensorStatus(Enum):
    UNINIT = 0
    OK = 1
    ERROR = 2

@dataclass
class VL53:
    nb: int
    gpio: gpiozero.LED
    status: SensorStatus
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


def init_sensor(s: VL53):
    s.status = SensorStatus.ERROR
    s.gpio.off()   # turn sensor ON
    time.sleep(0.5)
    p = subprocess.Popen([READER_PATH, str(s.port), str(0x60+2*s.nb)], stdout=subprocess.PIPE)
    start = time.time()
    buf = b''
    while time.time() - start < INIT_TIMEOUT:
        line = non_block_readline(p.stdout)
        if line.strip() != b'':
            print(line.decode())
            if b'VL53L5CX Ready !' in line:
                print(f"sensor {s.nb} INITIALIZED!")
                s.status = SensorStatus.OK
                return
        ret = p.poll()
        if ret is not None:
            print(f"sensor {s.nb} init FAILED!")
            return


def handle_sensor(s: VL53):
    while s.status == SensorStatus.OK and not s.terminate:
        data, addr = s.socket.recvfrom(1024) # buffer size is 1024 bytes
        dist = [int.from_bytes(data[offset:offset+2], 'little') for offset in range(0, len(data), 2)]
        msg = lidar_pb.Lidar(angles=[i for i in range(64)], distances=dist, nb_pts=64)
        s.pub.send(msg)


if __name__ == '__main__':
    print("VL53 Ecal")
    ecal_core.initialize(sys.argv, "vl53bridge")

    sensors: list[VL53] = []

    for i, gpio in enumerate(GPIOS):
        i += 1
        rst = gpiozero.LED(gpio)
        rst.on() # turn off VL53
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        port = BASE_PORT + i
        sock.bind((UDP_IP, port))
        pub = ProtoPublisher(f"vl53_{i}", lidar_pb.Lidar)
        sensors.append(VL53(i, rst, SensorStatus.UNINIT, port, sock, pub, False))


    for s in sensors:
        init_sensor(s)
    
    print("launching threads...")
    for s in sensors:
        if s.status == SensorStatus.OK:
            thread = Thread(target=handle_sensor, args=(s,))
            print(f"launching sensor {s.nb} thread.")
            thread.start()

    while True:
        time.sleep(2)

#reader_proc.terminate()

