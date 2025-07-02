#!/usr/bin/env python3
from serial import Serial
import time
from collections import namedtuple
import numpy as np
import sys
import re
from dataclasses import dataclass
import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
import generated.lidar_data_pb2 as lidar_pb
from numpy import radians

NB_POINTS = 541

@dataclass
class Command:
    command:str
    answer_expected:bool
    answer:str


START_RANGING = Command('#GT15466', False, '')
STOP_RANGING = Command('#ST5297', True, '#ST00A845')
ID = Command('#IN0D54', True, '')
ID2 = Command('#CLC2DD', True, '')
PLOP = Command('#GR0EEE1', True, '')


class UST05LN:
    """
    UST05LN LIDAR driver class.
    """
    #Command = namedtuple('Command',['command', 'answer_expected', 'answer'])



    def __init__(self, port, topic):
        """
        Initialize the UST05LN driver.
        """
        ecal_core.initialize(sys.argv, "UST05LN LIDAR Driver")
        self.lidar_pub = ProtoPublisher(topic, lidar_pb.Lidar)
        self.ser = Serial(port, 115200)
        self.stop_ranging()
    
    def send_command(self, command: Command, timeout=2):
        self.ser.write(command.command.encode()+b'\n')    # writes command to LIDAR
        self.ser.reset_input_buffer()
        data = b''
        start_time = time.time()
        if command.answer_expected and command.answer != '':    #precise answer expected, search for it !
            while time.time() - start_time < timeout:
                if self.ser.in_waiting:
                    data+=self.ser.read()
                    data = data.split(b'\n')[-1]
                    if command.answer.encode() in data:
                        break
            return data
        elif command.answer_expected:   # answer expected but be don't known which : return the first one (until \n)
            while time.time() - start_time < timeout:
                if self.ser.in_waiting:
                    data+=self.ser.read()
                    if b'\n' in data:
                        data = data.split(b'\n')[0]
                        break
            return data
        else:
            return b''

    def stop_ranging(self):
        self.send_command(STOP_RANGING)

    def start_ranging(self):
        self.stop_ranging()
        self.send_command(START_RANGING)

    def stop(self):
        self.stop_ranging()
        self.ser.close()
    
    def get_measures(self):
        """
        returns measures under the form (timestamp, [(distance, quality), ...])
        timestamp : time in seconds since the LIDAR startup
        distance range : 0 - 65635
        valur range : 0 - ??? (65635 max)
        eg: (102.123456, [(552, 1244), (646, 1216), (676, 1270), ...])
        """
        raw_bytes=self.ser.readline().strip()
        mesb = raw_bytes[26:-4] #m.groups()[1]
        if len(mesb) != 4328:
            return
        distances = np.fromiter((int(mesb[i:i+4],16)  for i in range(0, len(mesb), 8)),dtype = np.uint16)
        puissances = np.fromiter((min(254, int(mesb[i:i+4],16)/4)  for i in range(4, len(mesb), 8)),dtype = np.uint16)
        angles = [radians(i*270/NB_POINTS-270/2) for i in range(NB_POINTS)]
        msg = lidar_pb.Lidar(distances=distances, angles=angles, quality=puissances, nb_pts=NB_POINTS, angle_increment=radians(270/NB_POINTS))
        self.lidar_pub.send(msg)


    def start(self):
        """
        Connect to the UST05LN LIDAR device.
        """
        pass


if __name__ == "__main__":
    topic = sys.argv[2] if len(sys.argv) > 2 else "lidar_data"
    ust = UST05LN(port = sys.argv[1], topic=topic)
    try:
        ret = ust.send_command(ID)
        print(ret)
        ust.start_ranging()
        while True:
            ust.get_measures()
    finally:
        ust.stop()
