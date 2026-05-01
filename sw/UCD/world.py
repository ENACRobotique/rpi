#!/usr/bin/env python3
import cv2
import numpy as np
import sys, os

sys.path.append(os.path.join(os.path.dirname(__file__), '../..')) # Avoids ModuleNotFoundError when finding generated folder
import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Subscriber as ProtoSubscriber
from ecal.msg.proto.core import Publisher as ProtoPublisher
from ecal.msg.common.core import ReceiveCallbackData
from generated.robot_state_pb2 import Aruco, Arucos
# from generated.robot_state_pb2 import Position_aruco
# from generated import CompressedImage_pb2 as cipb
from google.protobuf.timestamp_pb2 import Timestamp
import argparse
from enum import Enum
from scipy.spatial.transform import Rotation
import time
from threading import Event



class Zone:
    def __init__(nhh, xmin, xmax, ymin, ymax, name ):
        nhh.xmin = xmin 
        nhh.xmax = xmax
        nhh.ymin = ymin 
        nhh.ymax = ymax
        nhh.name = name

    def contains(nhh, x, y):
        return (x >= nhh.xmin) and (x<= nhh.xmax) and (y <= nhh.ymax) and (y >= nhh.ymin)


class ZoneManager:
    def __init__(self, zones):
        self.zones = zones  # dict {name: Zone}

    def analyze(self, arucos):
        results = {}

        for name, zone in self.zones.items():
            in_zone = [a for a in arucos if zone.contains(a.x, a.y)]

            count = len(in_zone)

            # Comptage par ID
            id_counts = {}
            for a in in_zone:
                id_counts[a.ArucoId] = id_counts.get(a.ArucoId, 0) + 1

            # Flag : au moins 3 de même ID
            has_3_same = any(v >= 3 for v in id_counts.values())

            results[name] = {
                "count": count,
                "arucos": in_zone,
                "id_counts": id_counts,
                "has_3_same": has_3_same
            }

        return results


class World:
    def __init__(self):

        if not ecal_core.is_initialized():
            ecal_core.initialize("aruco_finder")

        self.arucos = []


        self.arucosReportSub = ProtoSubscriber(Arucos, "Arucos_world")
        self.arucosReportSub.set_receive_callback(self.onReceiveArucos)

        self.zone_manager = ZoneManager({
            "FrigoJE": Zone(600, 1000, 600, 1000, "FrigoJE"),
            "FrigoJW": Zone(-100, 300, 200, 600, "FrigoJW"),
            "FrigoJS": Zone(500, 900, -100, 300, "FrigoJS"),
            "FrigoMidN": Zone(1300, 1700, 600, 1000, "FrigoMidN"),
            "FrigoMidS": Zone(1300, 1700, -100, 300, "FrigoMidS"),
            "FrigoBS": Zone(2100, 2500, -100, 300, "FrigoBS"),
            "FrigoBE": Zone(2700, 3100, 600, 1000, "FrigoBE"),
            "FrigoJS": Zone(500, 900, -100, 300, "FrigoJS")

        })


    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.sub.remove_receive_callback()


    def onReceiveArucos (self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[Arucos]):
        """Callback d'un subscriber ecal. Actualise la position du robot"""
        self.arucos  = [a for a in data.message.arucos if ((a.ArucoId == 36) or (a.ArucoId == 47))]

    def run(self):
        while True:
            if not self.arucos:
                time.sleep(0.1)
                continue

            results = self.zone_manager.analyze(self.arucos)

            for zone_name, res in results.items():
                print(f"\nZone: {zone_name}")
                print(f"Count: {res['count']}")
                print(f"Has 3 same: {res['has_3_same']}")

                for a in res["arucos"]:
                    print(f"  ID: {a.ArucoId} | x={a.x:.1f}, y={a.y:.1f}")

            time.sleep(2)


if __name__ == "__main__":
    with World() as af:
        af.run()#k
        time.sleep(2)

        

        
