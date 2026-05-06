#!/usr/bin/env python3
import sys, os
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '../..')) # Avoids ModuleNotFoundError when finding generated folder
import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Subscriber as ProtoSubscriber
from ecal.msg.common.core import ReceiveCallbackData
from generated.robot_state_pb2 import Arucos

import matplotlib.pyplot as plt
import matplotlib.patches as patches


# ---------------- ZONES ----------------

class Zone:
    def __init__(self, xmin, xmax, ymin, ymax, name):
        self.xmin = xmin
        self.xmax = xmax
        self.ymin = ymin
        self.ymax = ymax
        self.name = name

    def contains(self, x, y):
        return (self.xmin <= x <= self.xmax) and (self.ymin <= y <= self.ymax)


class ZoneManager:
    def __init__(self, zones):
        self.zones = zones

    def analyze(self, arucos):
        results = {}

        for name, zone in self.zones.items():
            in_zone = [a for a in arucos if zone.contains(a.x, a.y)]
            count = len(in_zone)

            id_counts = {}
            for a in in_zone:
                id_counts[a.ArucoId] = id_counts.get(a.ArucoId, 0) + 1

            has_3_same = any(v >= 3 for v in id_counts.values())

            results[name] = {
                "count": count,
                "arucos": in_zone,
                "has_3_same": has_3_same
            }

        return results


# ---------------- WORLD ----------------

class World:
    def __init__(self):

        if not ecal_core.is_initialized():
            ecal_core.initialize("aruco_finder")

        self.arucos = []

        self.sub = ProtoSubscriber(Arucos, "Arucos_world")
        self.sub.set_receive_callback(self.onReceiveArucos)

        # Zones rangement
        self.zone_rangement = ZoneManager({
            "FrigoJE": Zone(600, 1000, 600, 1000, "FrigoJE"),
            "FrigoJW": Zone(-100, 300, 600, 1000, "FrigoJW"),
            "FrigoJS": Zone(500, 900, -100, 300, "FrigoJS"),
            "FrigoJN": Zone(1100, 1400, 1300, 1600, "FrigoJN"),
            "FrigoMidN": Zone(1300, 1700, 600, 1000, "FrigoMidN"),
            "FrigoMidS": Zone(1300, 1700, -100, 300, "FrigoMidS"),
            "FrigoBS": Zone(2100, 2500, -100, 300, "FrigoBS"),
            "FrigoBE": Zone(2700, 3100, 600, 1000, "FrigoBE"),
            "FrigoBW": Zone(2000, 2400, 600, 1000, "FrigoJS"),
            "FrigoBN": Zone(1600, 1900, 1300, 1600,"FrigoBN"), 
        })

        # Zones collecte
        self.zone_collecte = ZoneManager({
            "NoixJN": Zone(50, 300, 1050, 1350,"NoixJN"),
            "NoixJSW": Zone(50, 300, 250, 550, "NoixJSW"),  
            "NoixJE": Zone(1000, 1300, 675, 925,"NoixJE"),
            "NoixJSE": Zone(950, 1250, 50, 300, "NoixJSE"),
            "NoixBN": Zone(2700, 2950, 1050, 1350,"NoixBN"),
            "NoixBSE": Zone(2700, 2950, 250, 550,"NoixBSE"),
            "NoixBW": Zone(1700, 2000,675, 925, "NoixBW"),
            "NoixBSW": Zone(1750, 2050, 50, 300, "NoixBSW")
        })

        # ----------- PLOT -----------
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-200, 3200)
        self.ax.set_ylim(-200, 1800)
        self.ax.set_title("Carte des ArUco")

        self.patches = {}
        self.texts = {}

        self.init_plot()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.sub.remove_receive_callback()

    # ----------- CALLBACK -----------

    def onReceiveArucos(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[Arucos]):
        self.arucos  = [a for a in data.message.arucos if ((a.ArucoId == 36) or (a.ArucoId == 47))]

    # ----------- VISU -----------

    def init_plot(self):
        # Zones rangement
        for name, zone in self.zone_rangement.zones.items():
            rect = patches.Rectangle(
                (zone.xmin, zone.ymin),
                zone.xmax - zone.xmin,
                zone.ymax - zone.ymin,
                linewidth=2,
                edgecolor='black',
                facecolor='blue',
                alpha=0.5
            )
            self.ax.add_patch(rect)
            self.patches[name] = rect

            txt = self.ax.text(0, 0, "", ha='center', va='center', color='white')
            self.texts[name] = txt

        # Zones collecte
        for name, zone in self.zone_collecte.zones.items():
            rect = patches.Rectangle(
                (zone.xmin, zone.ymin),
                zone.xmax - zone.xmin,
                zone.ymax - zone.ymin,
                linewidth=2,
                edgecolor='green',
                facecolor='cyan',
                alpha=0.3
            )
            self.ax.add_patch(rect)
            self.patches[name] = rect

    def get_color(self, count):
        max_val = 4
        ratio = min(count / max_val, 1.0)
        return (ratio, 0, 1 - ratio)  # bleu -> rouge

    def update_plot(self):
        for coll in self.ax.collections:
            coll.remove()

        results = self.zone_rangement.analyze(self.arucos)

        for name, res in results.items():
            zone = self.zone_rangement.zones[name]
            rect = self.patches[name]

            rect.set_facecolor(self.get_color(res["count"]))

            # update texte
            self.texts[name].set_position((
                (zone.xmin + zone.xmax) / 2,
                (zone.ymin + zone.ymax) / 2
            ))
            self.texts[name].set_text(str(res["count"]))

        # Affichage ArUco
        if self.arucos:
            xs = [a.x for a in self.arucos]
            ys = [a.y for a in self.arucos]
            self.ax.scatter(xs, ys, s=10)

        plt.draw()
        plt.pause(0.01)

    # ----------- LOOP -----------

    def run(self):
        while True:
            if not self.arucos:
                time.sleep(0.05)
                continue

            self.update_plot()
            time.sleep(0.1)  # ~10 Hz


# ---------------- MAIN ----------------

if __name__ == "__main__":
    with World() as w:
        w.run()