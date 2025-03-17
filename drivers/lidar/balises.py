from itertools import combinations
import ecal.core.core as ecal_core
import math
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher
import numpy as np
import time, os, sys
from numpy import typing as npt
import generated.robot_state_pb2 as robot_pb
import generated.lidar_data_pb2 as lidar_pb
import itertools

# il faut Créer lidar_pb.Coord_beacons 

class Balises:
    def __init__(self):
        self.odom_pos = (0, 0)
        ecal_core.initialize(sys.argv, "lid_amalgameur")

        # Subscribers
        self.sub_lidar = ProtoSubscriber("amalgames", lidar_pb.Amalgames)
        self.sub_odom = ProtoSubscriber("odom_pos", robot_pb.Position)

        # Publishers
        self.pub_balises_odom = ProtoPublisher("balises_odom", lidar_pb.Balises)
        self.pub_closest_to_odom_beacons = ProtoPublisher("balises_near_odom", lidar_pb.BalisesProches)

        # Callback setup
        self.sub_odom.set_callback(self.get_balises_odom)
        self.sub_lidar.set_callback(self.beacon_detection)  # Lidar callback

        # Initial beacon positions (theoretical positions)
        self.coords_init = [(-94, 1000), (3094, 50), (3094, 1950)]
        self.odomx=[]
        self.odomy=[]
    
    def get_balises_odom(self, topic_name, msg, time):

        xs = []
        ys = []
        for balise in self.coords_init:
            dx = balise[0] - msg.x
            dy = balise[1] - msg.y
            x_b_rr = dx * np.cos(msg.theta) + dy * np.sin(msg.theta)
            y_b_rr = dy * np.cos(msg.theta) - dx * np.sin(msg.theta)
            xs.append(x_b_rr)
            ys.append(y_b_rr)
            

        self.odomx=xs
        self.odomy=ys
        msg = lidar_pb.Balises(index=[1, 2, 3], x=xs, y=ys)
        self.pub_balises_odom.send(msg)

    def dist_2_pts(self,pt1, pt2):
        return math.sqrt((pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) ** 2)
    
    def estimate_third_beacon(self, idxs, points):

        distance_12 = self.dist_2_pts(self.coords_init[0], self.coords_init[1])
        distance_13 = self.dist_2_pts(self.coords_init[0], self.coords_init[2])
        distance_23 = self.dist_2_pts(self.coords_init[1], self.coords_init[2])

        if 1 in idxs and 2 in idxs:  
            return self.estimate_third_from_two(points[0], points[1], distance_12, distance_13)

        elif 1 in idxs and 3 in idxs:  
            return self.estimate_third_from_two(points[0], points[1], distance_12, distance_23)

        elif 2 in idxs and 3 in idxs: 
            return self.estimate_third_from_two(points[0], points[1], distance_13, distance_23)
    def estimate_third_from_two(self, p1, p2, dist1, dist2):

        x1, y1 = p1
        x2, y2 = p2

        d = self.dist_2_pts(p1, p2)

        if dist1 + dist2 < d or abs(dist1 - dist2) > d:
            raise ValueError("No intersection points found. The circles do not intersect.")

        a = (dist1**2 - dist2**2 + d**2) / (2 * d)
        h = math.sqrt(dist1**2 - a**2)

        x3 = x1 + a * (x2 - x1) / d
        y3 = y1 + a * (y2 - y1) / d

        intersection1 = (x3 + h * (y2 - y1) / d, y3 - h * (x2 - x1) / d)
        intersection2 = (x3 - h * (y2 - y1) / d, y3 + h * (x2 - x1) / d)

        return intersection1  
    
    def assign_beacons(self, amalgames):
        d_ab = self.dist_2_pts(self.coords_init[0], self.coords_init[1])  # Beacon A - Beacon B
        d_ac = self.dist_2_pts(self.coords_init[0], self.coords_init[2])  # Beacon A - Beacon C
        d_bc = self.dist_2_pts(self.coords_init[1], self.coords_init[2])  # Beacon B - Beacon C

        combinations_of_points = list(itertools.combinations(amalgames, 3))

        for comb in combinations_of_points:
            d_fg = self.dist_2_pts(comb[0], comb[1])  # Distance between F and G
            d_fh = self.dist_2_pts(comb[0], comb[2])  # Distance between F and H
            d_gh = self.dist_2_pts(comb[1], comb[2])  # Distance between G and H

            matching = self.check_distances([ d_ab, d_ac, d_bc],d_fg, d_fh, d_gh)

            if matching:
                print(f"Valid combination found: {comb}")
                assignment = self.get_assignment(comb, d_fg, d_fh, d_gh, d_ab, d_ac, d_bc)
                print(f"Assigned points: {assignment}")
                return  assignment
            
 

    def get_assignment(self, comb, d_fg, d_fh, d_gh, d_ab, d_ac, d_bc):

        assignment = {}
        distances = [(d_fg, 'AB'), (d_fh, 'AC'), (d_gh, 'BC')]

        distances.sort()  

        for i, (dist, beacon) in enumerate(distances):
            if beacon == 'AB':
                assignment['1'] = self.coords_init[0]  # Beacon A
                assignment['2'] = self.coords_init[1]  # Beacon B
            elif beacon == 'AC':
                assignment['1'] = self.coords_init[0]  # Beacon A
                assignment['3'] = self.coords_init[2]  # Beacon C
            elif beacon == 'BC':
                assignment['2'] = self.coords_init[1]  # Beacon B
                assignment['3'] = self.coords_init[2]  # Beacon C

        return assignment


    def check_distances(self, dist_abc, dist_12, dist_13, dist_23):

        threshold = 100  # Tolerance value (can be adjusted)
        dist_12_close = abs(dist_abc[0] - dist_12) < threshold
        dist_13_close = abs(dist_abc[1] - dist_13) < threshold
        dist_23_close = abs(dist_abc[2] - dist_23) < threshold

        return dist_12_close and dist_13_close and dist_23_close


    def beacon_detection(self, topic_name, msg, time):
        amalgames = [(msg.x[i], msg.y[i]) for i in range(len(msg.x))]  

        closest_points = {1: [], 2: [], 3: []}
        valid_points = [] 
    
        for idx, (ox, oy) in enumerate(zip(self.odomx, self.odomy), start=1):
            distances = [self.dist_2_pts((ox, oy), amalgame) for amalgame in amalgames]

            closest_idx = np.argmin(distances)  
            closest_point = amalgames[closest_idx]  
            closest_points[idx] = closest_point  

            if distances[closest_idx] < 400:
                valid_points.append((idx, closest_point))  

        if len(valid_points) == 3:
        # Case where all three beacons are detected, publish directly
            print("All three beacons detected within range.")
            msg = lidar_pb.BalisesProches(
                index=[1, 2, 3],
                x=[p[1][0] for p in valid_points],
                y=[p[1][1] for p in valid_points]
        )
            self.pub_closest_to_odom_beacons.send(msg)
        elif len(valid_points) == 2:
            print("Two beacons detected. Estimating the position of the third beacon.")
            idxs, points = zip(*valid_points)
            estimated_third_point = self.estimate_third_beacon(idxs, points)
            print(estimated_third_point)
            msg = lidar_pb.BalisesProches(
                index=[1, 2, 3],
                x=[points[0][0], points[1][0], estimated_third_point[0]],
                y=[points[0][1], points[1][1], estimated_third_point[1]]
            )
            self.pub_closest_to_odom_beacons.send(msg)
        else:
            try:
                assignment = self.assign_beacons(amalgames)

                if assignment is None:
                    raise ValueError("No valid beacon assignment found.")

                msg = lidar_pb.BalisesProches(
                    index=[1, 2, 3],
                    x=[assignment['1'][0], assignment['2'][0], assignment['3'][0]],
                    y=[assignment['1'][1], assignment['2'][1], assignment['3'][1]]
                )
            except Exception as e:
                print(f"Error assigning beacons: {e}. Falling back to theoretical positions.")

                msg = lidar_pb.BalisesProches(
                    index=[1, 2, 3],
                    x=self.odomx,
                    y=self.odomy
                )

        self.pub_closest_to_odom_beacons.send(msg)
if __name__ == "__main__":
    d = Balises()

    while ecal_core.ok():
        time.sleep(1)
