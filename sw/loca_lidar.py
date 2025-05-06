#!/usr/bin/env python3
import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher
import numpy as np
from collections import namedtuple
from scipy.optimize import least_squares, OptimizeResult
import time, sys
sys.path.append("../")
from generated import robot_state_pb2 as robot_pb
from generated import lidar_data_pb2 as lidar_pb
from common import Pos


class Amalgame:
    def __init__(self, pos: Pos, size) -> None:
        self.pos = pos
        self.size = size


BEACONS_BLUE = {
    0: Pos(-94, 1000, 0),
    1: Pos(3094, 50, 0),
    2: Pos(3094, 1950, 0),
    3: Pos(1275, 2100, 0)
}
BEACONS_YELLOW = {
    0: Pos(3094, 1000, 0),
    1: Pos(-94, 50, 0),
    2: Pos(-94, 1950, 0),
    3: Pos(1725, 2100, 0)
}


MAX_COST = (150**2) * 6
TOLERANCE = 500 #mm
BEACON_MAX_SIZE = 150 # mm 
BEACON_MIN_SIZE = 50 # mm 



class LidarLoca:
    def __init__(self):
        if not ecal_core.is_initialized():
            ecal_core.initialize(sys.argv, "loca_lidar")
        
        # last odom_pos received
        self.odom_pos = Pos(0, 0, 0)
        # estimated pos using lidar and odometry
        self.estimated_pos = Pos(0, 0, 0)

        # Subscribers
        self.sub_lidar = ProtoSubscriber("amalgames", lidar_pb.Amalgames)
        self.sub_lidar.set_callback(self.amalgames_cb)
        
        self.sub_odom = ProtoSubscriber("odom_pos", robot_pb.Position)
        self.sub_odom.set_callback(self.odom_pos_cb)

        self.sub_reset_pos = ProtoSubscriber("reset", robot_pb.Position)
        self.sub_reset_pos.set_callback(self.reset_pos_cb)

        self.sub_color = ProtoSubscriber("color",robot_pb.Side)
        self.sub_color.set_callback(self.color_cb)
        # Publishers
        self.pub_lidar = ProtoPublisher("lidar_pos", robot_pb.Position)
        self.pub_balises_odom = ProtoPublisher("balises_odom", lidar_pb.Balises)
        self.pub_closest_to_odom_beacons = ProtoPublisher("balises_near_odom", lidar_pb.Balises)
        self.BEACONS = BEACONS_BLUE

    def color_cb(self, topic_name, msg, time):
        self.color = msg.color
        if self.color == robot_pb.Side.Color.BLUE:
            self.BEACONS = BEACONS_BLUE
        else:
            self.BEACONS = BEACONS_YELLOW

    def recalage(self, pos_recalage: Pos):
        self.odom_pos = pos_recalage
        self.estimated_pos = pos_recalage

    def reset_pos_cb(self, topic_name, msg, time):
        self.recalage(Pos.from_proto(msg))

    def odom_pos_cb(self, topic_name, msg, time):
        """ 
        Integrate odometry to estimated robot pos and send beacons expected positions.
        """
        new_odom_pos = Pos.from_proto(msg)
        # get the odometry move since last odom_pos
        dpos = new_odom_pos - self.odom_pos
        self.odom_pos = new_odom_pos

        # add the odometry move to the last estimated position
        self.estimated_pos += dpos
        self.pub_lidar.send(self.estimated_pos.to_proto())

        # send the positions in robot frame where we expect the beacons to be
        beacon_odom = {beacon_id: bpos.to_frame(self.estimated_pos) for beacon_id, bpos in self.BEACONS.items()}
        self.send_beacons_pos(beacon_odom, self.pub_balises_odom)
    
    def amalgames_cb(self, topic_name, msg, time):
        estimatedBeacons = self.estimate_beacons_pos(msg)

        if len(estimatedBeacons) >=2:
            result = self.estimate(estimatedBeacons)
            # TODO test result.success, and tune ftol (in least_squares args)?
            if result.cost <  MAX_COST and len(estimatedBeacons) >= 2:
                # only use the result if the cost is low enough and at least 2 beacons where associated.
                self.estimated_pos = Pos.from_np(result.x)
            else:
                print(f"cost too high: {int(result.cost)}/{MAX_COST}")
            self.pub_lidar.send(self.estimated_pos.to_proto())

    @staticmethod
    def closest_amalgame(beacon_pos_r: Pos, amalgames: list[Amalgame]) -> tuple[Amalgame, float]:
        """Returns amalgame and dist of the closest amalgame in the list"""
        dists = [beacon_pos_r.distance(amalgame.pos) for amalgame in amalgames]
        index = np.argmin(dists)
        return amalgames[index], dists[index]

    def estimate_beacons_pos(self, msg) -> dict[int, Amalgame]:
        # On filtre les amalgames qui ne sont pas des balises
        amalgames = [Amalgame(Pos(x, y, 0), size) for (x, y, size) in zip(msg.x, msg.y, msg.size) if  size < BEACON_MAX_SIZE]
        
        # {beacon_id: Amalgame}
        estimatedBeacons: dict[int, Amalgame] = {}
        
        for beacon_id, beacon_pos in self.BEACONS.items():
            # estimated position of the beacon in robot frame (using best estimated robot pos)
            beacon_pos_r = beacon_pos.to_frame(self.estimated_pos)
            # best candidate (Amalgame), and its distance to the theoretical position
            candidate, dist = self.closest_amalgame(beacon_pos_r, amalgames)

            if dist <= TOLERANCE:
                estimatedBeacons[beacon_id] = candidate

        if len(estimatedBeacons) > 0:
            # On ne publie que les balises trouvée
            estimated_beacons_pos = {beacon_id: am.pos for beacon_id, am in estimatedBeacons.items()}
            self.send_beacons_pos(estimated_beacons_pos, self.pub_closest_to_odom_beacons)
        
        return estimatedBeacons
    
    @staticmethod
    def jacobienne(x: np.ndarray, am_beacon_pairs: list[tuple[Pos, Pos]]):
        c = np.cos(x[2])
        s = np.sin(x[2])
        jac = np.zeros((2*len(am_beacon_pairs), 3))
        for i, (am_pos_r, _) in enumerate(am_beacon_pairs):
            jac[i,0] = 1
            jac[i,1] = 0
            jac[i,2] = -s*am_pos_r.x + c*am_pos_r.y
            jac[i+1,0] = 0
            jac[i+1,1] = 1
            jac[i+1,2] = -c*am_pos_r.x - s*am_pos_r.y
        return jac

    @staticmethod
    def objective_function(x: np.ndarray, am_beacon_pairs: list[tuple[Pos, Pos]]):
        # for each pair (amalgame, beacon), get the distance between the beacon position (in table frame)
        # and the amalgame position, originaly expressed in frame pos_t.
        # if pos_t is the current robot position, the distance should be small.
        pos_t = Pos.from_np(x)
        #residuals = [am_pos_r.from_frame(pos_t).distance(beacon_pos_t) for am_pos_r, beacon_pos_t in am_beacon_pairs]
        residuals = []
        for am_pos_r, beacon_pos_t in am_beacon_pairs:
            pos_error = am_pos_r.from_frame(pos_t) - beacon_pos_t
            residuals.append(pos_error.x)
            residuals.append(pos_error.y)
        #print (residuals)
        return residuals

    def estimate(self, detected_beacons_r: dict[int, Amalgame]) -> OptimizeResult:
        # list of tuples: (am_pos_r, beacon_pos_t)
        # associate amalgame pos in robot frame to theoretical beacon pos in table frame
        am_beacon_pairs: list[tuple[Pos, Pos]] = []
        for beacon_id, am in detected_beacons_r.items():
            b = self.BEACONS[beacon_id]
            am_beacon_pairs.append((am.pos, b))

        result = least_squares(
            fun = self.objective_function,
            x0 = self.estimated_pos.to_np(),          # start from latest estimated position
            # jac=self.jacobienne,
            args = (am_beacon_pairs,),
            method = 'lm'
        )

        return result

    def send_beacons_pos(self, b_pos_r: dict[int, Pos], pub: ProtoPublisher):
        """
        Send beacons positions over publisher @pub to be displayed on radarQt.
        Beacon positions must be expressed in robot frame
        """
        xs = []
        ys = []
        ids = []
        for beacon_id, beacon_pos_r in b_pos_r.items():
            ids.append(beacon_id)
            xs.append(beacon_pos_r.x)
            ys.append(beacon_pos_r.y)
        msg = lidar_pb.Balises(index=ids, x=xs, y=ys)
        pub.send(msg)


    # def dist_2_pts(self,pt1, pt2):
    #     return math.sqrt((pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) ** 2)
    
    # def get3rd(self, knownIdA, knownIdB, unkwownIdC):
    #     radiusA = self.dist_2_pts(RealBeacons[knownIdA],RealBeacons[unkwownIdC])
    #     radiusB = self.dist_2_pts(RealBeacons[knownIdB],RealBeacons[unkwownIdC])
    #     pA = self.estimatedBeacons[knownIdA]
    #     pB = self.estimatedBeacons[knownIdB]
    #     xA, yA = pA
    #     xB, yB = pB
    #     print('x1,y1 ',xA,yA, ' x2,y2', xB,yB)

    #     d = self.dist_2_pts(pA, pB)

    #     if radiusA + radiusB < d or abs(radiusA - radiusB) > d:
    #         raise ValueError("No intersection points found. The circles do not intersect.")

    #     a = (radiusA**2 - radiusB**2 + d**2) / (2 * d)
    #     h = math.sqrt(radiusA**2 - a**2)

    #     xC = xA + a * (xB - xA) / d
    #     yC = yA + a * (yB - yA) / d

    #     intersection1 = (xC + h * (yB - yA) / d, yC - h * (xB - xA) / d)
    #     intersection2 = (xC - h * (yB - yA) / d, yC + h * (xB - xA) / d)


    #     return intersection1, intersection2
    
    # def estimate_third_beacon(self, idxs, points):
    #     distance_01 = self.dist_2_pts(self.initialGuess[0], self.initialGuess[1])
    #     distance_02 = self.dist_2_pts(self.initialGuess[0], self.initialGuess[2])
    #     distance_12 = self.dist_2_pts(self.initialGuess[1], self.initialGuess[2])
    #     # print(idxs,distance_01,distance_02,distance_12)

    #     if 1 in idxs and 2 in idxs:  
    #         return self.estimate_third_from_two(points[0], points[1], distance_01, distance_02)

    #     elif 1 in idxs and 3 in idxs:  
    #         return self.estimate_third_from_two(points[0], points[1], distance_01, distance_12)

    #     elif 2 in idxs and 3 in idxs: 
    #         return self.estimate_third_from_two(points[0], points[1], distance_02, distance_12)
    
    # def estimate_third_from_two(self, p1, p2, dist1, dist2):

    #     x1, y1 = p1
    #     x2, y2 = p2
    #     print('x1,y1 ',x1,y1, ' x2,y2', x2,y2)

    #     d = self.dist_2_pts(p1, p2)

    #     if dist1 + dist2 < d or abs(dist1 - dist2) > d:
    #         raise ValueError("No intersection points found. The circles do not intersect.")

    #     a = (dist1**2 - dist2**2 + d**2) / (2 * d)
    #     h = math.sqrt(dist1**2 - a**2)

    #     x3 = x1 + a * (x2 - x1) / d
    #     y3 = y1 + a * (y2 - y1) / d

    #     intersection1 = (x3 + h * (y2 - y1) / d, y3 - h * (x2 - x1) / d)
    #     intersection2 = (x3 - h * (y2 - y1) / d, y3 + h * (x2 - x1) / d)

    #     return intersection2
    
    # def assign_beacons(self, amalgames):
    #     """Dans le cas où aucune balise n’est clairement identifiée, cette méthode teste toutes les combinaisons de 3 amalgames pour vérifier si leurs distances correspondent aux distances entre les balises théoriques. Si un triplet de points correspond (avec une certaine tolérance), il est considéré comme valide.
    #     C'est la méthode Jonathan si vraiment on s'en sort plus"""
    #     d_ab = self.dist_2_pts(self.initialGuess[0], self.initialGuess[1])  # Beacon A - Beacon B
    #     d_ac = self.dist_2_pts(self.initialGuess[0], self.initialGuess[2])  # Beacon A - Beacon C
    #     d_bc = self.dist_2_pts(self.initialGuess[1], self.initialGuess[2])  # Beacon B - Beacon C

    #     combinations_of_points = list(itertools.combinations(amalgames, 3))

    #     for comb in combinations_of_points:
    #         d_fg = self.dist_2_pts(comb[0], comb[1])  # Distance between F and G
    #         d_fh = self.dist_2_pts(comb[0], comb[2])  # Distance between F and H
    #         d_gh = self.dist_2_pts(comb[1], comb[2])  # Distance between G and H

    #         matching = self.check_distances([ d_ab, d_ac, d_bc],d_fg, d_fh, d_gh)

    #         if matching:
    #             print(f"Valid combination found: {comb}")
    #             assignment = self.get_assignment(comb, d_fg, d_fh, d_gh, d_ab, d_ac, d_bc)
    #             print(f"Assigned points: {assignment}")
    #             return  assignment
 
    # def get_assignment(self, comb, d_fg, d_fh, d_gh, d_ab, d_ac, d_bc):

    #     assignment = {}
    #     distances = [(d_fg, 'AB'), (d_fh, 'AC'), (d_gh, 'BC')]

    #     distances.sort()  

    #     for i, (dist, beacon) in enumerate(distances):
    #         if beacon == 'AB':
    #             assignment['1'] = self.initialGuess[0]  # Beacon A
    #             assignment['2'] = self.initialGuess[1]  # Beacon B
    #         elif beacon == 'AC':
    #             assignment['1'] = self.initialGuess[0]  # Beacon A
    #             assignment['3'] = self.initialGuess[2]  # Beacon C
    #         elif beacon == 'BC':
    #             assignment['2'] = self.initialGuess[1]  # Beacon B
    #             assignment['3'] = self.initialGuess[2]  # Beacon C

    #     return assignment

    # def check_distances(self, dist_abc, dist_12, dist_13, dist_23):

    #     threshold = 100  # Tolerance value (can be adjusted)
    #     dist_12_close = abs(dist_abc[0] - dist_12) < threshold
    #     dist_13_close = abs(dist_abc[1] - dist_13) < threshold
    #     dist_23_close = abs(dist_abc[2] - dist_23) < threshold

    #     return dist_12_close and dist_13_close and dist_23_close


if __name__ == "__main__":
    d = LidarLoca()

    while ecal_core.ok():
        time.sleep(0.01)
