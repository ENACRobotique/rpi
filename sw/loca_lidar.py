#!/usr/bin/env python3
import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Subscriber as ProtoSubscriber
from ecal.msg.proto.core import Publisher as ProtoPublisher
from ecal.msg.common.core import ReceiveCallbackData
import numpy as np
from collections import namedtuple
from scipy.optimize import least_squares, OptimizeResult
import time, sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '../'))
from generated import robot_state_pb2 as robot_pb
from generated import common_pb2 as common_pb
from generated import lidar_data_pb2 as lidar_pb
from common import Pos
import itertools


class Amalgame:
    def __init__(self, pos: Pos, size) -> None:
        self.pos = pos
        self.size = size


BEACONS_BLUE = {
    0: Pos(-94, 1000, 0),
    1: Pos(3094, 50, 0),
    2: Pos(3094, 1950, 0),
    3: Pos(1725, 2100, 0)
}
BEACONS_YELLOW = {
    0: Pos(3094, 1000, 0),
    1: Pos(-94, 50, 0),
    2: Pos(-94, 1950, 0),
    3: Pos(1275, 2100, 0)
}


MAX_COST = (150**2) * 6
TOLERANCE = 500 #mm
BEACON_MAX_SIZE = 150 # mm 
BEACON_MIN_SIZE = 50 # mm 



class LidarLoca:
    def __init__(self):
        if not ecal_core.is_initialized():
            ecal_core.initialize("loca_lidar")
        
        # last odom_pos received
        self.odom_pos = Pos(0, 0, 0)
        # estimated pos using lidar and odometry
        self.estimated_pos = Pos(0, 0, 0)

        # Subscribers
        self.sub_lidar = ProtoSubscriber(lidar_pb.Amalgames, "amalgames")
        self.sub_lidar.set_receive_callback(self.amalgames_cb)

        self.sub_odom = ProtoSubscriber(common_pb.Position, "odom_pos")
        self.sub_odom.set_receive_callback(self.odom_pos_cb)

        self.sub_reset_pos = ProtoSubscriber(common_pb.Position, "reset")
        self.sub_reset_pos.set_receive_callback(self.reset_pos_cb)

        self.sub_color = ProtoSubscriber(robot_pb.Side, "color")
        self.sub_color.set_receive_callback(self.color_cb)
        # Publishers
        self.pub_lidar = ProtoPublisher(common_pb.Position, "lidar_pos")
        self.pub_balises_odom = ProtoPublisher(lidar_pb.Balises, "balises_odom")
        self.pub_closest_to_odom_beacons = ProtoPublisher(lidar_pb.Balises, "balises_near_odom")
        self.BEACONS = BEACONS_BLUE
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        self.sub_lidar.remove_receive_callback()
        self.sub_odom.remove_receive_callback()
        self.sub_reset_pos.remove_receive_callback()
        self.sub_color.remove_receive_callback()

    def color_cb(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[robot_pb.Side]):
        msg = data.message
        self.color = msg.color
        if self.color == robot_pb.Side.Color.BLUE:
            self.BEACONS = BEACONS_BLUE
        else:
            self.BEACONS = BEACONS_YELLOW

    def recalage(self, pos_recalage: Pos):
        self.odom_pos = pos_recalage
        self.estimated_pos = pos_recalage

    def reset_pos_cb(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[common_pb.Position]):
        self.recalage(Pos.from_proto(data.message))

    def odom_pos_cb(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[common_pb.Position]):
        """ 
        Integrate odometry to estimated robot pos and send beacons expected positions.
        """
        new_odom_pos = Pos.from_proto(data.message)
        # get the odometry move since last odom_pos
        dpos = new_odom_pos - self.odom_pos
        self.odom_pos = new_odom_pos

        # add the odometry move to the last estimated position
        #self.estimated_pos += dpos
        self.pub_lidar.send(self.estimated_pos.to_proto())

        # send the positions in robot frame where we expect the beacons to be
        beacon_odom = {beacon_id: bpos.to_frame(self.estimated_pos) for beacon_id, bpos in self.BEACONS.items()}
        self.send_beacons_pos(beacon_odom, self.pub_balises_odom)
    
    def amalgames_cb(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[lidar_pb.Amalgames]):
        def estimate_pos(estimatedBeacons):
            assert(len(estimatedBeacons) >=2)
            result = self.estimate(estimatedBeacons)
            # TODO test result.success, and tune ftol (in least_squares args)?
            estimated_pos = Pos.from_np(result.x)
            return estimated_pos, result.cost, estimatedBeacons

        msg = data.message
        estimatedBeacons_comb = self.estimate_beacons_pos(msg)
        filtered_combs = filter(lambda x: len(x) >= 2, estimatedBeacons_comb)  # filter out combinations with less than 2 beacons
        estimated_poses = map(estimate_pos, filtered_combs)
        sorted_estimated_poses = list(sorted(estimated_poses, key=lambda x: x[1]))  # sort by cost
        #print([cost for estimated_pos, cost, estimatedBeacons in sorted_estimated_poses])
        if len(sorted_estimated_poses) > 0:
            estimated_pos, cost, estimatedBeacons = sorted_estimated_poses[0]
            if cost < MAX_COST:
                self.estimated_pos = estimated_pos
                self.pub_lidar.send(self.estimated_pos.to_proto())
                estimated_beacons_pos = {beacon_id: am.pos for beacon_id, am in estimatedBeacons.items()}
                self.send_beacons_pos(estimated_beacons_pos, self.pub_closest_to_odom_beacons)



    
    @staticmethod
    def all_combinations(d):
        # Ne garder que les clés avec des listes non vides
        filtered_items = [(k, v) for k, v in d.items() if v]
        if not filtered_items:
            return [{}]  # aucune combinaison possible, mais on retourne un dict vide
        keys, values_lists = zip(*filtered_items)
        values_product = itertools.product(*values_lists)
        return [dict(zip(keys, combination)) for combination in values_product]

    @staticmethod
    def closest_amalgames(beacon_pos_r: Pos, amalgames: list[Amalgame]) -> list[Amalgame]:
        """Returns amalgame and dist of the closest amalgame in the list"""
        dists = [beacon_pos_r.distance(amalgame.pos) for amalgame in amalgames]
        # filter out dists > TOLERANCE
        dist_filtered = filter(lambda x: x[1] <= TOLERANCE, zip(amalgames, dists))
        da_sorted = sorted(dist_filtered, key=lambda x: x[1])
        return [amalgame for amalgame, _ in da_sorted]

    def estimate_beacons_pos(self, msg) -> list[dict[int, Amalgame]]:
        # On filtre les amalgames qui ne sont pas des balises
        amalgames = [Amalgame(Pos(x, y, 0), size) for (x, y, size) in zip(msg.x, msg.y, msg.size) if  size < BEACON_MAX_SIZE]
        
        # {beacon_id: Amalgame}
        estimatedBeacons: dict[int, list[Amalgame]] = {}
        
        for beacon_id, beacon_pos in self.BEACONS.items():
            # estimated position of the beacon in robot frame (using best estimated robot pos)
            beacon_pos_r = beacon_pos.to_frame(self.estimated_pos)
            # best candidate (Amalgame), and its distance to the theoretical position
            candidates = self.closest_amalgames(beacon_pos_r, amalgames)

            estimatedBeacons[beacon_id] = candidates
        
        combinations = self.all_combinations(estimatedBeacons)
        # for comb in combinations:
        #     estimated_beacons_pos = {beacon_id: am.pos for beacon_id, am in comb.items()}
        #     self.send_beacons_pos(estimated_beacons_pos, self.pub_closest_to_odom_beacons)
        return combinations

    
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


if __name__ == "__main__":
    with LidarLoca() as d:
        while ecal_core.ok():
            time.sleep(0.01)

