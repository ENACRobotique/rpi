import logging
import sys, os
import time
from typing import Tuple, Union, List
import numpy as np
from math import radians, pi, degrees
import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher, StringPublisher

from loca_lidar.ObstacleCalc import ObstacleCalc
import loca_lidar.CloudPoints as cp
import loca_lidar.PatternFinder as pf
from loca_lidar.PointsDataStruct import PolarPts
import loca_lidar.config as config

import generated.lidar_data_pb2 as lidar_pb
import generated.robot_state_pb2 as robot_pb

ecal_core.initialize(sys.argv, "loca_lidar_ecal_interface")

sub_odom_pos = ProtoSubscriber("odom_pos", robot_pb.Position)
sub_lidar = ProtoSubscriber("lidar_data", lidar_pb.Lidar)
sub_side = ProtoSubscriber("color", robot_pb.Side)
#sub_tirette = ProtoSubscriber("ihm", robot_pb.IHM)

pub_filtered_pts = ProtoPublisher("lidar_filtered", lidar_pb.Lidar)
pub_amalgames = ProtoPublisher("amalgames_loca", lidar_pb.Lidar)
pub_beacons = StringPublisher("beacons") # Only up to 5 points are sent, the index correspond to the fixed_point
pub_lidar_pos = ProtoPublisher("lidar_pos", robot_pb.Position)
pub_lidar_pos_deg = ProtoPublisher("deg_lidar_pos", robot_pb.Position)
#pub_obstacles = ProtoPublisher("obstacles_wrt_table", lidar_pb.Amalgames)

last_known_lidar = (0, 0, 0) #x, y, theta (meters, degrees)
robot_pose = (0.0, 0.0, 0.0) #x, y, theta (meters, degrees)
OBSTACLE_CALC = ObstacleCalc(
    config.lidar_x_offset, config.lidar_y_offset, config.lidar_theta_offset)

SIDE_SET = False
BLUE_BEACONS = pf.GroupAmalgame(tuple((x / 1000, y / 1000) for x,y in config.blue_points_in_mm), True)
YELLOW_BEACONS = pf.GroupAmalgame(tuple((x / 1000, y / 1000) for x,y in config.yellow_points_in_mm), True)
beacons_to_use = BLUE_BEACONS

BLUE_FINDER = pf.LinkFinder(BLUE_BEACONS, 0.06, 1.5)
YELLOW_FINDER = pf.LinkFinder(YELLOW_BEACONS, 0.06, 1.5)
finder_to_use = BLUE_FINDER

def send_obstacles_wrt_table(obstacles: List[List[Union[float, float]]]):
    msg = lidar_pb.Amalgames()
    x, y = [], []
    for obstacle in obstacles:
        x.append(obstacle[0])
        y.append(obstacle[1])
    msg.x.extend(x)
    msg.y.extend(y)
    #pub_obstacles.send(msg, ecal_core.getmicroseconds()[1])
    
def send_lidar_scan(pub, distances, angles):
    lidar_msg = lidar_pb.Lidar()
    lidar_msg.angle_increment = float(-1.0) # prevent empty message when sending empty lidar scan (eg no obstacle found)
    lidar_msg.angles.extend(angles)
    lidar_msg.distances.extend(distances)
    pub.send(lidar_msg, ecal_core.getmicroseconds()[1])

def send_lidar_pos(x, y, theta):
    pos_msg = robot_pb.Position()
    pos_msg.x = float(x)*1000
    pos_msg.y = float(y)*1000
    pos_msg.theta = radians(-theta) + config.loca_theta_offset
    #print(pos_msg)
    pub_lidar_pos.send(pos_msg, ecal_core.getmicroseconds()[1])
    # human readable version : 
    pub_lidar_pos_deg.send(
        robot_pb.Position(x=float(x), y=float(y), theta=float(theta+degrees(config.loca_theta_offset))), 
        ecal_core.getmicroseconds()[1]) 

def on_side_set(topic_name, side_msg, time):
#def on_side_set(topic_name, side_msg, time):
    global SIDE_SET, beacons_to_use, finder_to_use
    if side_msg.color == robot_pb.Side.Color.BLUE and beacons_to_use != BLUE_BEACONS:
        beacons_to_use = BLUE_BEACONS
        finder_to_use = BLUE_FINDER
    elif side_msg.color == robot_pb.Side.Color.YELLOW and beacons_to_use != YELLOW_BEACONS:
        beacons_to_use = YELLOW_BEACONS
        finder_to_use = YELLOW_FINDER
    else:
        pass
        # raise ValueError("ecal_loca_lidar - on_side_set - Invalid side value")
    SIDE_SET = True


def on_robot_pos(topic_name, pos_msg, time):
    global robot_pose
    robot_pose = (pos_msg.x, pos_msg.y, pos_msg.theta)


def on_lidar_scan(topic_name, proto_msg, time):
    global last_known_dest, robot_pose, last_known_lidar

    if robot_pose == (0.0, 0.0, 0.0):
        pass #logging.warning("Robot pose not received yet - invalid obstacle avoidance")
    
    t = ecal_core.getmicroseconds()[1]
    # Filter 
    quality_filter = np.array(proto_msg.quality) > config.QUALITY_REJECTION_VAL

    distances = np.array(proto_msg.distances)[quality_filter]/1000 # in meters
    angles = np.array(proto_msg.angles)[quality_filter]
    lidar_scan =  np.rec.fromarrays([distances,angles], dtype=PolarPts)
    basic_filtered_scan = cp.basic_filter_pts(lidar_scan)

    amalgames = cp.amalgames_from_cloud(basic_filtered_scan)
    amalgames = cp.filter_amalgame_size(amalgames)
    send_lidar_scan(pub_amalgames, amalgames['center_polar']['distance'], amalgames['center_polar']['angle']) # Display filtered data for debugging purposes

    #position calculation
    lidar2table = {}
    #TODO : remove empty amalgames['centerpolar]
    lidar_pose = calculate_lidar_pose(amalgames['center_polar'], robot_pose, lidar2table)
    #print(lidar_pose)
    if lidar_pose != (0, 0, 0):
        pub_beacons.send(str(lidar2table))
        send_lidar_pos(*lidar_pose)

    t2 = ecal_core.getmicroseconds()[1] - t
    # print("processing duration total in ms : ",t2)

def calculate_lidar_pose(amalgame_scan, robot_pose = (0.0, 0.0, 0.0), corr_out = {}) :# -> Tuple[float, float, float]:
    """_summary_

    Args:
        lidar_scan (_type_): amalgame from lidar scan ((distance, angle), ...)
        corr_out (dict, optional): Correspondance lidar2table dict. Modify this dictionary. Can be used for visualization purpose. Defaults to {}.

    Returns:
        Tuple[float, float, float]: _description_
    """
    #Find correspondances between lidar and table
    amalgame_1 = pf.GroupAmalgame(amalgame_scan, False)
    try:
        lidar2table_set = finder_to_use.find_pattern(amalgame_1)
    except pf.NotEnoughAmalgames:
        return 0, 0, 0

    if lidar2table_set == None:
        logging.info("No correspondance found between lidar and table")
        return (0, 0, 0)

    poses = []
    best_pose = [0, 0, 0]

    # ###INITIAL FILTERING : remove correspondances that don't have the "mat central" (if it's present at least once) ###
    # remove the correspondances that don't have the "mat central" 
    lidar2table_with_mat = [corr for corr in lidar2table_set 
                                if config.unsymetrical_point_index in corr.values()]
    # consider correspondances with mat central if it's present in at least one correspondance
    lidar2table_set = lidar2table_with_mat if len(lidar2table_with_mat) > 0 else lidar2table_set
    
    for i, corr in enumerate(lidar2table_set):
        lidar_pos = pf.lidar_pos_wrt_table(
            corr, amalgame_1.points, beacons_to_use.points)
        lidar_angle = pf.lidar_angle_wrt_table(
            lidar_pos, corr, amalgame_1.points, beacons_to_use.points)
        if len(lidar2table_set) == 1: # trivial case
            best_pose = [lidar_pos[0], lidar_pos[1], lidar_angle]
            #corr_out |= corr #fusion the two dicts, to make sure that outside the function the dict is not empty
            corr_out.update(corr)
            break
        poses.append((lidar_pos[0], lidar_pos[1], lidar_angle))

    if poses != []: # if multiple poses found, select one of them      
        # print("lidar2table_set", lidar2table_set)
        #eliminate poses outside the tables
        poses = [pose for pose in poses if 
                 pose[0] > config.table_x_min and pose[0] < config.table_x_max 
                 and pose[1] > config.table_y_min and pose[1] < config.table_y_max]
        
        def in_table(pose):
            return pose[0] > config.table_x_min and \
                pose[0] < config.table_x_max and \
                pose[1] > config.table_y_min and \
                pose[1] < config.table_y_max
            
        poses_in_table = list(filter(in_table, poses))
        # take the pose closest to odometry pose
        if len(poses_in_table) > 0:
            best_pose = min(poses_in_table, key=lambda x: cp.get_squared_dist_cartesian(robot_pose[:2], x))
            closest_pt_index = poses_in_table.index(best_pose)
            corr_out.update(list(lidar2table_set)[closest_pt_index])
            #print("multiple poses found : best one is : ", best_pose)
    #best_pose[0] = best_pose[0]
    #best_pose[1] = best_pose[1]
    return best_pose
   

if __name__ == "__main__":
    sub_lidar.set_callback(on_lidar_scan)
    sub_odom_pos.set_callback(on_robot_pos)
    sub_side.set_callback(on_side_set)
    #sub_tirette.set_callback(on_tirette_set)

    while ecal_core.ok():
        time.sleep(0.01)

    ecal_core.finalize()
