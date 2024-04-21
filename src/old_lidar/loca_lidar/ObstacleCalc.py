# Manage Obstacles position wrt Table - can be used for planning by navigation (check if a place is free)

from dataclasses import dataclass
from functools import lru_cache
import numpy as np
from typing import Tuple, List, Union
try:
    from loca_lidar.config import table_x_min, table_x_max, table_y_min, table_y_max, lidar_theta_offset
except ModuleNotFoundError:
    from config import table_x_min, table_x_max, table_y_min, table_y_max, lidar_theta_offset


@dataclass
class ObstacleCalc():
    # last_lidar_dist: tuple
    x_offset: float
    y_offset: float
    theta_offset: float # offset of the lidar relative to the robot center (meters, degrees)

    def __post_init__(self):
        if self.x_offset is None: self.x_offset = 0
        if self.y_offset is None: self.y_offset = 0
        if self.theta_offset is None: self.theta_offset = 0
    
    def calc_obstacles_wrt_table(self, robot_wrt_table: tuple, lidar_pts: Tuple[Tuple[float, float], ...]) -> List[List[Union[float, float]]]: 
        # Calculate and returns the cartesian coordinates of the obstacles 
        # given the lidar polar points and the lidar position on the table
        # !! Filter the obstacles outside the table
        # !! theta angle is parallel to the y axis currently
        obstacles = []
        print("theta_off", self.theta_offset)
        rot_angle = (robot_wrt_table[2] + self.theta_offset) # radians
        for pt in lidar_pts: # May be possible to vectorize with numpy to avoid for loop
            # From Lidar Frame to table frame directly
            # pt_wrt_lidar = self.polar_lidar_to_cartesian(pt)
            pt_wrt_table = self.lidar_polar2table_cart(robot_wrt_table, pt, rot_angle)
            # print("lidar", pt)
            # print("table", pt_wrt_table)
            obstacles.append(pt_wrt_table)

        return obstacles

    @staticmethod
    def mask_filter_obs(obstacles: List[bool]):
        # keep obstacles inside the "table" area (taking into account edge offset)
        # [True, False] = [Obstacle in table, Obstacle outside table]
        filtered_obstacles = []
        for obs in obstacles:
            if not (obs[0] < table_x_min or obs[0] > table_x_max 
                    or obs[1] < table_y_min or obs[1] > table_y_max):
                filtered_obstacles.append(True)
            else:
                filtered_obstacles.append(False)
        return filtered_obstacles
    
    @staticmethod
    def polar_lidar_to_cartesian(polar_coord: Tuple[float, float]):
    #input : (r, theta) 
    # r = distance ; theta = angle in DEGREES
        r = polar_coord[0]
        theta = np.radians(polar_coord[1]) + np.pi/2 #adding offset
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        return np.array([x, y])
    
    @staticmethod
    def lidar_polar2table_cart(robot_wrt_table: tuple, lidar_pt: Tuple[float, float], rot_angle: float) -> List[float]:
        pt_wrt_lidar = ObstacleCalc.polar_lidar_to_cartesian(lidar_pt)
        rot_mat = np.array([[np.cos(rot_angle), -np.sin(rot_angle)], 
                            [np.sin(rot_angle), np.cos(rot_angle)]])
        #Formula below : Ax + d from https://youtu.be/TWTMoFvcBFc?t=116
        pt_wrt_table = np.dot(rot_mat, pt_wrt_lidar) + np.array([robot_wrt_table[0], robot_wrt_table[1]])
        return pt_wrt_table

    @staticmethod
    def is_valid_lidar2table_transform(robot_pose:tuple, beacon_pose:tuple, expected_beacon_pose:tuple):
        #robot_pose = (x,y, theta RADIANS) in table, beacon_pose = (r, theta radians) in lidar, expected_lidar_pose = (x,y) in table
        # Return true if the lidar2table transform looks good (within 3cm error)
        # Return false if it doesn't look good (but it might be localization problem !) and you need to tune config.lidar_theta_offset
        table_coord = ObstacleCalc.lidar_polar2table_cart(robot_pose, beacon_pose, robot_pose[2] + lidar_theta_offset)
        #if np.isclose
        print("expected", expected_beacon_pose)
        print("calculated", table_coord)
        return table_coord

if __name__ == "__main__":
    for t_int in range(0, 63):
        theta = t_int/10 #2.9 + t_int/100
        print(theta)
        a = ObstacleCalc(0.0, 0.0, theta)
        a.calc_obstacles_wrt_table((0.212, 0.235, -0.022), 
            (
            (0.76, 333), # beacon "mat central" (-0.022, 1.000)
            (0.359, 110) #beacon bottom_left_blue (0.050, -0.094)
            )
        ) 


    



