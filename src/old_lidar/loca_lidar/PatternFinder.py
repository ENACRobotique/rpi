from dataclasses import dataclass
from functools import lru_cache, cached_property#, cache
from itertools import combinations, chain, islice
from copy import deepcopy
from math import isclose, radians, pi, atan, degrees
import numpy as np
import time
from typing import Tuple, NamedTuple, List, Dict
import logging

from loca_lidar.PointsDataStruct import hashabledict
import loca_lidar.CloudPoints as cp

DistPts = NamedTuple('DistPts', [
    ('index_pt1', int), 
    ('index_pt2', int), 
    ('sqrd_dist', float)])


class NotEnoughAmalgames(Exception):
    ...


# Regroupment of amalgame of 'same type' : fixed beacons, amalgames detected by lidar, ...
@dataclass
class GroupAmalgame: 
    points: Tuple[Tuple[float, float], ...] #((x, y), ...) Coordinate of (relative) center of these amalgames
    cartesian: bool = True #True if points are in cartesian coordinates, False if in polar coordinates
    @cached_property
    #calculate and return the distances between all amalgames in format ((amalgame1, amalgame2, distance), ...)
    def distances(self) -> Tuple[DistPts]:
        temp_distances = []
        for point_combination in combinations(enumerate(self.points), 2): 
            #get all possible combination (unique permutation) of points : [((Index PT1, (x,y)), (Index Pt2, (x,y))), ...]
            index_pt1 = point_combination[0][0]
            pt1 = point_combination[0][1]
            index_pt2 = point_combination[1][0]
            pt2 = point_combination[1][1]
            if self.cartesian:
                squared_dist = cp.get_squared_dist_cartesian(pt1, pt2) #calculate squared distance
            else:
                squared_dist = cp.get_squared_dist_polar(pt1, pt2)
            temp_distances.append(DistPts(index_pt1, index_pt2, squared_dist))

        #list of all the distances possibles between the points A/0, B/1, C/2, D/3, E/4   False example : ((0, 1, 2.0), (0,2, 4.4232)
        return tuple(temp_distances)
    
    def sqrd_dist_to_origin(self, pt_i) -> float:
        pt = self.points[pt_i]
        if self.cartesian:
            return cp.get_squared_dist_cartesian(pt, (0,0))
        else:
            return cp.get_squared_dist_polar(pt, (0,0))
    
# Function to convert from polar to cartesian coordinates
def polar_lidar_to_cartesian(polar_coord: List[float]): # distance, degrees
    #input : (r, theta) 
    # r = distance ; theta = angle in DEGREES
    r = polar_coord[0]
    theta = radians(polar_coord[1]) + pi/2 #adding offset
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return np.array([x, y])

def angle_3_pts(a: tuple, b: tuple, c:tuple, is_cartesian = True)-> float:
    # return the angle between 3 points in degrees
    # b angle is the middle one
    if not is_cartesian:
        a = polar_lidar_to_cartesian(a) # type: ignore
        b = polar_lidar_to_cartesian(b) # type: ignore
        c = polar_lidar_to_cartesian(c) # type: ignore
    ang = np.rad2deg(np.arctan2(c[1]-b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0]))
    return ang + 360 if ang < 0 else ang
    

def angle(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return np.arctan2(dy, dx)

class LinkFinder:
    # Tools to find the possible correspondances between two GroupAmalgame (ex : beacons amalgames + lidar amalgames)
    # TODO : Filter the possible correspondances with angles known from the beacons and must be present beacons (experience or poteau fixe)
    def __init__(self, amalg, error_margin, angle_err_marg = 1.5): #DistPts, float
        self.table = amalg
        self.dist_pts_reference = amalg.distances
        self.error_margin = error_margin
        self.angle_err_marg = angle_err_marg
        self._generate_candidate_table_cache()
        nb_pts = len(set(chain.from_iterable(
            (x.index_pt1, x.index_pt2) for x in self.dist_pts_reference)
        )) # get the number of distinct points in the dist_pts
        
        self.table_pivot_dist = {pivot_index: self.get_distances_from_pivot(pivot_index, self.dist_pts_reference) 
            for pivot_index in range(nb_pts)}
        #example dictionnary {0: ((0, 1, 0.3242), (0, 2, 0.231)), ...}
        pass 

    def find_pattern(self, amalg):
        #return dict lidar_to_table association
        #calculate all amalgame distances squared and find if any distance match the ones in self.distances
        #perform a search to find possible matching distances (if nothing found, it's not a corners fixed known of the map)
        dist_pts_lidar = amalg.distances
        lidar_to_table_corr = []
        if len(dist_pts_lidar) <= 1:
            #raise ValueError("dist_pts_lidar size <= 1 | can't find pattern with 2 points or less")
            raise NotEnoughAmalgames()
        for detected_DistPts in dist_pts_lidar:
            #TODO : convert to binary search ?
            for candidate_table_point in self._get_candidates_table_point(detected_DistPts[2]): #check only the first point as "pivot"
                table_dists = self.table_pivot_dist[candidate_table_point]
                dists_from_pivot = self.get_distances_from_pivot(detected_DistPts[0], dist_pts_lidar)
                
                #Test candidate_table_point - finding others distance of candidate/Beacon :
                lidar_to_table_pts = self._lidar2table_from_pivot(candidate_table_point, dists_from_pivot, table_dists)
                if lidar_to_table_pts is not None and len(lidar_to_table_pts) >= 3:  #at least 3 points have been "correlated" between lidar & table frame
                    lidar_to_table_corr.append(lidar_to_table_pts)
        return self._filter_candidate_corr(lidar_to_table_corr, amalg)
                
        return None #no pattern found
    def _filter_candidate_corr(self, lidar_to_table_corr, amalg): #list[dict(correspondances)]
        if len(lidar_to_table_corr) == 0:
            return None

        # Filter the possible correspondances with angles known from the beacons :
        pts = amalg.points
        if not amalg.cartesian:
            pts = [polar_lidar_to_cartesian(p) for p in pts]
        valid_corr = set() # correspondances that seem valid using angle calc # [{} for _ in range(len(lidar_to_table_corr)) ] # avoid having same dict duplicated
        # check all possible correspondances
        for corr_i, corr in enumerate(lidar_to_table_corr):
            cur_valid_corr = hashabledict() # {lidar_index: table_index} hashable dict in order to eliminate the same correspondances that appear multiple times
            pass
            for triangle in combinations(corr, 3):
                # check if the angle of the triangle is the same as the one in the table

                table1_i, table2_i, table3_i = corr[triangle[0]], corr[triangle[1]], corr[triangle[2]]
                if table1_i == table2_i or table1_i == table3_i or table2_i == table3_i:
                    continue # the triangle is not valid
                table_angle1 = angle_3_pts(self.table.points[table1_i], self.table.points[table2_i], self.table.points[table3_i])
                lidar_angle1 = angle_3_pts(pts[triangle[0]], pts[triangle[1]], pts[triangle[2]])
                table_angle2 = angle_3_pts(self.table.points[table2_i], self.table.points[table3_i], self.table.points[table1_i])
                lidar_angle2 = angle_3_pts(pts[triangle[1]], pts[triangle[2]], pts[triangle[0]])
                if (isclose(table_angle1, lidar_angle1, abs_tol=self.angle_err_marg) 
                    and isclose(table_angle2, lidar_angle2, abs_tol=self.angle_err_marg)):
                    # the triangle is valid (in terms of angles)
                    cur_valid_corr[triangle[0]] = table1_i
                    cur_valid_corr[triangle[1]] = table2_i
                    cur_valid_corr[triangle[2]] = table3_i
            valid_corr.add(cur_valid_corr)
        
        # returns the correspondances with the maximum amount of beacons
        max_len = len(max(valid_corr, key=len))
        if max_len == 0:
            return None
        return set(x for x in valid_corr if len(x) == max_len) # max(valid_corr, key=len)




    def _lidar2table_from_pivot(self, candidate_table_point, dists_from_pivot, table_dists) -> Dict:
        # returns : {i_from_pivot:i_from_table, ...} 
        # where the squared_distances between the associated elements of the two arrays areclose(rtol=self.error_pargin)
        pt_lidar_to_table = {}
        for i, dist_pivot in enumerate(dists_from_pivot):
            for j, dist_table in enumerate(table_dists):
                if isclose(dist_pivot.sqrd_dist, dist_table.sqrd_dist, rel_tol=self.error_margin):
                    pt_lidar_to_table[dist_pivot.index_pt2] = dist_table.index_pt2
                    #break #Distances are non_unique and we need to match all, so keep it commented?
        if len(pt_lidar_to_table) >= 2:
            #add the pivot point :
            lidar_i = dists_from_pivot[0].index_pt1
            pt_lidar_to_table[lidar_i] = candidate_table_point

            return pt_lidar_to_table
        return None

    #@cache 
    def _get_candidates_table_point(self, squared_dist)->tuple:
        #returns tuple of possible points in known_points from table reference for a certain squared_dist
        #Example : If distance is close to 2.5 m, it returns possible points (0, 1, 2)/(A,B,C)
        candidates = []
        for table_ref in self.dist_pts_reference:
            if isclose(table_ref[2], squared_dist, rel_tol=self.error_margin):
                if table_ref[0] not in candidates:
                    candidates.append(table_ref[0])
                if table_ref[1] not in candidates:
                    candidates.append(table_ref[1])
        return tuple(candidates)

    def _generate_candidate_table_cache(self):
        for dist_pts in self.dist_pts_reference:
            self._get_candidates_table_point(dist_pts.sqrd_dist)

    @staticmethod
    def get_distances_from_pivot(pt_index: np.int64, pt_distances: List[DistPts]) -> List[DistPts]:
        #return all sqred_distances from the pt given by pt_index 
        #returns format : ((pt_index, other point, squared_dist)) of type DistPts
        
        distances_of_pivot = []
        for pt_dist in pt_distances:
            if pt_dist.index_pt1 == pt_index:
                distances_of_pivot.append(DistPts(pt_dist[0], pt_dist[1], pt_dist[2]))
            elif pt_dist.index_pt2 == pt_index:
                distances_of_pivot.append(DistPts(pt_dist[1], pt_dist[0], pt_dist[2]))  #'sort' the array, so that the order is for each element always (pt_index, other_pt, dist) and not (other_pt, pt_index, dist)

        return distances_of_pivot
    
    @staticmethod
    def _get_rel_pos(pt1, pt2, pt3) -> int:
        #returns the relative position of pt3 to the line defined by pt1 and pt2
        # returns 1 if pt3 is on the left side of the line, -1 if on the right side and 0 if on the line
        # https://stackoverflow.com/questions/1560492/how-to-tell-whether-a-point-is-to-the-right-or-left-side-of-a-line
        val = (pt2[1] - pt1[1]) * (pt3[0] - pt2[0]) - (pt2[0] - pt1[0]) * (pt3[1] - pt2[1])
        if val == 0:
            return 0
        elif val > 0:
            return 1
        else:
            return -1
        
#https://stackoverflow.com/questions/20546182/how-to-perform-coordinates-affine-transformation-using-python-part-2?answertab=votes#tab-top
def lidar_pos_wrt_table(lidar_to_table, lidar_amalgames, fixed_pts)-> Tuple[float, float]:
    """returns average computed (x,y, angle) in meters, meters, radians using Least Square

    Args:
        self (_type_): _description_
        lidar_to_table (dict {int:int}): Association  of points {lidar_amalgames_index:Fixed_Point_index}
        lidar_amalgames (np.ndarray of ndtype PolarPoints): _description_
    """
    # Select correspondences
    lidar_idxs = list(lidar_to_table.keys())
    known_idxs = [lidar_to_table[i] for i in lidar_idxs]
    
    # Convert lidar coordinates to cartesian coordinates
    lidar_coords = np.array([polar_lidar_to_cartesian(lidar_amalgames[i]) for i in lidar_idxs])
    table_coords = np.array([fixed_pts[i] for i in known_idxs])

    #determine using least square the transform equation from lidar to table
    pad = lambda x: np.hstack([x, np.ones((x.shape[0], 1))])
    unpad = lambda x: x[:,:-1]
    X = pad(lidar_coords)
    Y = pad(table_coords)
    A, res, rank, s = np.linalg.lstsq(X, Y, rcond=0.01) #rcond value : Cut-off ratio
    #transform = lambda x: unpad(np.dot(pad(x), A))
    #print(f"Target : {table_coords} \n result : {transform(lidar_coords)}")
    #print( "Max error:", np.abs(table_coords - transform(lidar_coords)).max())

    lidar_wrt_table =  A[2][:2].reshape(2, 1) #calculated from least square optimisation
    return lidar_wrt_table

def lidar_angle_wrt_table(lidar_wrt_table, lidar_to_table, lidar_amalgames, fixed_pts): #TODO : fixedpts default  = fp.known_points()
    """Determine lidar angle compared to vertical axis pointing up in table/world frame
    
    Args:
        lidar_wrt_table (np.array): (x,y) of the lidar on the table/world frame
        lidar_to_table (dict): {index(lidar_amalgames) : index(fixed_pts)} association
        lidar_amalgames (tuple): (r, theta) (meters, degrees)  angles must be all positive (0-360)
        fixed_pts (_type_, optional): _description_. Defaults to FPts.known_points().
    """
    #TODO : avoid code repetition (and lru_cache it ?)
    # Select correspondences
    lidar_idxs = list(lidar_to_table.keys())
    known_idxs = [lidar_to_table[i] for i in lidar_idxs]
    
    # Convert lidar coordinates to cartesian coordinates
    lidar_polar = np.array([(lidar_amalgames[i]) for i in lidar_idxs])
    table_coords = np.array([fixed_pts[i] for i in known_idxs])

    # for each beacon :
    # calculate right triangle ABC with C lidar/beacon, A horizontal beacon, B vertical lidar 
    # we determine the angle using pythagorus, arctan(line a/line b)  ### formulas were determined "experimentaly" using geogebra
    rad_computed_angle = []
    for i, coord in enumerate(table_coords):
        angle_lidar_beacon = lidar_polar[i][1]
        lidar_angle_wrt_table = None
        # if beacon on top left to the lidar position (<x & >y)
        if coord[0] < lidar_wrt_table[0] and coord[1] > lidar_wrt_table[1]: 
            #shape of ABC : ◥
            a = lidar_wrt_table[0] - coord[0]
            b = coord[1] - lidar_wrt_table[1]
            lidar_triangle_angle = degrees(atan(a/b))
            lidar_angle_wrt_table =  lidar_triangle_angle + (360 - angle_lidar_beacon)

        #if beacon on top right to the lidar position (>x, >y)
        elif coord[0] > lidar_wrt_table[0] and coord[1] > lidar_wrt_table[1]:
            #shape of ABC : ◤
            a = coord[0] - lidar_wrt_table[0]
            b = coord[1] - lidar_wrt_table[1]
            lidar_triangle_angle = degrees(atan(a/b))
            lidar_angle_wrt_table = 360 - angle_lidar_beacon - lidar_triangle_angle 

        #if beacon on bottom left to the lidar position (<x, <y)
        elif coord[0] < lidar_wrt_table[0] and coord[1] < lidar_wrt_table[1]:
            #shape of ABC : ◢
            a = lidar_wrt_table[0] - coord[0] 
            b = lidar_wrt_table[1] - coord[1]
            lidar_triangle_angle = degrees(atan(a/b))
            lidar_angle_wrt_table = 180 - lidar_triangle_angle - angle_lidar_beacon

        #if beacon on bottom right to the lidar position (>x, <y)
        elif coord[0] > lidar_wrt_table[0] and coord[1] < lidar_wrt_table[1]:
            #shape of ABC : ◥
            #Here, A : vertical beacon, B : horizontal lidar 
            a = lidar_wrt_table[1] - coord[1] 
            b = coord[0] - lidar_wrt_table[0] 
            lidar_triangle_angle = degrees(atan(a/b))
            lidar_angle_wrt_table = 270 - lidar_triangle_angle - angle_lidar_beacon

        else: 
            logging.warning(f"lidar position {lidar_wrt_table} is perfectly aligned with a beacon - \
                can't determine angle using lidar amalgame {lidar_idxs[i]} and table fixed {known_idxs[i]}  ")

        if lidar_angle_wrt_table != None: 
             # normalize angle to rad
            rad_lidar_angle_wrt_table = np.deg2rad(lidar_angle_wrt_table)
            rad_lidar_angle_wrt_table = np.arctan2(np.sin(rad_lidar_angle_wrt_table),np.cos(rad_lidar_angle_wrt_table))
            rad_computed_angle.append(rad_lidar_angle_wrt_table)

    #TODO : remove after enough testing below testing : 
    averaged_angle_rad = np.array(rad_computed_angle).mean()
    # makes it parallel to x axis instead of y axis
    averaged_angle = np.pi/2 - averaged_angle_rad
    averaged_angle = np.rad2deg(averaged_angle)
    if np.any((rad_computed_angle < averaged_angle_rad - 1.5)|(rad_computed_angle > averaged_angle_rad + 1.5)):
        logging.warning(f"angle triangulation determined mean deviation of more than 0.03 rad/ 1.7° \n \
            angles are {rad_computed_angle} for beacons {table_coords} ")
    return averaged_angle


if __name__ == "__main__":
    pass
    # finder = LinkFinder(fp.known_distances(), 0.02)
    # lidar2table = finder.find_pattern(cp.distances)
    #lidarpos = Triangulate.lidar_pos_wrt_table(lidar2table, cp.amalgame_sample_1)
    #print(Triangulate.lidar_angle_wrt_table(lidarpos, lidar2table, cp.amalgame_sample_1))
