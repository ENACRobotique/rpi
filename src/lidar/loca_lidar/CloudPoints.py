# Set of tools to manage points from 2D lidar cloud points and creation of amalgames
import logging
from math import cos, radians, dist, atan2, pi
import numpy as np
from typing import Tuple, List, Union
try:
    from loca_lidar.PointsDataStruct import PolarPts, DistPts, AmalgamePolar, AmalgameCartesian, \
        PolarPts_t, AmalgamePolar_t
    import loca_lidar.config as config
    from loca_lidar.SmallestEnclosingCircle import make_circle as make_enclosing_circle
    from loca_lidar.MinimumBoundingBox import MinimumBoundingBox as min_bounding_box
except ModuleNotFoundError:
    from PointsDataStruct import PolarPts, DistPts, AmalgamePolar, AmalgameCartesian, \
    PolarPts_t, AmalgamePolar_t
    import config as config
    from SmallestEnclosingCircle import make_circle as make_enclosing_circle
    from MinimumBoundingBox import MinimumBoundingBox as min_bounding_box
def get_squared_dist_cartesian(pt1=(0,0), pt2= (0,0)): #x,y
    return (pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2

#https://math.stackexchange.com/questions/1506706/how-to-calculate-the-distance-between-two-points-with-polar-coordinates
def get_squared_dist_polar(pt1, pt2):
    r1 = pt1[0]
    r2 = pt2[0]
    theta1 = pt1[1]
    theta2 = pt2[1]
    return r1**2 + r2**2 - 2 * r1 * r2 * cos(radians(theta2 - theta1))

def cart2pol(x, y):
    # returns phi in degrees
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    phi = np.rad2deg(phi)
    phi = phi if phi >= 0 else phi + 360
    return(rho, phi)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

def fit_circle(point_set):
    # https://github.com/tysik/obstacle_detector/blob/master/include/obstacle_detector/utilities/figure_fitting.h
    # Conversion C++ -> Python by chatGPT
    N = len(point_set)
    assert N >= 3

    input = np.zeros((N, 3))
    output = np.zeros(N)
    params = np.zeros(3)

    i = 0
    for point in point_set:
        input[i, 0] = point[0]
        input[i, 1] = point[1]
        input[i, 2] = 1.0

        output[i] = -(point[0]**2 + point[1]**2)
        i += 1

    # Find a_1, a_2 and a_3 coefficients from linear regression
    params = np.linalg.pinv(input) @ output

    center = (-params[0] / 2.0, -params[1] / 2.0)
    radius = np.sqrt((params[0]**2 + params[1]**2) / 4.0 - params[2])

    return center, radius
# function to remove incoherent points from lidar scan (to pre-filter them)
def basic_filter_pts(pts: PolarPts_t) -> PolarPts_t:
    # pt should be at least within 5 cm & 3 meters distance from lidar to be in the table
    mask = (pts['distance'] < config.max_lidar_dist) & (pts['distance'] > config.min_lidar_dist)
    return pts[mask]

def position_filter_pts(pts: PolarPts_t, lidar_x, lidar_y, lidar_theta) -> PolarPts_t:
    # TODO 1. Add additionnal check with the last known lidar position on the table
    return pts

def obstacle_in_path(robot_pose: Tuple, pts: List[List[Union[float, float]]], target: Tuple) -> int:
    """_summary_

    Check if obstacle in path to target position (assuming straight line from cur pose to target)
    Args:
        pts (PolarPts_t): _description_
        speed (tuple): _description_

    Raises:
        ValueError: _description_

    Returns:
        int: 0 : ok, 1 : warning, 2 : stop
    """
    max_alert = 0 # level depends on obstacle found within the rectangle
    direction_vec = (target[0] - robot_pose[0], target[1] - robot_pose[1])

    # 1. Calculate rectangle for stop
    #### STOP Rectangle ###
    stop_x_edge1, stop_y_edge1, \
    stop_x_edge2, stop_y_edge2, \
    stop_x_edge3, stop_y_edge3, \
    stop_x_edge4, stop_y_edge4 = _get_rectangle_edge(
        robot_pose[0], robot_pose[1], target[0], target[1], 
        direction_vec[0], direction_vec[1], config.stop_cyl_width, config.stop_cyl_dist)
    
    #### WARNING Rectangle ###
    warn_x_edge1, warn_y_edge1, \
    warn_x_edge2, warn_y_edge2, \
    warn_x_edge3, warn_y_edge3, \
    warn_x_edge4, warn_y_edge4 = _get_rectangle_edge(
        robot_pose[0], robot_pose[1], target[0], target[1], 
        direction_vec[0], direction_vec[1], config.warning_cyl_width, config.warning_cyl_dist)

    x_a, y_a, x_b, y_b, x_c, y_c, x_d, y_d = _determine_edge(
        stop_x_edge1, stop_y_edge1, stop_x_edge2, stop_y_edge2, stop_x_edge3, stop_y_edge3, stop_x_edge4, stop_y_edge4)

    x_e, y_e, x_f, y_f, x_g, y_g, x_h, y_h = _determine_edge(
        warn_x_edge1, warn_y_edge1, warn_x_edge2, warn_y_edge2, warn_x_edge3, warn_y_edge3, warn_x_edge4, warn_y_edge4)
    #test 1&2, 2&4, 4&3, 3&1
    for pt in pts:

        #2. Check if inside rectangle
        # https://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not#
        if (_is_on_left_edge(x_a, y_a,x_b, y_b, pt[0], pt[1]) and
            _is_on_left_edge(x_b, y_b, x_c, y_c, pt[0], pt[1]) and
            _is_on_left_edge(x_c, y_c, x_d, y_d, pt[0], pt[1]) and
            _is_on_left_edge(x_d, y_d, x_a, y_a, pt[0], pt[1])):
            return 2 # STOP
        
        if (_is_on_left_edge(x_e, y_e,x_f, y_f, pt[0], pt[1]) and
            _is_on_left_edge(x_f, y_f, x_g, y_g, pt[0], pt[1]) and
            _is_on_left_edge(x_g, y_g, x_h, y_h, pt[0], pt[1]) and
            _is_on_left_edge(x_h, y_h, x_e, y_e, pt[0], pt[1])):
            max_alert = 1

        # if within circle stop distance
        dist_from_lidar = np.sqrt(get_squared_dist_cartesian((pt[0], pt[1]), (robot_pose[0], robot_pose[1])))
        if ( dist_from_lidar < config.stop_circular_dist
            and dist_from_lidar > config.robot_radius):
            return 2

    return max_alert # returns 0 if ok, 1 if warning, 2 if stop

def _get_rectangle_edge(robot_x, robot_y, next_x, next_y, speed_x, speed_y, width, length):
    # calcualte rectangle edge for collision box drawing, width = from robot POV, length = furthest distance from robot POV
        # find third point of right triangle
    #https://math.stackexchange.com/questions/2125690/find-coordinates-of-3rd-right-triangle-point-having-2-sets-of-coordinates-and-a
    L = np.sqrt(np.power(robot_x-next_x, 2) + np.power(robot_y - next_y, 2)) #sqrt((x2-x1)² + (y2-y1)²)
    C = width / 2
    
    x_edge1 = robot_x + (C * (next_y - robot_y)) / L
    y_edge1 = robot_y + (C * (robot_x - next_x)) / L
    x_edge2 = robot_x - (C * (next_y - robot_y)) / L
    y_edge2 = robot_y - (C * (robot_x - next_x)) / L

    #https://stackoverflow.com/questions/41317291/setting-the-magnitude-of-a-2d-vector
    ratio_magnitude = length / np.linalg.norm([speed_x, speed_y]) # factor to multiply the vector x/y
    x_edge3 = x_edge2 + speed_x * ratio_magnitude #extension of edge 2
    y_edge3 = y_edge2 + speed_y * ratio_magnitude
    x_edge4 = x_edge1 + speed_x * ratio_magnitude #extension of edge 1
    y_edge4 = y_edge1 + speed_y * ratio_magnitude

    return (x_edge1, y_edge1, x_edge2, y_edge2,
        x_edge3, y_edge3, x_edge4, y_edge4)

def _determine_edge(x_edge1, y_edge1, x_edge2, y_edge2, x_edge3, y_edge3, x_edge4, y_edge4):
    # sort edges in anticlockwise order
    all_x = [x_edge1, x_edge2, x_edge3, x_edge4]
    all_y = [y_edge1, y_edge2, y_edge3, y_edge4]
    all_angle = [0, 0, 0, 0]

    #sort all y and all x in anticlockwise order :
    # https://stackoverflow.com/a/1709546 (I can't explain how it works sorry, pure copy pasta)
    mx = sum(x for x in all_x) / len(all_x)
    my = sum(y for y in all_y) / len(all_y)
    for i in range(len(all_x)):
        all_angle[i] = (atan2(all_x[i] - mx, all_y[i] - my) + 2 * pi) % (2*pi)

    all_angle, all_x, all_y = zip(*sorted(zip(all_angle, all_x, all_y), reverse=True))
    return all_x[0], all_y[0], all_x[1], all_y[1], all_x[2], all_y[2], all_x[3], all_y[3]


def _is_on_left_edge(x1,y1, x2, y2, xp, yp):
    return True if (x2 - x1) * (yp - y1) - (xp - x1) * (y2 - y1) >= 0 else False

def _calculate_intersection_circles(pt1, radius1, pt2, radius2):
    """_summary_

    Args:
        pt1 (_type_): _description_
        pt2 (_type_): _description_

    Returns:
        Tuple : ((x1, y1), (x2, y2))
    """
    # https://stackoverflow.com/questions/3349125/circle-circle-intersection-points
    d = np.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)
    a = (radius1**2 - radius2**2 + d**2) / (2 * d)
    h = np.sqrt(radius1**2 - a**2)
    x3 = pt1[0] + a * (pt2[0] - pt1[0]) / d
    y3 = pt1[1] + a * (pt2[1] - pt1[1]) / d
    x4a = x3 + h * (pt2[1] - pt1[1]) / d
    x4b = x3 - h * (pt2[1] - pt1[1]) / d
    y4a = y3 - h * (pt2[0] - pt1[0]) / d
    y4b = y3 + h * (pt2[0] - pt1[0]) / d
    return ((x4a, y4a), (x4b, y4b))
    
def _fill_amalgame(amalgame:AmalgamePolar_t) -> AmalgamePolar_t:
    last_i = np.max(np.nonzero(amalgame['list_pts']['distance']))
    pts = amalgame['list_pts'][:last_i+1]

    # Calculate amalgame size :
    first_pt, last_pt = amalgame['list_pts'][0], amalgame['list_pts'][last_i]
    amalgame['size'] = np.sqrt(get_squared_dist_polar(first_pt, last_pt))

    # Calculate amalgame relative center 
    avg_pts_dist = amalgame['list_pts']['distance'][:last_i+1].mean()
    avg_pts_angle = amalgame['list_pts']['angle'][:last_i+1].mean()
    amalgame['center_polar']['distance'] = avg_pts_dist
    amalgame['center_polar']['angle'] = avg_pts_angle

    # if size might be a pylone, more than 3 points => Find center of a circle through fit_circle 
    if amalgame['size'] <= config.pylone_diam and amalgame['list_pts'][:last_i+1].size >= 3:
        # calculate the center of pylone by finding the furthest point 
        # made from the intersection of circle of pylone radius from the two extremities
        first_pt_cart = tuple(pol2cart(pts[0][0], np.deg2rad(pts[0][1])))
        second_pt_cart = tuple(pol2cart(pts[-1][0], np.deg2rad(pts[-1][1])))
        possible_center_cart = _calculate_intersection_circles(first_pt_cart, config.pylone_radius, 
                                        second_pt_cart, config.pylone_radius)
        center_pol_1 = cart2pol(possible_center_cart[0][0], possible_center_cart[0][1])
        center_pol_2 = cart2pol(possible_center_cart[1][0], possible_center_cart[1][1])
        # if no circle intersection : find half of line segment between first and last point
        if center_pol_1[0] == None or center_pol_2[0] == None:
            amalgame['center_polar']['distance'] = (pts[0]['distance'] + pts[-1]['distance'] )/2
            amalgame['center_polar']['angle'] = (pts[0]['angle'] + pts[-1]['angle'] )/2
        # if center_pol_1 is the furtest from lidar
        elif center_pol_1[0] >= center_pol_2[0]:
            amalgame['center_polar']['distance'] = center_pol_1[0]
            amalgame['center_polar']['angle'] = center_pol_1[1]
        elif center_pol_2[0] != None:
            amalgame['center_polar']['distance'] = center_pol_2[0]
            amalgame['center_polar']['angle'] = center_pol_2[1]
    
    if amalgame['size'] >= config.pylone_diam and amalgame['list_pts'][:last_i+1].size >= 3:
        closest_pt = pts[np.argmin(pts['distance'])]
        closest_dist = closest_pt['distance']

        amalgame['center_polar']['distance'] = closest_pt['distance']
        amalgame['center_polar']['angle'] = closest_pt['angle']
    
    return amalgame

def _fusion_amalgames(amalgame1:AmalgamePolar_t, amalgame2:AmalgamePolar_t, angle_norm = True) -> AmalgamePolar_t:
    # Function to manage the case when an amalgame is present at both beggining and end of scan
    # angle_norm : set to true to manage case of when angles are close to 360 and close to 0 for mean computation
    i_first_zero = np.where(amalgame1['list_pts']['distance'] == 0)[0][0]
    amalgame1['list_pts'] = np.concatenate((amalgame1['list_pts'][:i_first_zero], amalgame2['list_pts'][:-i_first_zero]))

    # recalculate size & center
    amalgame1 = _fill_amalgame(amalgame1)
    #manage case if fusion from points close to 360 AND 0 at the same time : 
    # https://stackoverflow.com/a/491784
    if angle_norm:
        last_i = np.where(amalgame1['list_pts']['distance'] == 0)[0][0]
        sum_sin = np.sum(np.sin(np.deg2rad(amalgame1['list_pts'][:last_i]['angle'])))
        sum_cos = np.sum(np.cos(np.deg2rad(amalgame1['list_pts'][:last_i]['angle'])))
        angle = np.rad2deg(np.arctan2(sum_sin, sum_cos))
        angle = angle if angle >= 0 else 360 + angle
        amalgame1['center_polar']['angle'] = angle

    return amalgame1

def amalgames_from_cloud(pts: PolarPts_t) -> AmalgamePolar_t:
    amalgames = np.zeros((30, ), dtype=AmalgamePolar) # Hypothesis that we won't detect more than 20 valid amalgames per scan
    amalg_i, amalg_pt_count = 0, 0
    #TODO : optimize below using numpy
    # Known limitation : if an amalgame made of two points is present only at the beggining and end of filtered scan, it won't be detected
    # But we consider that an amalgame should be made of at least three points
    # sort pts by angle
    pts = np.sort(pts, order='angle')
    for i, pt in enumerate(pts):
        #initialize first cur_pt
        if amalg_pt_count == 0: 
            amalgames[amalg_i]['list_pts'][amalg_pt_count] = pt
            amalg_pt_count += 1
            continue

        cur_pt = amalgames[amalg_i]['list_pts'][amalg_pt_count -1] #last added point in the amalgame

        # makes sure to finish last amalgame and adding last pt
        if i == pts.size - 1 and get_squared_dist_polar(cur_pt, pt) <= config.amalgame_squared_dist_max:
            amalgames[amalg_i]['list_pts'][amalg_pt_count] = pt

        # if next point is not part of the amalgame currently being discovered, finish it
        # or if finishing pts
        if (get_squared_dist_polar(cur_pt, pt) > config.amalgame_squared_dist_max
            or i == pts.size - 1 # this condition makes sure to finish the last amalgame if reaching the end of the pts list
            or amalg_pt_count >= config.amalg_max_nb_pts - 1): # Prevents IndexError if an amalgame is too big. 
            if amalg_pt_count >= 1:
                #calculate rel center & size : 
                amalgames[amalg_i] = _fill_amalgame(amalgames[amalg_i])
                amalg_i += 1 
            else: #we start another amalgame from the same index 
                amalgames[amalg_i] = np.zeros((1, ))
            amalg_pt_count = 1
            try:
                amalgames[amalg_i]['list_pts'][0] = pt
            except IndexError:
                logging.info("IndexError : too many amalgames detected (>30). Skipping the rest of the scan"	)
                logging.info("last point - unprocessed ", pt)
                break
            continue

        # check if point could be part of the same amalgame as pt, and add it if so
        if get_squared_dist_polar(cur_pt, pt) <= config.amalgame_squared_dist_max:
            amalgames[amalg_i]['list_pts'][amalg_pt_count] = pt
            amalg_pt_count += 1
        
    # if first and last amalgame actually belong to the same amalgame, fusion them
    if amalg_i > 1: #if more than 2 amalgames detected
        #get index of last point added to amalgame
        last_i = np.max(np.nonzero(amalgames[amalg_i-1]['list_pts']['distance']))
        first_amalg_last_i = np.max(np.nonzero(amalgames[0]['list_pts']['distance']))
        first_pt_first_amalg = amalgames[0]['list_pts'][0]
        last_pt_last_amalg = amalgames[amalg_i-1]['list_pts'][last_i]
        if (get_squared_dist_polar(first_pt_first_amalg, last_pt_last_amalg) <= config.amalgame_squared_dist_max
            and first_amalg_last_i + last_i <= config.amalg_max_nb_pts - 1): # Avoid fusion of amalgame if the number of points will be too big
            amalgames[0] = _fusion_amalgames(amalgames[0], amalgames[amalg_i-1])
            amalgames[amalg_i-1] = np.zeros((1, ))

    amalgames = remove_none_amalgames(amalgames)
    return amalgames

def filter_amalgame_size(amalgames:AmalgamePolar_t) -> AmalgamePolar_t:
    #if amalgame is valid ("coherent size" ex :  not a wall, not a referee, ...)
    mask = np.where((amalgames['size'] > config.amalg_min_size) & (amalgames['size'] < config.amalg_max_size))
    valid_amalgames = amalgames[mask]
    return valid_amalgames

def remove_none_amalgames(amalgames:AmalgamePolar_t) -> AmalgamePolar_t:
    #if amalgame is valid ("coherent size" ex :  not a wall, not a referee, ...)
    mask = np.where(np.logical_and(
        amalgames['center_polar']['distance'] != 0, amalgames['center_polar']['angle'] != 0)
    )
    valid_amalgames = amalgames[mask]
    return valid_amalgames

# This function filter out list of points, to only keep center of amalgames in polar coords
def amalgame_numpy_to_tuple(amalgames:AmalgamePolar_t) -> Tuple:
    last_i = np.max(np.nonzero(amalgames['center_polar']['distance']))
    return tuple(amalgames[:last_i+1]['center_polar'])

if __name__ == '__main__':
    #al = obstacle_in_path((0.5, 0.5, 0.0), [(0.01, 0.01)], (-0.1, -0.1, 0.0))
    al = obstacle_in_path((1.62291,1.229, 0.0), [(1.777, 1.48)], (1.0, 0.65, 0.0))
    print(al)
    # left edge (0.41, 0.59)
    # right edge (0.59, 0.41)
    # top left edge (0.59, 0.77)
    #top right edge (0.77, 0.59)

    #trigger : (0.59, 0.72), (0.58, 0.48), (0.65, 0.65), (0.62, 0.42)
    #not trigger : (0.68, 0.78), (0.41, 0.55), (0.64, 0.39)
    pass
    # get_distances(amalgame_sample_1)
    
    #print(get_distances_from_pivot(1, distances))
