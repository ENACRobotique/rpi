import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher
import time, os, sys
import numpy as np
from math import radians, atan2, dist
from loca_lidar.config import loca_theta_offset
#sys.path.append(os.path.join(os.path.dirname(__file__), '../..')) # Avoids ModuleNotFoundError when finding generated folder
import generated.lidar_data_pb2 as lidar_data
import generated.robot_state_pb2 as robot_data

MIN_RADIUS = 20.0e-2    # Filter points on robot
TOLERANCE = 5.0e-2      # Margin around the table

# Warning radius: start sending slow messages
SLOW_RADIUS = 60.0e-2   
SLOW_DEPTH = 80.0e-2
SLOW_WIDTH = 60.0e-2

# Panic radius: immediately send stop messages
STOP_RADIUS = 30.0e-2   
STOP_DEPTH = 50.0e-2
STOP_WIDTH = 30.0e-2

MAX_RADIUS = max([SLOW_DEPTH, SLOW_RADIUS, SLOW_WIDTH, STOP_DEPTH, STOP_RADIUS, STOP_WIDTH])
MIN_DISTANCE = STOP_RADIUS # Stop applying rectangles strategy when closer than MIN_DISTANCE and revert back to circles only
LIDAR_OFFSET = np.radians(loca_theta_offset) + np.pi

last_status = 'stop'
last_position = [0.0, 0.0, 0.0] # (x, y, theta)
set_position = [0.0, 0.0, 0.0]
lidar_data_sub = None
set_position_sub = None
odom_position_pub = None
proximity_status_pub = None
lidar_filtered_pub = None

def send_lidar_scan(pub, distances, angles):
    lidar_msg = lidar_data.Lidar()
    lidar_msg.angle_increment = float(-1.0) # prevent empty message when sending empty lidar scan (eg no obstacle found)
    lidar_msg.angles.extend(angles)
    lidar_msg.distances.extend(distances)
    pub.send(lidar_msg, ecal_core.getmicroseconds()[1])


def compute_distances(topic_name, msg, time):
    global last_status
    global last_position
    global proximity_status_pub
    global lidar_filtered_pub

    distances = np.array(msg.distances)/1000 # in meters
    angles = np.radians(np.array(msg.angles))

    # Filter points that are too far or too close to be useful = range filter
    # => remove useless data for better performance
    #-----------------------------------------------------------------------------------------------
    indexes_range = np.logical_and(distances > MIN_RADIUS, distances <= MAX_RADIUS).nonzero()

    distances_filter_range = distances[indexes_range]
    angles_filter_range = angles[indexes_range] - LIDAR_OFFSET
    #-----------------------------------------------------------------------------------------------

    # Coordinate transform to remove points outside the table
    #-----------------------------------------------------------------------------------------------
    x_table = distances_filter_range * np.cos(angles_filter_range + last_position[2]) + last_position[0]
    y_table = distances_filter_range * np.sin(angles_filter_range + last_position[2]) + last_position[1]

    cond_x_table = np.logical_and(x_table > TOLERANCE, x_table < (3.0 - TOLERANCE))
    cond_y_table = np.logical_and(y_table > TOLERANCE, y_table < (2.0 - TOLERANCE))
    indexes_table = np.logical_and(cond_x_table, cond_y_table).nonzero()

    distances_filter_table = distances_filter_range[indexes_table]
    angles_filter_table = angles_filter_range[indexes_table]
    #-----------------------------------------------------------------------------------------------


    if set_position is not None and dist(set_position[:2], last_position[:2]) > MIN_DISTANCE:
        # Coordinate transform to find points inside slow and halt squares
        # ! ONLY IF WE HAVE A SET POSITION AND ARE FAR AWAY ENOUGH !
        #-----------------------------------------------------------------------------------------------
        # Finding direction vector
        dir_x, dir_y = set_position[0] - last_position[0], set_position[1] - last_position[1]
        dir_angle = atan2(dir_y, dir_x)
        
        x_dir = - distances_filter_table * np.sin(angles_filter_table - (dir_angle - last_position[2]))
        y_dir = distances_filter_table * np.cos(angles_filter_table - (dir_angle - last_position[2]))
        x_dir_abs = np.abs(x_dir)

        cond_x_dir_slow = x_dir_abs <= SLOW_WIDTH
        cond_x_dir_stop = x_dir_abs <= STOP_WIDTH
        cond_y_dir_inf = y_dir > 0.0
        cond_y_dir_slow = np.logical_and(y_dir <= SLOW_DEPTH, cond_y_dir_inf)
        cond_y_dir_stop = np.logical_and(y_dir <= STOP_DEPTH, cond_y_dir_inf)
        cond_dir_slow = np.logical_and(cond_x_dir_slow, cond_y_dir_slow) # Points in slow rect
        cond_dir_stop = np.logical_and(cond_x_dir_stop, cond_y_dir_stop) # Points in stop rect

        indexes_dir_slow = cond_dir_slow.nonzero()
        indexes_dir_stop = cond_dir_stop.nonzero()

        distances_filter_dir_slow = distances_filter_range[indexes_dir_slow]
        distances_filter_dir_stop = distances_filter_range[indexes_dir_stop]
        #-----------------------------------------------------------------------------------------------

        # send lidar points that are detected on table (for visualization debug purposes)
        send_lidar_scan(lidar_filtered_pub, 
                        distances_filter_table[indexes_dir_stop], 
                        np.degrees(angles_filter_table[indexes_dir_stop] + LIDAR_OFFSET)) # convert to positive deg angles

        msg = lidar_data.Proximity()
        closest = '?'
        if len(distances_filter_dir_stop) > 0:
            current_status = 'stop'
            msg.status = lidar_data.ProximityStatus.STOP
            closest = np.min(distances_filter_dir_stop)
            msg.closest_distance = closest
        elif len(distances_filter_dir_slow) > 0:
            current_status = 'slow'
            msg.status = lidar_data.ProximityStatus.WARNING
            closest = np.min(distances_filter_dir_slow)
            msg.closest_distance = closest
        else:
            current_status = 'ok'
            msg.status = lidar_data.ProximityStatus.OK

        if current_status == last_status:
            proximity_status_pub.send(msg)
            print('strategy rects:', current_status, closest)
        
        last_status = current_status 

    else:
        # send lidar points that are detected on table (for visualization debug purposes)
        send_lidar_scan(lidar_filtered_pub, 
                        distances_filter_table, 
                        np.degrees(angles_filter_table + LIDAR_OFFSET)) # convert to positive deg angles

        msg = lidar_data.Proximity()
        if len(distances_filter_table) == 0:
            current_status = 'ok'
            msg.status = lidar_data.ProximityStatus.OK

        else:
            closest = np.min(distances_filter_table)

            # Check cylinder location and send messages to robot
            msg.closest_distance = closest

            if closest <= STOP_RADIUS:
                current_status = 'stop'
                msg.status = lidar_data.ProximityStatus.STOP
            elif closest <= SLOW_RADIUS:
                current_status = 'slow'
                msg.status = lidar_data.ProximityStatus.WARNING
            else:
                current_status = 'ok'
                msg.status = lidar_data.ProximityStatus.OK


        if current_status == last_status:
            proximity_status_pub.send(msg)
            print('strategy circles:', current_status)
        
        last_status = current_status 
    

def get_last_position(topic_name, msg, time):
    global last_position
    last_position[0] = msg.x/1000
    last_position[1] = msg.y/1000
    last_position[2] = msg.theta


def get_set_position(topic_name, msg, time):
    global set_position
    set_position[0] = msg.x/1000
    set_position[1] = msg.y/1000
    set_position[2] = msg.theta

    
if __name__ == '__main__':
    print('Starting ultimate-fallback collision avoidance')
    ecal_core.initialize(sys.argv, "lidar_evitement")

    proximity_status_pub    = ProtoPublisher("proximity_status", lidar_data.Proximity)
    lidar_filtered_pub      = ProtoPublisher("lidar_filtered", lidar_data.Lidar)
    
    lidar_data_sub      = ProtoSubscriber("lidar_data", lidar_data.Lidar)
    lidar_data_sub.set_callback(compute_distances)
    odom_position_sub   = ProtoSubscriber('odom_pos', robot_data.Position)
    odom_position_sub.set_callback(get_last_position)
    set_position_sub    = ProtoSubscriber('set_position', robot_data.Position)
    set_position_sub.set_callback(get_set_position)

    while ecal_core.ok():
        time.sleep(0.01)

    ecal_core.finalize()
