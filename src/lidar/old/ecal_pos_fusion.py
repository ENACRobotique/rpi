import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher
import time, os, sys
from math import radians
sys.path.append(os.path.join(os.path.dirname(__file__), '../..')) # Avoids ModuleNotFoundError when finding generated folder
import generated.robot_state_pb2 as robot_pb
from lidar.old.position_smooth import Smoother

DEBUG = True

pos_smoother = Smoother([], [], [], [], 2000.0, 10, 3) # set timestamp to really high for when replaying
max_x_deviation = 0.1 # meters
max_y_deviation = 0.1
max_theta_deviation = 0.15 # rad (~10 degrees)

ecal_core.initialize([], "position_fusion")

sub_lidar = ProtoSubscriber("lidar_pos", robot_pb.Position)
sub_speed = ProtoSubscriber("odom_speed",robot_pb.Speed)
pub_pos = ProtoPublisher("smooth_pos", robot_pb.Position)

last_speed = ()

def logger(msg):
    if DEBUG:
        ecal_core.log_setlevel(1)
        ecal_core.log_message(str(msg))

def logger_warn(msg):
    if DEBUG:
        print(str(msg))
        ecal_core.log_setlevel(2)
        ecal_core.log_message(str(msg))
        ecal_core.log_setlevel(1)

def on_lidar_pos(topic_name, lidar_msg , time):
    global last_speed
    if last_speed == ():
        logger_warn("ecal_pos_fusion : not receiving speed (odom_speed) from robot !")
        return
    if (max_x_deviation - abs(last_speed[0]) > 0 #speed is slow enough to smooth
            and max_y_deviation - abs(last_speed[1]) > 0 
            and max_theta_deviation - abs(last_speed[2]) > 0):
        pos_smoother.add_data(lidar_msg.x, lidar_msg.y, lidar_msg.theta, time)
    else:
        pos_smoother.flush_data()
    smooth_pos = pos_smoother.calc_smooth()
    if smooth_pos:
        pub_pos.send(robot_pb.Position(x=smooth_pos[0], y=smooth_pos[1], theta=smooth_pos[2]), time=time)

def on_odom_speed(topic_name, speed_msg , time):
    global last_speed
    last_speed = (speed_msg.vx, speed_msg.vy, speed_msg.vtheta)

if __name__ == "__main__":
    
    sub_lidar.set_callback(on_lidar_pos)
    sub_speed.set_callback(on_odom_speed)

    while ecal_core.ok():
        time.sleep(0.5)

    ecal_core.finalize()
