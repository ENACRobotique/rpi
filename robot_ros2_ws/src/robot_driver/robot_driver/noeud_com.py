import rclpy
from rclpy.node import Node
from .comm_bas_niveau import Radio
from nav_msgs.msg import Odometry 
from math import pi, atan2
buffer = 10 #Arbitrary

class RosRadio(Node):

    def __init__(self):
        super().__init__("comm_BN")
        self.get_logger().info("Starting HN BN comm")
        self.radio = Radio()
        self.radio.startListening()
        self.odom_sub = self.create_subscription(Odometry, "/odom_rf2o",self.odom,buffer)
        self.setTargetPos(0.10,0.10,0)
    
    def odom(self,msg:Odometry):
        # THIS IS QATERNIONS NOT EULER ANGLES
        x = -msg.pose.pose.position.x
        y = -msg.pose.pose.position.y

        qx= msg.pose.pose.orientation.x
        qy= msg.pose.pose.orientation.y
        qz= msg.pose.pose.orientation.z
        qw= msg.pose.pose.orientation.w

        yaw = atan2(2 * (qw * qz + qx * qy),1 - 2 * (qy * qy + qz * qz))

        self.get_logger().info(
            " x = " + str(round(x,3)) + 
            " y = " + str(round(y,3)) + 
            " yaw = " +str(round(yaw*180/pi,3)) )
        
        self.radio.resetPosition(x,y,yaw)


    def setTargetPos(self,x,y,yaw):
        self.radio.setTargetPosition(x,y,yaw)
        



def main(args = None):
    rclpy.init(args=args)
    node = RosRadio()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=='__main__':
    main()
