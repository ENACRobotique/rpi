#!/usr/bin/env python3
import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher, StringPublisher
from ecal.core.subscriber import ProtoSubscriber, StringSubscriber
import time
from math import sqrt, pi, cos, sin, atan2 
import sys
import generated.robot_state_pb2 as robot_pb
import generated.lidar_data_pb2 as lidar_pb

from enum import Enum
from dataclasses import dataclass
import numpy as np
import nav 

XY_ACCURACY = 20  # mm
THETA_ACCURACY = 0.05 # radians

class Frame(Enum):
    TABLE = 0
    ROBOTCENTRIC = 1
    ROBOT = 2

@dataclass
class Pos:
    x: float
    y: float
    theta: float

    def __add__(self, other):
        return Pos(self.x + other.x, self.y + other.y, self.theta + other.theta)
    
    def __sub__(self, other):
        return Pos(self.x - other.x, self.y - other.y, self.theta - other.theta)

    def to_proto(self):
        return robot_pb.Position(x=self.x, y=self.y, theta=self.theta)
    
    @staticmethod
    def from_proto(p: robot_pb.Position):
        return Pos(p.x, p.y, p.theta)

    @staticmethod
    def from_np(p: np.array):
        return Pos(p[0], p[1], p[2])
    
    def to_frame(self, new_frame):
        """
        pos: position in the current frame
        new_frame: frame that pos will be converted to
        """
        dp = self - new_frame
        ct = cos(new_frame.theta)
        st = sin(new_frame.theta)
        rot = np.array([[ct,  st, 0],
                        [-st, ct, 0],
                        [0,   0,  1]])
        new_pos = rot.dot(np.array([dp.x, dp.y, dp.theta]))
        return Pos.from_np(new_pos)
    
    def from_frame(self, pos_frame):
        """
        pos: position in the pos_frame
        pos_frame: frame 'position' is expressed on, in the current frame
        """
        ct = cos(pos_frame.theta)
        st = sin(pos_frame.theta)
        rot = np.array([[ct,  st, 0],
                        [-st, ct, 0],
                        [0,   0,  1]])
        rot = rot.transpose()
        pos = rot.dot(np.array([self.x, self.y, self.theta]))
        pos = Pos.from_np(pos) + pos_frame
        return pos

@dataclass
class Speed:
    vx: float
    vy: float
    vtheta: float

    def to_proto(self):
        return robot_pb.Speed(x=self.vx, y=self.vy, theta=self.vtheta)
    
    @staticmethod
    def from_proto(s: robot_pb.Speed):
        return Speed(s.vx, s.vy, s.vtheta)


class Actionneur(Enum):
    Pince1 = 3
    Pince2 = 2
    Pince3 = 5
    Pince4 = 6
    Bras   = 4
    Pano   = 1
    AxL    = 7
    AxR    = 8

class ValeurActionneur(Enum):
    InitPince1 = 870
    InitPince2 = 870
    InitPince3 = 1700
    InitPince4 = 1400
    InitBras = 950
    InitPano = 1500
    InitAxL = 500
    InitAxR = 640

    OpenPince1 = 870
    OpenPince2 = 870
    OpenPince3 = 1700
    OpenPince4 = 1400
    
    ClosePince1 = 870
    ClosePince2 = 870
    ClosePince3 = 1700
    ClosePince4 = 1400
    
    DownBras = 1960
    UpBras = InitBras
    
    UpAxL = 500
    UpAxR = 640

    MidAxL = 500
    MidAxR = 640    

    DownAxL = 500
    DownAxR = 640

PANO_CONVERT = 90/105 # 2.5 cm
PANO_OFFSET = 125 # mm

class Robot:
    """Classe dont le but est de se subscribe à ecal pour avoir une représentation de l'état du robot
    
    Créez un objet de cette classe si vous avez besoin de connaître l'état du robot."""
    def __init__(self):
        """x et y sont en mètres
        theta est en radian"""
        ecal_core.initialize(sys.argv, "robotStateHolder")
        self.pos = Pos(0, 0, 0)
        self.speed = Speed(0, 0, 0)
        self.last_target = Pos(0, 0, 0)
        self.nav = nav.Nav()
        self.pano_angle = 0
        self.command_sent = False

        self.aruco_y = 0
        self.aruco_x = 0
        self.aruco_theta = 0

        #self.tirette = robot_pb.IHM.T_NONE
        #self.color = robot_pb.IHM.C_NONE
        #self.proximityStatus = None

        #self.lastCommandNumber = None
        #self.lastFinishedActionNumber = None
        #self.pointsEstimes =0

        self.tempsDebutMatch = None

        ### SUB ECAL ###

        #self.matchReportSub = ProtoSubscriber('match_start',robot_pb.Match)
        #self.matchReportSub.set_callback(self.onReceiveMatchStarted)

        self.positionReportSub = ProtoSubscriber("odom_pos",robot_pb.Position)
        self.positionReportSub.set_callback(self.onReceivePosition)

        self.speedReportSub = ProtoSubscriber("odom_speed",robot_pb.Speed)
        self.speedReportSub.set_callback(self.onReceiveSpeed)

        self.pano_sub = ProtoSubscriber("aruco",robot_pb.Position_aruco)
        self.pano_sub.set_callback(self.aruco)
        
        #self.setPositionSub = ProtoSubscriber("set_position", robot_pb.Position)
        #self.setPositionSub.set_callback(self.onSetTargetPostition)

        #self.proximitySub = ProtoSubscriber("proximity_status",lidar_pb.Proximity)
        #self.proximitySub.set_callback(self.onProximityStatus)

        #self.ihmSub = ProtoSubscriber("ihm",robot_pb.IHM)
        #self.ihmSub.set_callback(self.on_ihm)

        
        ### PUB ECAL ###
        self.set_target_pos_pub = ProtoPublisher("set_position", robot_pb.Position)
        self.reset_pos_pub = ProtoPublisher("reset", robot_pb.Position)

        self.IO_pub = ProtoPublisher("Actionneur",robot_pb.IO)

        #self.claw_pub = ProtoPublisher("set_pince", robot_pb.SetState)
        #self.score_pub = ProtoPublisher("set_score", robot_pb.Match)
                
        #self.slow_pub = ProtoPublisher("slow",robot_pb.no_args_func_)
        #self.stop_pub = ProtoPublisher("stop",robot_pb.no_args_func_)
        #self.resume_pub = ProtoPublisher("resume",robot_pb.no_args_func_)

        self.debug_pub =StringPublisher("debug_msg")
        time.sleep(1)

        self.nav.initialisation()
        self.initActionneur()


    def __repr__(self) -> str:
        return "Cooking Mama's status storage structure"


    @staticmethod
    def normalize(angle):
        while angle >= pi:
            angle-=2*pi
        while angle < -pi:
            angle += 2*pi
        return angle
    




    def hasReachedTarget(self):
        d=sqrt((self.pos.x-self.last_target.x)**2 + (self.pos.y-self.last_target.y)**2)
        return (d <= XY_ACCURACY) and (abs(self.pos.theta - self.last_target.theta) <= THETA_ACCURACY)
    

    def setTargetPos(self, pos: Pos, frame=Frame.TABLE):
        """Faire setTargetPos(Pos(x,y,theta)) en mm et angle? """

        
        if frame == Frame.ROBOTCENTRIC:
            pos += self.pos
        elif frame == Frame.ROBOT:
            pos = pos.from_frame(self.pos)
        pos.theta = Robot.normalize(pos.theta)
        pb_pos = pos.to_proto()
        #print(f"go to: {pos}")
        self.set_target_pos_pub.send(pb_pos)
        self.last_target = pos
    
    def move(self, distance, direction):
        frame_pince = Pos(0, 0, direction)
        target = Pos(distance, 0, -direction).from_frame(frame_pince)
        self.setTargetPos(target, Frame.ROBOT)
    
    def move_rel(self,x,y):
        if x : 
            self.move(sqrt(x**2+y**2),atan2(y,x))
        else :
            self.move(y,pi/2*np.sign(y))
    
    def heading(self,angle):
        """ Angle en degré """ 
        self.setTargetPos(Pos(self.pos.x,self.pos.y,angle * pi/180))
    
    def rotate(self,angle):
        """ angle en degré """
        self.heading(self.pos.theta*180/pi + angle)
    
    def resetPos(self, pos: Pos):
        print(f"Pos reseted to : {pos.x},\t{pos.y}, \t{pos.theta} ")
        self.reset_pos_pub.send(pos.to_proto())
    
    #def updateScore(self):
    #    self.score_pub.send(robot_pb.Match(score=self.pointsEstimes))

    def onSetTargetPostition (self, topic_name, msg, timestamp):
        """Callback d'un subscriber ecal. Actualise le dernier ordre de position"""
        self.last_target = Pos.from_proto(msg)

    def onReceivePosition (self, topic_name, msg, timestamp):
        """Callback d'un subscriber ecal. Actualise la position du robot"""
        self.pos = Pos.from_proto(msg)

    def onReceiveSpeed(self, topic_name, msg, timestamp):
        """Callback d'un subscriber ecal. Actualise la vitesse du robot"""
        self.speed = Speed.from_proto(msg)
    
    
    def onReceiveMatchStarted (self, topic_name, msg, timestamp):
        self.tempsDebutMatch = time.time()
        ecal_core.log_message("Match started at " + str(self.tempsDebutMatch))


### NAVIGATION ###
    def initNav(self):
        """ Initialise la navigation """
        self.nav.initialisation()

    def goToWaypoint(self,waypoint, theta: None | float = None ):
        """ Le robot va directement à un waypoint """
        if theta is None :
            theta = self.pos.theta
            print(theta*180/pi)
        x,y = self.nav.getCoords(waypoint)
        self.setTargetPos(Pos(x,y,theta))
        #closest = self.nav.closestWaypoint(self.pos.x,self.pos.y)
        #self.pathFinder(closest,waypoint)

    def resetPosFromNav(self,waypoint):
        x,y = self.nav.getCoords(waypoint)
        self.resetPos(Pos(x,y,self.pos.theta))

    def pathFinder(self,dest):
        """Recherche le plus court chemin entre deux points. 
        \nRetenu dans l'object self.nav.chemin
        \nUtiliser les noms des waypoints de graph.txt"""

        self.nav.entree = self.nav.closestWaypoint(self.pos.x,self.pos.y)
        self.nav.sortie = dest
        self.nav.findPath()

        self.n_points = len(self.nav.chemin)
        self.current_point_index = 0
        self.nav.current = self.nav.chemin[self.current_point_index]
        print(self.nav.chemin)
    
    def followNav(self):
        """ Fait suivre au robot le chemin en mémoire de la nav
         \nIl faut l'appler en boucle pour qu'il passe d'un point à un autre
         \nJe pense qu'on peut faire mieux !"""
        # !!! en milimètres !!!

        if len(self.nav.chemin) != 0 : 
            self.nav.current = self.nav.chemin[self.current_point_index]
            self.goToWaypoint(self.nav.current)

            if self.hasReachedTarget():
                #print(f"Following path, now at {self.nav.current}")
                self.current_point_index += 1
                if self.isNavDestReached():
                    print(" Destination Reached !")
                    self.nav.resetPath()

    def isNavDestReached(self):
        """Si le dernier point de Nav est atteint renvoie True"""
        return self.current_point_index == self.n_points

    ### Actionneur ###
    def setActionneur(self, actionneur: Actionneur,val : ValeurActionneur | int):
        """ Définir en externe les valeurs à prendre 
        \nFaire self.setActionneur(Actionneur.AxL,valeur) pour piloter l'ax de gauche !"""
        if type(val) == int :
            msg = robot_pb.IO(id = actionneur.value , val = val)    
        else :
            msg = robot_pb.IO(id = actionneur.value , val = val.value)
        
        self.IO_pub.send(msg)

    def initActionneur(self):
        time.sleep(0.1)
        self.setActionneur(Actionneur.Pince1,ValeurActionneur.InitPince1)
        time.sleep(0.1)
        self.setActionneur(Actionneur.Pince2,ValeurActionneur.InitPince2)
        time.sleep(0.1)
        self.setActionneur(Actionneur.Pince3,ValeurActionneur.InitPince3)
        time.sleep(0.1)
        self.setActionneur(Actionneur.Pince4,ValeurActionneur.InitPince4)
        time.sleep(0.1)
        self.setActionneur(Actionneur.Bras,ValeurActionneur.InitBras)
        time.sleep(0.1)
        self.setActionneur(Actionneur.Pano,ValeurActionneur.InitPano)
        time.sleep(0.1)
        self.setActionneur(Actionneur.AxL,ValeurActionneur.InitAxL)
        time.sleep(0.1)
        self.setActionneur(Actionneur.AxR,ValeurActionneur.InitAxR)
        time.sleep(0.1)

    def aruco(self, topic_name, msg, timestamp):
        # print(msg)
        self.aruco_theta = msg.theta + self.pano_angle
        self.aruco_y = msg.x - cos(np.deg2rad(self.aruco_theta)) * 15
        self.aruco_x = -(msg.z - PANO_OFFSET) - sin(np.deg2rad(self.aruco_theta)) * 15
        

        commande_pano = self.aruco_theta


        #print(f"aruco cmd : x = {self.aruco_x}\t y = {self.aruco_y}")
        if commande_pano > 180 : 
            commande_pano  = commande_pano - 360

        if commande_pano < -180 : 
           commande_pano  = commande_pano + 360

        self.commande_pano = commande_pano*PANO_CONVERT + ValeurActionneur.InitPano.value
    
    def panoDo(self,commande):
        time.sleep(1)
        self.setActionneur(Actionneur.Bras,ValeurActionneur.DownBras)
        time.sleep(1)
        self.setActionneur(Actionneur.Pano,int(commande))
        print("commande: ",commande)
        time.sleep(2)
        self.setActionneur(Actionneur.Bras,ValeurActionneur.UpBras)
        time.sleep(0.5)
        self.setActionneur(Actionneur.Pano,ValeurActionneur.InitPano)
        time.sleep(2)

if __name__ == "__main__":
    r = Robot()
    while(True):
        print(r.pos)
        time.sleep(0.5)
        
