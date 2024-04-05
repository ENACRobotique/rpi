import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher, StringPublisher
from ecal.core.subscriber import ProtoSubscriber, StringSubscriber
from time import time, sleep
from math import sqrt, pi
import sys
import generated.robot_state_pb2 as robot_pb

import nav

LIDAR_XY_ACCURACY = 0.2
LIDAR_THETA_ACCURACY = pi/4

class Robot:
    """Classe dont le but est de se subscribe à ecal pour avoir une représentation de l'état du robot
    
    Créez un objet de cette classe si vous avez besoin de connaître l'état du robot."""
    def __init__(self):
        """x et y sont en mètres
        theta est en radian"""
        ecal_core.initialize(sys.argv, "robotStateHolder")

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.Vx =0.0
        self.Vy =0.0
        self.Vtheta=0.0

        self.lastTargetX = 0.0
        self.lastTargetY = 0.0
        self.lastTargetTheta = 0.0

        self.nav = nav.Nav()
        
        self.XY_ACCURACY = 0.02  #m
        self.THETA_ACCURACY = 0.05 # radians

        ## subs
        self.positionReportSub = ProtoSubscriber("odom_pos",robot_pb.Position)
        self.positionReportSub.set_callback(self.onReceivePosition)

        self.speedReportSub = ProtoSubscriber("odom_speed",robot_pb.Speed)
        self.speedReportSub.set_callback(self.onReceiveSpeed)

        self.setPositionSub = ProtoSubscriber("set_position", robot_pb.Position)
        self.setPositionSub.set_callback(self.onSetTargetPostition)

        ## pubs
        self.set_target_pos_pub = ProtoPublisher("set_position", robot_pb.Position)
        self.reset_pos_pub = ProtoPublisher("reset", robot_pb.Position)
        

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
        d=sqrt((self.x-self.lastTargetX)**2 + (self.y-self.lastTargetY)**2)
        return (d <= self.XY_ACCURACY) and (abs(self.theta - self.lastTargetTheta) <= self.THETA_ACCURACY)
    

    def setTargetPos(self, x, y, theta):
        """ Envoie la target de position sur le topic 'set_position' 
        \nUnité en mètres !!!"""
        pos = robot_pb.Position(x=x, y=y, theta=theta)
        self.set_target_pos_pub.send(pos)
        self.lastTargetX = x
        self.lastTargetY = y
        self.lastTargetTheta = theta

    def resetPos(self, x, y, theta):
        """ Donne au bas niveau sa position sur le topic 'reset' 
        \nUnité en mètres !!!"""
        self.reset_pos_pub.send(robot_pb.Position(x=x, y=y, theta=theta))

    def onSetTargetPostition (self, topic_name, msg, timestamp):
        """Callback d'un subscriber ecal. Actualise le dernier ordre de position"""
        self.lastTargetX = msg.x
        self.lastTargetY = msg.y
        self.lastTargetTheta = msg.theta

    def onReceivePosition (self, topic_name, msg, timestamp):
        """Callback d'un subscriber ecal. Actualise la position du robot"""
        self.x=msg.x
        self.y=msg.y
        self.theta=msg.theta
        # TODO : publish side deduced from position & if match hasn't started
        # self.pubSide.send(robot_pb.Side(side=robot_pb.Side.BLUE))
        # self.pubSide.send(robot_pb.Side(side=robot_pb.Side.GREEN))

    def onReceiveSpeed(self, topic_name, msg, timestamp):
        """Callback d'un subscriber ecal. Actualise la vitesse du robot"""
        self.Vx = msg.vx
        self.Vy = msg.vy
        self.Vtheta = msg.vtheta    

    ## Nav
    
    def init_nav(self):
        """ Initialise la navigation """
        self.nav.initialisation()

    
    def pathFinder(self,start,end):
        """Recherche le plus court chemin entre deux points. 
        \nRetenu dans l'object self.nav.chemin
        \nUtiliser les noms des waypoints de graph.txt"""

        self.nav.entree = start
        self.nav.sortie = end
        self.nav.findPath()
        self.n_points = len(self.nav.chemin)
        self.current_point = 0
        self.nav.current = self.nav.chemin[self.current_point]
    
    def reset_pos_from_nav(self,waypoint):     
        x,y = self.nav.getCoords(waypoint)
        self.resetPos(x/1000,y/1000,self.theta)



    def followPath(self):
        """ Fait suivre au robot le chemin en mémoire de la nav"""
        x,y = self.nav.current # !!! en milimètres !!!
        print(f"Following path, now at {x,y}")
        if not (self.hasReachedTarget()): # si le robot n'est pas arrivé
            self.setTargetPos(x/1000,y/1000,self.theta)           # continue d'aller au point en cour
            print(f"{x}\t,{y} reached !")
        else:                                               # sinon si le point est atteint
            self.current_point += 1                     # on passe au suivant 
            if self.current_point == self.n_points :        # et si on est arrivé au dernier
                print(" Je t'avais dis je sais conduire ")  # on s'arrette là
                return
            else :                                          # et sinon si on est arrivé au précédent mais pas le dernier
                self.nav.current = self.nav.chemin[self.current_point] 

print("pré-ping")
if __name__ == "__main__":
    print("ping")
    r = Robot()
    while ecal_core.ok():
        print(r.x,r.y,r.theta)