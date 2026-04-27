#!/usr/bin/env python3
import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Publisher as ProtoPublisher
from ecal.msg.proto.core import Subscriber as ProtoSubscriber
from ecal.msg.common.core import ReceiveCallbackData
from ecal.msg.string.core import Publisher as StringPublisher
import time
from math import sqrt, atan2, radians
import sys
import logging
import generated.common_pb2 as common_pb
import generated.robot_state_pb2 as robot_pb
import generated.lidar_data_pb2 as lidar_pb
import generated.messages_pb2 as base_pb
import IO.actionneurs as act
from common import Pos, Speed, next_path, normalize_angle
from camera.arucoState import ArucoState
from queue import Queue, Empty

from scipy.stats import linregress

from enum import Enum
import numpy as np
import sw.nav.nav as nav 


HEIGHT = 2000
WIDTH = 3000
DELTA = 40
XY_ACCURACY = 20  # mm
THETA_ACCURACY = radians(10) # radians
#AVOIDANCE_OBSTACLE_MARGIN = 500 #in mm.  Standard robot enemy radius is 22 cm


# avoidance bounds
BOUNDS = (-250,500,-250,250)
ROTATE_BOUNDS = (-250,250,-250,250) # bounds for rotation
#BOUNDS = (-400,400,-400,400)
TABLE_BOUNDS = (0,3000,0,2000)
#timing for actionneur movements
ACT_TIME = 0.5 # seconds

class Team(Enum):
    BLEU = 1
    JAUNE = 2
    AUCUNE = 10

class Tirette(Enum):
    IN = 1
    OUT = 0

class Frame(Enum):
    TABLE = 0
    ROBOT = 2

class Strat(Enum):
    Basique = 0
    Demo = 1
    Homologation = 2
    Audacieuse = 3
    ShowOff = 4

class Caisse(Enum):
    BLEU = 36
    JAUNE = 47
    RIEN = 0
    TOUT = 999

class Velocity(Enum):
    FAST = Speed(600,0,4)
    NORMAL = Speed(300,0,2)
    SLOW = Speed(100,0,1)

class Cote(Enum):
    DROIT = True
    GAUCHE = False

COTE_DROIT = True
COTE_GAUCHE = False

class Robot:
    """Classe dont le but est de se subscribe à ecal pour avoir une représentation de l'état du robot
    
    Créez un objet de cette classe si vous avez besoin de connaître l'état du robot."""
    def __init__(self, name="robotStateHolder"):
        """x et y sont en mètres
        theta est en radian"""
        if not ecal_core.is_initialized():
            ecal_core.initialize(name)
        self.logger = logging.getLogger(name)
        logging.basicConfig(filename=next_path("/home/robot/logs/strat_log_{}.log"), level=logging.INFO)
        logging.getLogger().addHandler(logging.StreamHandler(sys.stdout))

        self.pos = Pos(0, 0, 0)
        self.last_d = 20 # ne pas mettre trop grand
        self.nb_pos_received = 0
        self.speed = Speed(0, 0, 0)
        self.last_target = Pos(0, 0, 0)
        self.nav = nav.Nav() #Nav
        self.command_sent = False
        
        self.color = Team.AUCUNE
        self.tirette = Tirette.OUT
        self.strat = Strat.Basique
        self.score = 0
        self.obstacles = [] # obstacles in table frame

        self._pid_gains = [0, 0, 0]     # Just for manual setting of PIDS

        self.actionneurs = act.IO_Manager()
        self.coteD = [Caisse.RIEN,Caisse.RIEN,Caisse.RIEN,Caisse.RIEN]
        self.coteG = [Caisse.RIEN,Caisse.RIEN,Caisse.RIEN,Caisse.RIEN]

        self.aruco_state = ArucoState(3)

        ### SUB ECAL ###

        self.positionReportSub = ProtoSubscriber(common_pb.Position, "odom_pos")
        self.positionReportSub.set_receive_callback(self.onReceivePosition)

        self.tiretteSub = ProtoSubscriber(robot_pb.Tirette, "tirette")
        self.tiretteSub.set_receive_callback(self.onReceiveTirette)

        self.speedReportSub = ProtoSubscriber(common_pb.Speed, "odom_speed")
        self.speedReportSub.set_receive_callback(self.onReceiveSpeed)

        self.amalgames_sub = ProtoSubscriber(lidar_pb.Amalgames, "amalgames")
        self.amalgames_sub.set_receive_callback(self.detection)

        self.lidarPosSub = ProtoSubscriber(common_pb.Position, "lidar_pos")
        self.lidarPosSub.set_receive_callback(self.onLidarPos)

        self.colorSub = ProtoSubscriber(robot_pb.Side, "color")
        self.colorSub.set_receive_callback(self.onColorChanged)

        # When Using Robokontrol
        self.setPositionSub = ProtoSubscriber(common_pb.Position, "set_position")
        self.setPositionSub.set_receive_callback(self.onSetTargetPostition)

        self.response_queue = Queue()
        self.response_sub = ProtoSubscriber(base_pb.Response, "response")
        self.response_sub.set_receive_callback(self.onResponse)
        

        #self.proximitySub = ProtoSubscriber(lidar_pb.Proximity, "proximity_status")
        #self.proximitySub.set_receive_callback(self.onProximityStatus)

        ### PUB ECAL ###
        self.reset_pos_pub = ProtoPublisher(common_pb.Position, "reset")

        self.speed_cons_pub = ProtoPublisher(common_pb.Speed, "speed_cons")

        self.target_pos_pub = ProtoPublisher(common_pb.Position, "set_position")

        self.target_relativ_pos_pub = ProtoPublisher(common_pb.Position, "set_relativ_pos")

        # self.IO_pub = ProtoPublisher("Actionneur",robot_pb.IO)

        self.color_pub = ProtoPublisher(robot_pb.Side, "color")

        self.pid_pub = ProtoPublisher(base_pb.MotorPid, "pid_gains")


        self.logs_pub = StringPublisher("logs")
        self.objects_pubs = [ProtoPublisher(common_pb.Position, f"Obstacle{i}") for i in range(3)]
        time.sleep(1)

        self.nav.initialisation()
        self.folowingPath = False
        self.actionneurs.initActionneur()

    def __repr__(self) -> str:
        return "Robot Enac status storage structure"
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        self.positionReportSub.remove_receive_callback()
        self.speedReportSub.remove_receive_callback()
        self.amalgames_sub.remove_receive_callback()
        self.setPositionSub.remove_receive_callback()

    def log(self, message:str):
        self.logger.info(message)
        self.logs_pub.send(message)

        
# ---------------------------- #
#             IHM              #
# ____________________________ #

    # def set_color(self, c):
    #     self.color = c
    #     msg = robot_pb.Side()
    #     if self.color == Team.JAUNE:
    #         msg.color = robot_pb.Side.Color.YELLOW
    #     else:
    #         msg.color = robot_pb.Side.Color.BLUE
    #     self.logger.info(f"Equipe : {c}")
    #     self.color_pub.send(msg)


    def updateScore(self,points):
        # TODO
        self.score += points

    
 
    # def set_strat(self, strat):
    #     self.strat = strat

    
    def ready_to_go(self):
        """ Rassembler les conditions nécéssaires pour que le robot commence son match"""
        a = self.color != Team.AUCUNE
        b = self.tirette == Tirette.OUT
        return a and b
# ---------------------------- #
#           CONTROL            #
# ____________________________ #

    def stop(self):
        self.set_speed(Speed(0,0,0))
        self.actionneurs.stopActionneur()
        

    def hasReachedTarget(self):
        if self.pos.distance(self.last_target) < 15 and (abs(self.pos.theta - self.last_target.theta) < np.deg2rad(3)):
            return True
        else:
            return False

    def setTargetPos(self, pos: Pos, frame=Frame.TABLE,blocking=False, timeout = 10):
        """Faire setTargetPos(Pos(x,y,theta)) en mm et angle en radian """
        if frame == Frame.ROBOT:
            pos = pos.from_frame(self.pos)
        pos.theta = normalize_angle(pos.theta)
        self.target_pos_pub.send(pos.to_proto())
        self.last_target = pos
        
        if blocking :
            try:
                success = self.response_queue.get(timeout=timeout)
                return success == 0
            except Empty:
                return False

    def move(self, distance, direction, blocking=False, timeout = 10):
        """
        BLOQUANT\n
        avance de distance dans la direction direction, repère robot 
        """
        target = Pos(distance, 0, normalize_angle(direction)) 
        self.target_relativ_pos_pub.send(target.to_proto())
        if blocking :
            success = self.response_queue.get(timeout=timeout)
            return success == 0





    
    def heading(self,angle,blocking=False, timeout = 10):
        """ S'oriente vers la direction donnée
         \nArgs float:theta en radian""" 
        return self.setTargetPos(Pos(self.pos.x,self.pos.y,angle),Frame.TABLE, blocking, timeout)
    
    def rotate(self,angle,blocking=False, timeout = 10):
        """ Rotation en relatif
         \nArgs, float:theta en radians """
        self.heading(self.pos.theta + angle,blocking=False, timeout = 10)

    def set_speed(self, speed: Speed):
        """
        Set robot speed in robot reference frame.
        Useful e.g. to move until a sensor triggers
        """
        self.speed_cons_pub.send(speed.to_proto())
    
    def resetPos(self, position: Pos, timeout=2):
        self.log(f"Reseting position to: {position} ")
        self.reset_pos_pub.send(position.to_proto())
        start_time = time.time()
        last_time = time.time()
        while time.time()-start_time < timeout:
            if self.pos.distance(position) < 1 and abs(self.pos.theta - position.theta) < radians(1):
                self.log(f"Pos reseted to : {position.x},\t{position.y}, \t{position.theta} ")
                break
            if time.time() - last_time > 1:
                self.log("reset_again")
                self.reset_pos_pub.send(position.to_proto())
                last_time = time.time()
            time.sleep(0.1)

    def resetPosOnLidar(self):
        """Recalage sur la pos du lidar"""
        self.resetPos(self.lidar_pos)
    
    def onSetTargetPostition (self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[common_pb.Position]):
        """Callback d'un subscriber ecal. Actualise le dernier ordre de position"""
        self.last_target = Pos.from_proto(data.message)

    def onReceivePosition (self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[common_pb.Position]):
        """Callback d'un subscriber ecal. Actualise la position du robot"""
        msg = data.message
        self.pos = Pos.from_proto(msg)
        self.nb_pos_received += 1

    def onColorChanged (self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[robot_pb.Side]):
        msg = data.message
        if msg.color == robot_pb.Side.Color.YELLOW :
            self.color = Team.JAUNE
        else:
            self.color = Team.BLEU

    def onLidarPos(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[common_pb.Position]):
        """Callback d'un subscriber ecal. Récup la position du lidar"""
        msg = data.message
        self.lidar_pos = Pos.from_proto(msg)

    def onReceiveSpeed(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[common_pb.Speed]):
        """Callback d'un subscriber ecal. Actualise la vitesse du robot"""
        self.speed = Speed.from_proto(data.message)
    
    def onReceiveTirette(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[robot_pb.Tirette]):
        """Callback d'un subscriber ecal. Actualise la tirette du robot"""
        if data.message.tirette_state == robot_pb.Tirette.IN :
            self.tirette = Tirette.IN
        else:
            self.tirette = Tirette.OUT

    def onResponse(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[base_pb.Response]):
        self.response_queue.put(data.message.status)
        

    def set_pid_gain(self, gain, value):
        self._pid_gains[gain] = value
        kp, ki, kd = self._pid_gains
        msg = base_pb.MotorPid(motor_no=0, kp=kp, ki=ki, kd=kd)
        self.pid_pub.send(msg)

# ---------------------------- #
#          NAVIGATION          #
# ____________________________ #   

    def initNav(self):
        """ Initialise la navigation """
        self.nav.initialisation()

    def goToWaypoint(self,waypoint, theta: None | float = None ):
        """ Le robot va directement à un waypoint avec ou sans angle donné
        \nArgs, string:waypoint, float|None:theta"""
        if theta is None :
            theta = self.pos.theta
            #self.log(theta*180/pi)
        x,y = self.nav.getCoords(waypoint)
        return self.setTargetPos(Pos(x,y,theta))
        #closest = self.nav.closestWaypoint(self.pos.x,self.pos.y)
        #self.pathFinder(closest,waypoint)
    
    def distance_from(self,waypoint):
        x,y = self.nav.getCoords(waypoint)
        return self.pos.distance(Pos(x,y,0))

    def resetPosFromNav(self, waypoint, theta=None):
        self.log("Reseted nav at : {waypoint}")
        if theta is None:
            theta = self.pos.theta
        x,y = self.nav.getCoords(waypoint)
        self.resetPos(Pos(x, y, theta))

    def pathFinder(self,dest,orientation):
        """Recherche le plus court chemin entre deux points.
        \nRetenu dans l'object self.nav.chemin
        \nUtiliser les noms des waypoints de graph.txt"""

        self.nav.entree = self.nav.closestWaypoint(self.pos.x,self.pos.y)
        self.nav.sortie = dest
        nav_pos = self.nav.findReducedPath(self.pos.theta,orientation)

        self.n_points = len(self.nav.chemin)
        self.log(f"entree: {self.nav.entree}")
        self.current_point_index = 0
        self.nav.current = self.nav.chemin[self.current_point_index]
        self.log(f"Path found : {self.nav.chemin}")
        self.nav_pos = [Pos(p[0],p[1],p[2]) for p in nav_pos]
        #self.log("Pos's are : ",self.nav_pos)
        self.folowingPath = True

    def dest_to_pos(self,tuple_dest_ang):
        x,y = self.nav.getCoords(tuple_dest_ang[0])
        return Pos(x,y,tuple_dest_ang[1])


    def closeToNavPoint(self, nav_id):
        d=sqrt((self.pos.x-self.nav_pos[nav_id].x)**2 + (self.pos.y-self.nav_pos[nav_id].y)**2)
        return (d <= XY_ACCURACY)
    
    def isNavDestReached(self):
        """Si le dernier point de Nav est atteint renvoie True\n
        Nécéssite de vider continuement la liste des points de nav !"""
        end = self.closeToNavPoint(-1) and self.hasReachedTarget()
        if end:
            self.folowingPath = False
        return end 
        
    def detection(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[lidar_pb.Amalgames]):
        msg = data.message
        """ Try to find ennemies 
        \nSend 3 detected object Pos on ecal to visualize but saves all of them """
        def filter_pos(pos_size):
            
            pos, size = pos_size
            if 0+DELTA < pos.x < WIDTH-DELTA and  0+DELTA < pos.y < HEIGHT-DELTA :# exclude object out of the table
                if self.pos.distance(pos) < 1500 and size < 30 : # exclude close and tiny object ( test in conditions !)
                    return False
                if self.pos.distance(pos) < 150: # exclude object mixed up with the robot
                    return False
                return True
            return False

        amalgames = [(Pos(x,y,0).from_frame(self.pos), size) for x,y,size in zip(msg.x,msg.y,msg.size)]
        
        self.obstacles = list(filter(filter_pos,amalgames))
        for i,ob in enumerate(self.obstacles) :
            if i < 3:
                self.objects_pubs[i].send(ob[0].to_proto())

    def obstacle_in_way(self, target_pos: Pos):
        """Return True if obstacles in bounds towards target_pos"""
        if self.pos.distance(target_pos) < 1:
            return False
        dir = atan2(target_pos.y-self.pos.y,target_pos.x-self.pos.x)
        traj_frame = Pos(self.pos.x, self.pos.y, dir)

        for obj_pos, _size in self.obstacles:
            obj = obj_pos.to_frame(traj_frame)
            
            x_min, x_max, y_min, y_max = BOUNDS
            if x_min < obj.x < x_max and  y_min < obj.y < y_max :
                return True
            
        return False
    
    def obstacle_in_speed(self, speed: Speed):
        """Return True if obstacles in bounds towards speed direction"""
        speed_norm = speed.xy_norm()
        if speed_norm > 1:
            bounds = BOUNDS
            #if speed_norm > 300: # if speed is too high, we adapt the bounds
            #    x_min, x_max, y_min, y_max = bounds
            #    coef = speed_norm / 300
            #    bounds = (x_min, x_max*coef, y_min, y_max)
        else:
            bounds = ROTATE_BOUNDS
        dir = atan2(speed.vy, speed.vx)
        traj_frame = Pos(self.pos.x, self.pos.y, dir)

        for obj_pos, _size in self.obstacles:
            obj = obj_pos.to_frame(traj_frame)
            x_min, x_max, y_min, y_max = bounds
            if x_min < obj.x < x_max and  y_min < obj.y < y_max :
                return True
        return False
    
    def align_with_pack(self,coteDroit,timeout=0):
        time_deb = time.time()
        arucosPosRobot = self.aruco_state.get_aruco_robot()
        while (len(arucosPosRobot)!=4 and (time.time()-time_deb)<timeout):
            arucosPosRobot = self.aruco_state.get_aruco_robot()
               
        # print(arucosPosRobot[2].pos,type(arucosPosRobot[2].pos))
        if len(arucosPosRobot)==4:

            print("Bip bop alignement")

            x_centerPack,y_centerPack = 0,0
            for aruco in arucosPosRobot:
                x_centerPack += aruco.pos[0]/4
                y_centerPack += aruco.pos[1]/4
            
            # Hypothèse : on est globalement dans le bon sens à peut de choses près...
            x_droite = (min([aruco.pos for aruco in arucosPosRobot], key=lambda elt: elt[0]), max([aruco.pos for aruco in arucosPosRobot], key=lambda elt: elt[0]))
            angle_droite_robot = np.atan2(x_droite[1][1]-x_droite[0][1],x_droite[1][0]-x_droite[0][0])

            ## Etre parralléle
            x_repereCaisse = x_centerPack * np.cos(angle_droite_robot) + y_centerPack * np.sin(angle_droite_robot)
            y_repereCaisse = y_centerPack * np.cos(angle_droite_robot) - x_centerPack * np.sin(angle_droite_robot)

            ## Alignement en y :
            if (abs(y_repereCaisse) > 300) or (abs(y_repereCaisse) < 130) :
                print("Mauvais dy")

            self.rotate(angle_droite_robot,blocking=True)
        
            ## Alignement en x: 
            #print(-x_repereCaisse+200)
            #self.move(200 - x_repereCaisse,0,blocking=True,timeout=2)
            print(x_repereCaisse)
            self.move(x_repereCaisse,0,blocking=True,timeout=2)
            if coteDroit :
                self.coteD = [Caisse.BLEU if aruco.id == Caisse.BLEU.value else Caisse.JAUNE for aruco in sorted(arucosPosRobot,key = lambda aruco : aruco.pos[0])]
            else :
                self.coteG = [Caisse.BLEU if aruco.id == Caisse.BLEU.value else Caisse.JAUNE for aruco in sorted(arucosPosRobot,key = lambda aruco : aruco.pos[0])]

    def cote_droit_vide(self):
        for caisse in self.coteD:
            if caisse != Caisse.RIEN:
                return False
        return True
    
    def cote_gauche_vide(self):
        for caisse in self.coteG:
            if caisse != Caisse.RIEN:
                return False
        return True

    def cote_droit_ours(self):
        ### True si le cote droit a des caisse de notre couleur
        ### False sinon
        for caisse in self.coteD:
            if (caisse == Caisse.BLEU and self.color == Team.BLEU) or (caisse == Caisse.JAUNE and self.color == Team.JAUNE):
                return False
        return True

    def cote_gauche_ours(self):
        ### True si le cote gauche a des caisse de notre couleur
        ### False sinon
        for caisse in self.coteG:
            if (caisse == Caisse.BLEU and self.color == Team.BLEU) or (caisse == Caisse.JAUNE and self.color == Team.JAUNE):
                return True
        return False
    
    def brasThermo(self):
        self.actionneurs.moveTricepsD(act.PosTentacle.THERMO)

    def thermoAct(self,thermo_pos):
        self.setTargetPos(self.dest_to_pos(thermo_pos),blocking=True,timeout=8)
        self.move(100,thermo_pos[1] + np.pi,blocking=True,timeout=2) # ie on bourre le mur
        if self.color ==Team.JAUNE:
            self.actionneurs.moveTricepsD(act.PosTentacle.THERMO)
        else :
            self.actionneurs.moveTricepsG(act.PosTentacle.THERMO)
        self.move(500,0,blocking=True,timeout=8)
        if self.color ==Team.JAUNE:
            self.actionneurs.moveTricepsD(act.PosTentacle.HAUT)
        else :
            self.actionneurs.moveTricepsG(act.PosTentacle.HAUT)
        return True
    
    def attraper(self,coteDroit):
        if coteDroit :
            self.actionneurs.moveD(act.PosTentacle.BAS)
            self.actionneurs.GrabD(True)
            time.sleep(1)
            self.actionneurs.moveD(act.PosTentacle.HAUT)
        else :
            self.actionneurs.moveG(act.PosTentacle.BAS)
            self.actionneurs.GrabG(True)
            time.sleep(1)
            self.actionneurs.moveG(act.PosTentacle.HAUT)
    
    def relacher(self,coteDroit,couleur:Caisse):
        print("RELEASE : cote droit =", self.coteD,"cote gauche =",self.coteG)
        if coteDroit :
            self.actionneurs.moveD(act.PosTentacle.DROP)
            for (i,caisse) in enumerate(self.coteD) :
                if caisse == couleur or couleur == Caisse.TOUT :
                    self.actionneurs.Grab(act.POMPES_DROITES[i],False)
                    self.coteD[i] = Caisse.RIEN
            time.sleep(2)
            self.actionneurs.moveD(act.PosTentacle.HAUT)
        else :
            self.actionneurs.moveG(act.PosTentacle.DROP)
            for (i,caisse) in enumerate(self.coteG):
                if caisse == couleur or couleur == Caisse.TOUT :
                    self.actionneurs.Grab(act.POMPES_GAUCHES[i],False)
                    self.coteG[i] = Caisse.RIEN
            time.sleep(2)
            self.actionneurs.moveG(act.PosTentacle.HAUT)
        return

if __name__ == "__main__":
    with Robot() as r:
        while(True):
            # r.logger.info(r.pos)
            time.sleep(0.5)
        
