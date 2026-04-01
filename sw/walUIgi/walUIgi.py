#!/usr/bin/env python3
import sys
import os
import argparse
import math as math
import time

from PyQt6 import QtCore, QtWidgets
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel,
    QGraphicsView, QGraphicsScene, QGraphicsProxyWidget,
    QFrame,QGraphicsItemGroup,
    QGraphicsEllipseItem, QGraphicsTextItem, QGraphicsLineItem
)
from PyQt6.QtGui import QImage, QPainter, QColor, QPen, QTransform
from PyQt6.QtCore import Qt,pyqtSignal,QObject,QTimer
import generated.robot_state_pb2 as robot_pb

import generated.common_pb2 as hgpb
import generated.lidar_data_pb2 as lidarpb


import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Subscriber as ProtoSubscriber
from ecal.msg.proto.core import Publisher as ProtoPublisher
from ecal.msg.common.core import ReceiveCallbackData

from lidar.radar_view import RadarView
from actuators.sap_master import SAPMaster

########################
# 
#       TABLE DES
#      CONSTANTES
#
#########################

LISTE_SERVICE = ["robot_aruco","robot_aruco2","robot_bridge","robot_IO","robot_joystick","robot_lidar_driver","robot_lidar_amalgameur","robot_lidar_loca","robot_start","robot_strat"]
TOTAL_SERVICE = len(LISTE_SERVICE)

LISTE_ID_ACTIONNEURS = [5,7,11,20,40,41,42,43,44,45,46,47]
TOTAL_ACTIONNEURS = len(LISTE_ID_ACTIONNEURS)

BASEROULANTE_TIMEOUT = 1000 # ms

class Robot:
    def __init__(self):
        self.name = "Dave"
        self.diameter = 300 #mm
        self.couleur = "Jaune"
        self.color_pub = ProtoPublisher(robot_pb.Side, "color")
        self.position_lidar = (0, 0, 0)
        self.position_odom = (0, 0, 0)
        self.last_baseRoulante = False
        self.nbBalises = 0
        self.score = 0

        self.nbServicesActifs=0
        self.nbActionneursActifs = 0

        self.map_width = 3000 #mm
        self.map_height = 2000 #mm
        self.scale =  0.43 # mm -> graphic_size
        self.background = "./table2026.png"


####################

class SignalEmitter(QObject):
    pos_lidar_signal = pyqtSignal(float, float, float)
    pos_odom_signal = pyqtSignal(float, float, float)
    balise_signal = pyqtSignal(int)

    service_signal = pyqtSignal()
    actionneur_signal = pyqtSignal(bool)
    baseRoulante_signal = pyqtSignal(bool)

#########
# Permet de tourner l'interface
#########
class RotatedWindow(QGraphicsView):
    def __init__(self, widget, angle=90):
        scene = QGraphicsScene()
        super().__init__(scene)

        proxy = QGraphicsProxyWidget()
        proxy.setWidget(widget)

        # Centre de rotation
        proxy.setTransformOriginPoint(proxy.boundingRect().center())
        proxy.setRotation(angle)

        scene.addItem(proxy)

        # Désactive scrollbars (PyQt6 enums)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        # Frame enum déplacé dans QFrame
        self.setFrameShape(QFrame.Shape.NoFrame)

        # Taille max disponible à l'écran
        screen_rect = QApplication.primaryScreen().availableGeometry()
        screen_width = screen_rect.width()
        screen_height = screen_rect.height()

        self.resize(screen_width, screen_height)


#############
#           #
#    TAB    #
#   STATUS  #
#           #
#############
class TabStatus(QtWidgets.QWidget):

    def __init__(self,robot:Robot,signal_emitter):
        super().__init__()

        self.robot = robot
        self.signal_emitter = signal_emitter

        layout = QtWidgets.QVBoxLayout(self)

        topLayout = QtWidgets.QHBoxLayout()

        self.bar_batterie = QtWidgets.QProgressBar()
        self.bar_batterie.setMaximumSize(80, 33)
        self.bar_batterie.setValue(24)

        self.title_status = QtWidgets.QLabel("STATUS")
        self.title_status.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)

        topLayout.addWidget(self.bar_batterie)
        topLayout.addWidget(self.title_status)
        topLayout.addStretch()

        layout.addLayout(topLayout)

        self.b_status_color = QtWidgets.QPushButton(self.robot.name)
        self.b_status_color.setMinimumSize(300, 100); self.b_status_color.setStyleSheet("background-color: yellow")
        self.b_status_color.clicked.connect(lambda:self.updateCouleur(False if self.robot.couleur=="Jaune" else True))

        layout.addWidget(self.b_status_color, alignment=QtCore.Qt.AlignmentFlag.AlignHCenter)

        grid = QtWidgets.QGridLayout()

        self.l_baseRoulante = QtWidgets.QLabel("Base roulante"); self.l_baseRoulante.setStyleSheet("color: red")
        self.icon_baseRoulante = QtWidgets.QLabel("X")
        self.signal_emitter.baseRoulante_signal.connect(self.updateBaseRoulante)
        self.baseRoulante_timer = QTimer()
        self.baseRoulante_timer.setInterval(BASEROULANTE_TIMEOUT)
        self.baseRoulante_timer.timeout.connect(self.updateBaseRoulante)

        self.l_status_service = QtWidgets.QLabel(f"{self.robot.nbServicesActifs}/{TOTAL_SERVICE} services"); self.l_status_service.setStyleSheet("color: red")
        self.icon_service = QtWidgets.QLabel("X")
        self.signal_emitter.service_signal.connect(self.updateServices)

        self.l_status_actionneurs = QtWidgets.QLabel(f"{self.robot.nbActionneursActifs}/{TOTAL_ACTIONNEURS} actionneurs"); self.l_status_actionneurs.setStyleSheet("color: red")
        self.icon_actionneur = QtWidgets.QLabel("X")
        self.signal_emitter.actionneur_signal.connect(self.updateActionneurs)

        self.l_nbBalises = QLabel("Nombre balises : No ecal message (yet)"); self.l_nbBalises.setStyleSheet("color: red")
        self.icon_nbBalises = QLabel("X")
        self.balise_sub = ProtoSubscriber(lidarpb.Balises,"balises_odom")
        def sendBaliseSignal( pub_id : ecal_core.TopicId, data : ReceiveCallbackData[lidarpb.Balises]):
            self.signal_emitter.balise_signal.emit(len(data.message.index))
        self.balise_sub.set_receive_callback(sendBaliseSignal)
        self.signal_emitter.balise_signal.connect(self.updateBalise)
        

        grid.addWidget(self.l_baseRoulante, 0, 0)
        grid.addWidget(self.icon_baseRoulante, 0, 1)

        grid.addWidget(self.l_status_service, 1, 0)
        grid.addWidget(self.icon_service, 1, 1)

        grid.addWidget(self.l_status_actionneurs, 2, 0)
        grid.addWidget(self.icon_actionneur, 2, 1)

        grid.addWidget(self.l_nbBalises, 3, 0)
        grid.addWidget(self.icon_nbBalises, 3, 1)

        layout.addLayout(grid)

        class Map_view(QGraphicsView):
            def resizeEvent(self, event):
                if self.scene():
                    self.fitInView(self.scene().sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)
                super().resizeEvent(event)

        class Map(QGraphicsScene):
            def __init__(self,x,y,w,h,background):
                super().__init__(x,y,w,h)
                self.background = QImage(background)

            def drawBackground(self, painter, rect):
                painter.drawImage(self.sceneRect(), self.background)
                
        class RobotGraphic(QGraphicsItemGroup):
            def __init__(self, name, robot, color = "darkblue") -> None:
                super().__init__()
                self.color = color
                self.name = name
                self.robot = robot
                
                body = QGraphicsEllipseItem((- self.robot.diameter/2) * self.robot.scale, - self.robot.diameter/2 * self.robot.scale, self.robot.diameter * self.robot.scale, self.robot.diameter * self.robot.scale)
                body.setBrush(QColor(self.color))
                self.addToGroup(body)

                orientation_pointer = QGraphicsLineItem(0,0,300*self.robot.scale,0)
                orientation_pointer.setPen(
                    QPen(
                        Qt.GlobalColor.black,
                        10 * self.robot.scale,
                        Qt.PenStyle.SolidLine,
                        Qt.PenCapStyle.RoundCap,
                        Qt.PenJoinStyle.RoundJoin
                    ))
                self.addToGroup(orientation_pointer)

            def updatePosition(self,x,y,theta):
                self.x = x
                self.y = y
                self.theta = theta

                ## Affichage graphique
                super().setPos(x * self.robot.scale ,(self.robot.map_height - y)* self.robot.scale)
                super().setRotation(math.degrees(-theta))

        map = Map(0,0,self.robot.map_width * self.robot.scale, self.robot.map_height * self.robot.scale,self.robot.background)

        self.point_odom = RobotGraphic("odom",self.robot,"red")
        self.pos_odom_sub = ProtoSubscriber(hgpb.Position,"odom_pos")
        def sendPosOdomSignal( pub_id : ecal_core.TopicId, data : ReceiveCallbackData[hgpb.Position]):
            self.robot.last_baseRoulante = True
            self.signal_emitter.pos_odom_signal.emit(data.message.x, data.message.y, data.message.theta)
        self.pos_odom_sub.set_receive_callback(sendPosOdomSignal)
        self.signal_emitter.pos_odom_signal.connect(self.point_odom.updatePosition)

        self.point_lidar = RobotGraphic("lidar",self.robot,"darkblue")
        self.pos_lidar_sub = ProtoSubscriber(hgpb.Position,"lidar_pos")
        def sendPosLidarSignal( pub_id : ecal_core.TopicId, data : ReceiveCallbackData[hgpb.Position]):
            self.signal_emitter.pos_lidar_signal.emit(data.message.x, data.message.y, data.message.theta)
        self.pos_lidar_sub.set_receive_callback(sendPosLidarSignal)
        self.signal_emitter.pos_lidar_signal.connect(self.point_lidar.updatePosition)
            
        map.addItem(self.point_odom)
        map.addItem(self.point_lidar)

        view_map = Map_view(map)

        bottomLayout = QtWidgets.QHBoxLayout()

        bottomLayout.addWidget(view_map)
        layout.addLayout(bottomLayout)

        self.baseRoulante_timer.start()
        

    def updateCouleur(self, jaune):
        msg = robot_pb.Side()
        if jaune:
            self.robot.couleur = "Jaune"
            self.b_status_color.setStyleSheet("background-color: yellow")
            msg.color = robot_pb.Side.Color.YELLOW
        else:
            self.robot.couleur = "Bleu"
            self.b_status_color.setStyleSheet("background-color: blue")
            msg.color = robot_pb.Side.Color.BLUE
        self.robot.color_pub.send(msg)

    def updateBalise(self,nb):
        self.robot.nbBalises = nb
        self.l_nbBalises.setText("Nombre balises : " + str(nb))
        if nb >= 3 :
            self.l_nbBalises.setStyleSheet("color: red")
            self.icon_nbBalises.setText("V")
        else :
            self.l_nbBalises.setStyleSheet("color: green")
            self.icon_nbBalises.setText("X")

    def updateServices(self):
        if self.robot.nbServicesActifs == TOTAL_SERVICE:
            self.l_status_service.setText(f"{self.robot.nbServicesActifs}/{TOTAL_SERVICE} services"); self.l_status_service.setStyleSheet("color: green")
            self.icon_service.setText("V")
        else:
            self.l_status_service.setText(f"{self.robot.nbServicesActifs}/{TOTAL_SERVICE} services"); self.l_status_service.setStyleSheet("color: red")
            self.icon_service.setText("X")

    def updateActionneurs(self,serviceIO):
        if serviceIO :
            if self.robot.nbActionneursActifs == TOTAL_ACTIONNEURS:
                self.l_status_actionneurs.setText(f"{self.robot.nbActionneursActifs}/{TOTAL_ACTIONNEURS} actionneurs"); self.l_status_actionneurs.setStyleSheet("color: green")
                self.icon_actionneur.setText("V")
            else:
                self.l_status_actionneurs.setText(f"{self.robot.nbActionneursActifs}/{TOTAL_ACTIONNEURS} actionneurs"); self.l_status_actionneurs.setStyleSheet("color: red")
                self.icon_actionneur.setText("X")
        else:
            self.l_status_actionneurs.setText(f"{self.robot.nbActionneursActifs}/{TOTAL_ACTIONNEURS} actionneurs"); self.l_status_actionneurs.setStyleSheet("color: gray")
            self.icon_actionneur.setText("(No service robot_IO)")

    def updateBaseRoulante(self):
        if self.robot.last_baseRoulante :
            self.l_baseRoulante.setStyleSheet("color: green")
            self.icon_baseRoulante.setText("V")
        else:
            self.l_baseRoulante.setStyleSheet("color: red")
            self.icon_baseRoulante.setText("X")
        self.robot.last_baseRoulante = False




#############
#           #
#    TAB    #
#  SERVICE  #
#           #
#############

class ServiceWidget(QWidget):

    def __init__(self, service, parent: QWidget = None) -> None:
        super().__init__(parent)
        self.service = service
        self.hl = QHBoxLayout(self)
        self.l_service = QtWidgets.QLabel(f"{service}")
        self.b_stop_service = QPushButton("Stop"); self.b_stop_service.setMaximumSize(80, 35)
        self.b_stop_service.clicked.connect(self.stop)
        self.b_start_service = QPushButton("Start"); self.b_start_service.setMaximumSize(80, 35)
        self.b_start_service.clicked.connect(self.start)
        self.l_ok_service = QLabel("V")

        self.hl.addWidget(self.l_service)
        self.hl.addWidget(self.b_stop_service)
        self.hl.addWidget(self.b_start_service)
        self.hl.addWidget(self.l_ok_service)
    
    def stop(self,service):
        os.system(f'systemctl --user kill --quiet {self.service}.service')

    def start(self,service):
        os.system(f'systemctl --user start --quiet {self.service}.service')
    
    def rafraichir(self):
        status = os.system(f'systemctl --user is-active --quiet {self.service}.service')
        if status == 0:
            # Le service marche
            self.l_service.setStyleSheet("color: green")
            self.b_stop_service.show()
            self.b_start_service.hide()
            self.l_ok_service.show()
            return True
        else :
            self.l_service.setStyleSheet("color: red")
            self.b_stop_service.hide()
            self.b_start_service.show()
            self.l_ok_service.hide()
            return False
        

class TabServices(QtWidgets.QWidget):

    def __init__(self,robot,signal_emitter):
        super().__init__()

        self.robot = robot
        self.signal_emitter = signal_emitter

        self.timer = QTimer()
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.rafraichir)

        layout = QVBoxLayout(self)

        topLayout = QHBoxLayout(self)
        title = QtWidgets.QLabel("SERVICES")
        title.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        topLayout.addWidget(title)

        layout.addLayout(topLayout)

        vbox = QtWidgets.QVBoxLayout()

        self.services_widgets: list[ServiceWidget] = []

        for service in LISTE_SERVICE:
            service_w = ServiceWidget(service)
            self.services_widgets.append(service_w)
            vbox.addWidget(service_w)

        layout.addLayout(vbox)

        self.rafraichir()

        self.timer.start()
    
    def rafraichir(self):
        self.robot.nbServicesActifs = 0
        for service in self.services_widgets:
            if service.rafraichir() == True:
                self.robot.nbServicesActifs +=1
        self.signal_emitter.service_signal.emit()
            

    

##############
#            #
#    TAB     #
# ACTIONNEUR #
#            #
##############

class ActionneurWidget(QWidget):

    def __init__(self, id, sap_master, parent: QWidget = None) -> None:
        super().__init__(parent)
        self.id = id
        self.sap_master = sap_master
        self.hl = QHBoxLayout(self)
        self.l_act = QtWidgets.QLabel(f"{id}")
        self.b_ping_act = QPushButton("Ping"); self.b_ping_act.setMaximumSize(80, 35)
        self.b_ping_act.clicked.connect(self.ping)
        self.l_status_act = QLabel("V")

        self.hl.addWidget(self.l_act)
        self.hl.addWidget(self.b_ping_act)
        self.hl.addWidget(self.l_status_act)

    def ping(self):
        self.sap_master.protocol.ping(self.id)
    
    def rafraichir(self):
        status = self.sap_master.protocol.ping(self.id)
        if status:
            # L'actionneur répond
            self.l_act.setStyleSheet("color: green")
            self.l_status_act.setText("V")
            return True
        else :
            self.l_act.setStyleSheet("color: red")
            self.l_status_act.setText("X")
            return False

class TabActionneurs(QtWidgets.QWidget):

    def __init__(self,robot,signal_emitter):
        super().__init__()
        
        self.robot = robot
        self.signal_emitter = signal_emitter

        self.already_created = False # Ceci est un patch pour ajouter les widgets quand le service est actif (ie empecher les "Waiting for SAP service ...")

        layout = QtWidgets.QVBoxLayout(self)

        title = QtWidgets.QLabel("ACTIONNEURS")
        title.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        rafraichissement =  QtWidgets.QPushButton("refresh")
        rafraichissement.clicked.connect(self.rafraichir)
        layout.addWidget(title)
        layout.addWidget(rafraichissement)

        self.actionneurs_widgets: list[ActionneurWidget] = []
        
        self.vbox = QtWidgets.QVBoxLayout()
        
        self.error_msg = QLabel("Service IO inactif ..."); self.error_msg.setStyleSheet("color : red")
        self.error_msg.hide()

        self.vbox.addWidget(self.error_msg)

        layout.addLayout(self.vbox)

        self.rafraichir()

    def rafraichir(self):
        if os.system(f'systemctl --user is-active --quiet robot_IO.service') == 0:
            self.error_msg.hide()
            if not self.already_created:
                self.sap_master = SAPMaster()
                for act in LISTE_ID_ACTIONNEURS:
                    act_w = ActionneurWidget(act,self.sap_master)
                    self.actionneurs_widgets.append(act_w)
                    self.vbox.addWidget(act_w)
                self.already_created = True

            self.robot.nbActionneursActifs = 0
            for act_w in self.actionneurs_widgets:
                if act_w.rafraichir() == True:
                    self.robot.nbActionneursActifs +=1
            self.signal_emitter.actionneur_signal.emit(True)
        else : 
            self.error_msg.show()
            self.signal_emitter.actionneur_signal.emit(False)

#############
#           #
#    TAB    #
#   RADAR   #
#           #
#############
class TabRadar(QtWidgets.QWidget):

    def __init__(self):
        super().__init__()

        layout = QtWidgets.QVBoxLayout(self)

        title = QtWidgets.QLabel("RADAR")
        title.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)

        #parser = argparse.ArgumentParser()
        #parser.add_argument("-d", "--lidar", default="lidar_data", help="LidarData topic name")
        #parser.add_argument("-l", "--no-loca", action="store_false", default=True, help="Do not draw localisation helpers")
        #parser.add_argument("-p", "--pi", action="store_true", default=False, help="run with the linuxfb platform")
        #args = parser.parse_args()

        self.radar = RadarView("lidar_data",True)
        
        layout.addWidget(title)
        layout.addWidget(self.radar)


#############
#           #
#    TAB    #
#   CAMERA  #
#           #
#############
class TabCameras(QtWidgets.QWidget):

    def __init__(self):
        super().__init__()

        layout = QtWidgets.QVBoxLayout(self)

        title = QtWidgets.QLabel("CAMERAS")
        title.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)

        layout.addWidget(title)

class MainWindow(QtWidgets.QMainWindow):

    def __init__(self,robot,signal_emitter):
        super().__init__()

        self.robot = robot
        self.signal_emitter = signal_emitter

        self.resize(450, 540)

        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)

        self.l_score = QtWidgets.QLabel(f"SCORE : {self.robot.score}")
        self.l_score.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight)

        layout.addWidget(self.l_score)


        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setTabPosition(QtWidgets.QTabWidget.TabPosition.West)

        self.tabStatus = TabStatus(self.robot,self.signal_emitter)
        self.tabService = TabServices(self.robot,self.signal_emitter)
        self.tabActionneurs = TabActionneurs(self.robot,self.signal_emitter)
        self.tabRadar = TabRadar()
        self.tabCameras = TabCameras()

        self.tabs.addTab(self.tabStatus, "Status")
        self.tabs.addTab(self.tabService, "Services")
        self.tabs.addTab(self.tabActionneurs, "Actionneurs")
        self.tabs.addTab(self.tabRadar, "Radar")
        self.tabs.addTab(self.tabCameras, "Cameras")


        layout.addWidget(self.tabs)

        self.setCentralWidget(central)


if __name__ == "__main__":
    ecal_core.initialize("walUIgi")

    ## Gestion pour la rasberry
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--pi", action="store_true", default=False, help="run with the linuxfb platform")
    args = parser.parse_args()

    robot = Robot()
    signal_emitter = SignalEmitter()
    
    ## Gestion application
    
    if args.pi:
        sys.argv.extend(["-platform", "linuxfb"])

    app = QtWidgets.QApplication(sys.argv)
    main_widget = MainWindow(robot,signal_emitter)

    if args.pi:
        window = RotatedWindow(main_widget, angle=90)
    else:
        window = main_widget

    window.show()
    sys.exit(app.exec())
