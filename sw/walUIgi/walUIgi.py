#!/usr/bin/env python3

import sys
import argparse
import math
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel,
    QGraphicsView, QGraphicsScene, QGraphicsProxyWidget,
    QFrame,QGraphicsItemGroup,
    QGraphicsEllipseItem, QGraphicsTextItem, QGraphicsLineItem
)
from PyQt6.QtGui import QImage, QPainter, QColor, QPen, QTransform
from PyQt6.QtCore import Qt,pyqtSignal,QObject


import generated.common_pb2 as hgpb
import generated.lidar_data_pb2 as lidarpb

import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Subscriber as ProtoSubscriber
from ecal.msg.common.core import ReceiveCallbackData

#from radarqt import RadarView

class MySignalEmitter(QObject):
    pos_lidar_signal = pyqtSignal(float, float, float)
    pos_odom_signal = pyqtSignal(float, float, float)
    balise_signal = pyqtSignal(int)

class Robot:
    def __init__(self):
        self.name = "Dave"
        self.couleur = "Jaune"
        self.position_lidar = (0, 0, 0)
        self.position_odom = (0, 0, 0)
        self.nbBalises = 0
        self.score = 0

    def changeCouleur(self, col):
        self.couleur = col

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


class Status(QWidget):
    def __init__(self, robot):
        super().__init__()
        self.setWindowTitle("Menu")
        
        self.signal_emitter = MySignalEmitter()

        self.robot = robot
        self.rectangle_couleur = QPushButton()
        self.rectangle_couleur.setStyleSheet("background-color: yellow")
        self.label_couleur = QLabel("Couleur : " + self.robot.couleur)

        self.label_position_lidar = QLabel("    lidar : No ecal message (yet)")
        self.pos_lidar_sub = ProtoSubscriber(hgpb.Position,"lidar_pos")
        def sendPosLidarSignal( pub_id : ecal_core.TopicId, data : ReceiveCallbackData[hgpb.Position]):
            self.signal_emitter.pos_lidar_signal.emit(data.message.x, data.message.y, data.message.theta)
        self.pos_lidar_sub.set_receive_callback(sendPosLidarSignal)
        self.signal_emitter.pos_lidar_signal.connect(self.updatePositionLidar)

        self.label_position_odom = QLabel("    odom : No ecal message (yet)")
        self.pos_odom_sub = ProtoSubscriber(hgpb.Position,"odom_pos")
        def sendPosOdomSignal( pub_id : ecal_core.TopicId, data : ReceiveCallbackData[hgpb.Position]):
            self.signal_emitter.pos_odom_signal.emit(data.message.x, data.message.y, data.message.theta)
        self.pos_odom_sub.set_receive_callback(sendPosOdomSignal)
        self.signal_emitter.pos_odom_signal.connect(self.updatePositionOdom)

        self.label_nbBalises = QLabel("Nombre balises : No ecal message (yet)")
        self.balise_sub = ProtoSubscriber(lidarpb.Balises,"balises_odom")
        def sendBaliseSignal( pub_id : ecal_core.TopicId, data : ReceiveCallbackData[lidarpb.Balises]):
            self.signal_emitter.balise_signal.emit(len(data.message.index))
        self.balise_sub.set_receive_callback(sendBaliseSignal)
        self.signal_emitter.balise_signal.connect(self.updateBalise)

        self.label_score = QLabel("Score : " + str(self.robot.score))


        self.initUI()

    def initUI(self):
        layoutPrincipal = QVBoxLayout(self)

        ## Status
        layoutStatus = QVBoxLayout()
        self.rectangle_couleur.clicked.connect(lambda : self.updateCouleur(False if self.robot.couleur=="Jaune" else True))
        self.rectangle_couleur.setMinimumHeight(150);layoutStatus.addWidget(self.rectangle_couleur)
        label_name = QLabel(self.robot.name); label_name.setStyleSheet("color: #561273; font-family: Arial; font-size: 24px"); layoutStatus.addWidget(label_name)
        label_status = QLabel("=== Status ==="); label_status.setStyleSheet("font-family: Arial; font-size: 18px"); layoutStatus.addWidget(label_status)
        self.label_couleur.setStyleSheet("font-family: Arial; font-size: 16px"); layoutStatus.addWidget(self.label_couleur)
        self.label_position_lidar.setStyleSheet("font-family: Arial; font-size: 16px"); layoutStatus.addWidget(self.label_position_lidar)
        self.label_position_odom.setStyleSheet("font-family: Arial; font-size: 16px"); layoutStatus.addWidget(self.label_position_odom)
        self.label_nbBalises.setStyleSheet("font-family: Arial; font-size: 16px"); layoutStatus.addWidget(self.label_nbBalises)
        self.label_score.setStyleSheet("font-family: Arial; font-size: 16px"); layoutStatus.addWidget(self.label_score)
        layoutPrincipal.addLayout(layoutStatus)

        ## Choix Couleur
        layoutCouleur = QHBoxLayout()
        boutonJ = QPushButton("Jaune")
        boutonJ.clicked.connect(lambda: self.updateCouleur(True))
        layoutCouleur.addWidget(boutonJ)
        boutonB = QPushButton("Bleu")
        boutonB.clicked.connect(lambda: self.updateCouleur(False))
        layoutCouleur.addWidget(boutonB)
        layoutPrincipal.addLayout(layoutCouleur)

        ## Ouvrir RadarQT
        bouton_radarqt = QPushButton("Ouvrir RadarQT")
        bouton_radarqt.clicked.connect(self.ouvrirRadarqt)
        layoutPrincipal.addWidget(bouton_radarqt)

        ## Map style RobotKontroll avec position robot (+ Balises détectés via radarqt ?)
        
    def updateCouleur(self, jaune):
        if jaune:
            self.robot.changeCouleur("Jaune")
            self.rectangle_couleur.setStyleSheet("background-color: yellow")
        else:
            self.robot.changeCouleur("Bleu")
            self.rectangle_couleur.setStyleSheet("background-color: blue")
        self.label_couleur.setText("Couleur : " + self.robot.couleur)

    def updatePositionLidar(self,x,y,theta):
        self.robot.position_lidar = (x,y,theta)
        self.label_position_lidar.setText("  lidar : " + "x=" + format(x,".1f") + "    y=" + format(y,".1f") + "    θ=" + format(math.degrees(theta),".1f") + "°")

    def updatePositionOdom(self,x,y,theta):
        self.robot.position_odom = (x,y,theta)
        self.label_position_odom.setText("   odom : " + "x=" + format(x,".1f") + "    y=" + format(y,".1f") + "    θ=" + format(math.degrees(theta),".1f") + "°")

    def updateBalise(self,nb):
        self.robot.nbBalises = nb
        self.label_nbBalises.setText("Nombre balises : " + str(nb))

class Radar(QWidget):
    def __init__(self, robot):
        super().__init__()
        self.setWindowTitle("Menu")
        
        self.signal_emitter = MySignalEmitter()

        self.robot = robot
        self.rectangle_couleur = QPushButton()
        self.rectangle_couleur.setStyleSheet("background-color: yellow")
        self.label_couleur = QLabel("Couleur : " + self.robot.couleur)

        self.label_position_lidar = QLabel("    lidar : No ecal message (yet)")
        self.pos_lidar_sub = ProtoSubscriber(hgpb.Position,"lidar_pos")
        def sendPosLidarSignal( pub_id : ecal_core.TopicId, data : ReceiveCallbackData[hgpb.Position]):
            self.signal_emitter.pos_lidar_signal.emit(data.message.x, data.message.y, data.message.theta)
        self.pos_lidar_sub.set_receive_callback(sendPosLidarSignal)
        self.signal_emitter.pos_lidar_signal.connect(self.updatePositionLidar)

        self.label_position_odom = QLabel("    odom : No ecal message (yet)")
        self.pos_odom_sub = ProtoSubscriber(hgpb.Position,"odom_pos")
        def sendPosOdomSignal( pub_id : ecal_core.TopicId, data : ReceiveCallbackData[hgpb.Position]):
            self.signal_emitter.pos_odom_signal.emit(data.message.x, data.message.y, data.message.theta)
        self.pos_odom_sub.set_receive_callback(sendPosOdomSignal)
        self.signal_emitter.pos_odom_signal.connect(self.updatePositionOdom)

        self.label_nbBalises = QLabel("Nombre balises : No ecal message (yet)")
        self.balise_sub = ProtoSubscriber(lidarpb.Balises,"balises_odom")
        def sendBaliseSignal( pub_id : ecal_core.TopicId, data : ReceiveCallbackData[lidarpb.Balises]):
            self.signal_emitter.balise_signal.emit(len(data.message.index))
        self.balise_sub.set_receive_callback(sendBaliseSignal)
        self.signal_emitter.balise_signal.connect(self.updateBalise)

        self.label_score = QLabel("Score : " + str(self.robot.score))



if __name__ == "__main__":
    ecal_core.initialize("walUIgi")
    
    ## Gestion pour la rasberry
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--pi", action="store_true", default=False, help="run with the linuxfb platform")
    args = parser.parse_args()
    if args.pi:
        sys.argv.extend(["-platform", "linuxfb"])

    ## Gestion application
    app = QApplication(sys.argv)
    robot = Robot()
    main_widget = Status(robot)
    window = RotatedWindow(main_widget, angle=90)
    window.show()
    sys.exit(app.exec())