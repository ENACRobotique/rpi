#!/usr/bin/env python3
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtCore import QObject, pyqtSignal
import time
import math
import sys
import argparse
import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Subscriber as ProtoSubscriber
from ecal.msg.common.core import ReceiveCallbackData
sys.path.append('../../generated')
import lidar_data_pb2  as pbl

TOLERANCE = 500

class RadarView(QtWidgets.QWidget):
    COLOR_SCALE = ["#00876c", "#3d9c73", "#63b179",
                   "#88c580", "#aed987", "#d6ec91",
                   "#ffff9d", "#fee17e", "#fcc267",
                   "#f7a258", "#ef8250", "#e4604e",
                   "#d43d51"]
    lidar_data_sig = pyqtSignal(list)
    amalgame_sig = pyqtSignal(list)
    balises_odom_sig=pyqtSignal(list)
    balises_nearodom_sig=pyqtSignal(list)
    transforms_sig=pyqtSignal(list)

    def __init__(self, args, parent=None):
        QtWidgets.QWidget.__init__(self, parent)
        self.args = args
        if not ecal_core.is_initialized():
            ecal_core.initialize("RadarQt receiver")
        self.lidar_sub = ProtoSubscriber(pbl.Lidar, args.lidar)
        self.lidar_sub.set_receive_callback(self.handle_lidar_data)
        if args.no_loca:
            self.lidar_amalgames_sub = ProtoSubscriber(pbl.Amalgames, "amalgames")
            self.lidar_amalgames_sub.set_receive_callback(self.handle_amalgames_data)
            self.lidar_balises_odom_sub = ProtoSubscriber(pbl.Balises, "balises_odom")
            self.lidar_balises_odom_sub.set_receive_callback(self.handle_balises_odom_data)   
            self.lidar_balises_nearodom_sub = ProtoSubscriber(pbl.Balises, "balises_near_odom")
            self.lidar_balises_nearodom_sub.set_receive_callback(self.handle_balises_nearodom_data)  

        self.lidar_data_sig.connect(self.lidar_cb)
        self.amalgame_sig.connect(self.lidar_amalgame_cb)
        self.balises_odom_sig.connect(self.lidar_balise_odom_cd)
        self.balises_nearodom_sig.connect(self.lidar_balise_nearodom_cd)

        self.data = []
        self.amalgame_data = []
        self.balise_odom_data=[]
        self.nearodom_data=[]
        self.transforms_data=[]
        self.back = []
        self.last_tour_time = time.time()
        self.period = 1
        self.last_angle = 0
        self.mm_to_pixel = 0.1
        self.setSizePolicy(
            QtWidgets.QSizePolicy.MinimumExpanding,
            QtWidgets.QSizePolicy.MinimumExpanding
        )

    def color_from_quality(self, quality):
        #color_index = quality - 15 + len(self.COLOR_SCALE) / 2
        color_index = len(self.COLOR_SCALE) - int(quality * len(self.COLOR_SCALE) / 255) - 1
        c = QtGui.QColor(self.COLOR_SCALE[color_index])
        return c

    def wheelEvent(self, e: QtGui.QWheelEvent) -> None:
        d = e.angleDelta().y()
        self.mm_to_pixel *= (1 + d / 1000)
        self.update()

    def lidar_cb(self, data):
           self.data = data
           self.update()
    
    def lidar_amalgame_cb(self, amalgame_data):
        self.amalgame_data = amalgame_data
        self.update()

    def lidar_balise_odom_cd(self, balise_odom_data)  :
        self.balise_odom_data=balise_odom_data
        self.update() 

    def lidar_balise_nearodom_cd(self, nearodom_data):
        self.nearodom_data=nearodom_data
        self.update()
    
    def handle_lidar_data(self, pub_id : ecal_core.TopicId, msg : ReceiveCallbackData[pbl.Lidar]):
        now = time.time()
        dt = now - self.last_tour_time
        self.last_tour_time = now
        self.period = self.period*0.4 + dt * 0.6   # passe bas

        data = list(zip(msg.message.angles, msg.message.distances, msg.message.quality))
        self.lidar_data_sig.emit(data)
    
    def handle_amalgames_data(self, pub_id : ecal_core.TopicId, data : ReceiveCallbackData[pbl.Amalgames]):
        data = list(zip(data.message.x,data.message.y,data.message.size))    
        self.amalgame_sig.emit(data)

    def handle_balises_odom_data(self, pub_id : ecal_core.TopicId, data : ReceiveCallbackData[pbl.Balises]):
        data = list(zip(data.message.index,data.message.x,data.message.y))    
        self.balises_odom_sig.emit(data)

    def handle_balises_nearodom_data(self, pub_id : ecal_core.TopicId, data : ReceiveCallbackData[pbl.Balises]):
        data = list(zip(data.message.index,data.message.x,data.message.y))    
        self.balises_nearodom_sig.emit(data)


    def mousePressEvent(self, a0: QtGui.QMouseEvent) -> None:
        self.data = []

    def paintEvent(self, e: QtGui.QPaintEvent) -> None:
        painter = QtGui.QPainter(self)

        # paint background
        brush = QtGui.QBrush()
        brush.setColor(QtGui.QColor('black'))
        brush.setStyle(QtCore.Qt.SolidPattern)
        rect = QtCore.QRect(0, 0, painter.device().width(), painter.device().height())
        painter.fillRect(rect, brush)

        # paint frequency
        painter.setPen(QtCore.Qt.white)
        txt = "{:0>5.2f} Hz".format(1/self.period)
        painter.drawText(10, 20, txt)

        # paint scale
        for i, color in enumerate(self.COLOR_SCALE):
            scale_rect = QtCore.QRect(rect.right()-50, 10 + i*20, 40, 20)
            painter.setBrush(QtGui.QColor(color))
            painter.setPen(QtCore.Qt.NoPen)
            painter.drawRect(scale_rect)

        # translate and rotate painter
        painter.translate(painter.device().width()/2, painter.device().height()/2)
        painter.rotate(-90)

        # draw 0° line
        painter.setPen(QtCore.Qt.gray)
        painter.drawLine(QtCore.QPoint(0, 0), QtCore.QPoint(1000, 0))

        # paint circles
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.setPen(QtCore.Qt.gray)
        painter.drawEllipse(QtCore.QPoint(0, 0), self.mm_to_pixel * 500, self.mm_to_pixel * 500)
        painter.drawEllipse(QtCore.QPoint(0, 0), self.mm_to_pixel * 1500, self.mm_to_pixel * 1500)
        painter.drawEllipse(QtCore.QPoint(0, 0), self.mm_to_pixel * 2500, self.mm_to_pixel * 2500)
        painter.drawEllipse(QtCore.QPoint(0, 0), self.mm_to_pixel * 3500, self.mm_to_pixel * 3500)

        painter.setPen(QtCore.Qt.white)
        painter.drawEllipse(QtCore.QPoint(0, 0), self.mm_to_pixel * 1000, self.mm_to_pixel * 1000)
        painter.drawEllipse(QtCore.QPoint(0, 0), self.mm_to_pixel * 2000, self.mm_to_pixel * 2000)
        painter.drawEllipse(QtCore.QPoint(0, 0), self.mm_to_pixel * 3000, self.mm_to_pixel * 3000)


        # draw center
        painter.setBrush(QtCore.Qt.green)
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawEllipse(QtCore.QPoint(0, 0), 5, 5)

        # paint points
        painter.setBrush(QtCore.Qt.yellow)
        painter.setPen(QtCore.Qt.NoPen)
        for angle, distance, quality in self.data:
            if quality != 0 and distance != 0:
                pos = QtCore.QPointF(self.mm_to_pixel * distance * math.cos(-angle), self.mm_to_pixel * distance * math.sin(-angle))
                size = 5
                c = self.color_from_quality(quality)
                painter.setBrush(c)
                painter.drawEllipse(pos, size, size)
        
        pen = QtGui.QPen(QtGui.QColor(255, 0, 255), 2)
        painter.setPen(pen)
        painter.setBrush(QtCore.Qt.NoBrush)
        for x, y, size in self.amalgame_data:
            if size < 200:
                pos = QtCore.QPointF(self.mm_to_pixel * x, -self.mm_to_pixel * y)
                size *= self.mm_to_pixel
                painter.drawEllipse(pos, size/2, size/2)

        painter.setPen(QtCore.Qt.NoPen)
        
        font = QtGui.QFont()
        font.setPointSize(20)  # Augmente la taille ici si besoin
        painter.setPen(QtGui.QColor(52, 68, 191))
        pen = QtGui.QPen(QtGui.QColor(52, 68, 191))
        pen.setWidth(4)
        painter.setPen(pen)
        # Position estimée des balises à partir de l'odométrie
        for index,x,y in self.balise_odom_data:
            pos = QtCore.QPointF(self.mm_to_pixel * x, -self.mm_to_pixel * y)
            size = 8
            # painter.drawEllipse(pos, size, size)
            # print('odometrie :',index,x,x)
            painter.drawEllipse(pos, TOLERANCE*self.mm_to_pixel, TOLERANCE*self.mm_to_pixel)
            painter.drawLine(
                QtCore.QPointF(pos.x() - size, pos.y()),
                QtCore.QPointF(pos.x() + size, pos.y())
            )

            # Ligne verticale
            painter.drawLine(
                QtCore.QPointF(pos.x(), pos.y() - size),
                QtCore.QPointF(pos.x(), pos.y() + size)
            )
            painter.setFont(font)
            painter.drawText(
                QtCore.QPointF(pos.x() - 10, pos.y() - 10),
                str(index))

        # Position estimée des balises par les moindres carrés
        painter.setPen(QtGui.QColor("red"))
        pen = QtGui.QPen(QtGui.QColor("red"))
        pen.setWidth(4)
        painter.setPen(pen)
        for index,x,y in self.nearodom_data:
            pos = QtCore.QPointF(self.mm_to_pixel * x, -self.mm_to_pixel * y)
            size = 8    
            # print('estimation : ',index,x,x)
            painter.drawEllipse(pos, TOLERANCE*self.mm_to_pixel, TOLERANCE*self.mm_to_pixel)
            # Ligne horizontale
            painter.drawLine(
                QtCore.QPointF(pos.x() - size, pos.y()),
                QtCore.QPointF(pos.x() + size, pos.y())
            )

            # Ligne verticale
            painter.drawLine(
                QtCore.QPointF(pos.x(), pos.y() - size),
                QtCore.QPointF(pos.x(), pos.y() + size)
            )
            painter.setFont(font)
            painter.drawText(
                QtCore.QPointF(pos.x() + 10, pos.y() - 10),
                str(index))




    def sizeHint(self) -> QtCore.QSize:
        return QtCore.QSize(400, 400)


class ApplicationWindow(QtWidgets.QMainWindow):
    def __init__(self, args):
        super().__init__()
        self._main = QtWidgets.QWidget()
        self.setCentralWidget(self._main)
        layout = QtWidgets.QVBoxLayout(self._main)
        self.radarView = RadarView(args, self._main)
        layout.addWidget(self.radarView)
    
if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--lidar", default="lidar_data", help="LidarData topic name")
    parser.add_argument("-l", "--no-loca", action="store_false", default=True, help="Do not draw localisation helpers")
    args = parser.parse_args()

    qapp = QtWidgets.QApplication(sys.argv)
    app = ApplicationWindow(args)
    app.show()
    app.activateWindow()
    app.raise_()
    qapp.exec_()
