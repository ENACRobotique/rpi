#!/usr/bin/python3
from PyQt5 import QtCore, QtWidgets, QtGui
import time
import math
import sys
sys.path.append('../generated')
import lidar_data_pb2  as pbl
import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber


class RadarView(QtWidgets.QWidget):
    COLOR_SCALE = ["#00876c", "#3d9c73", "#63b179",
                   "#88c580", "#aed987", "#d6ec91",
                   "#ffff9d", "#fee17e", "#fcc267",
                   "#f7a258", "#ef8250", "#e4604e",
                   "#d43d51"]
    vl53_data = QtCore.pyqtSignal(list)

    def __init__(self, nb, parent):
        QtWidgets.QWidget.__init__(self, parent)
        ecal_core.initialize(sys.argv, "VL53Visu")
        self.lidar_sub = ProtoSubscriber(f"vl53_{nb}", pbl.Lidar)
        self.lidar_sub.set_callback(self.handle_lidar_data)
        self.vl53_data.connect(self.handle_data)
        self.data = []
        self.data = [0 for _ in range(64)]
        self.back = []
        self.last_tour_time = time.time()
        self.frequency = 0
        self.last_angle = 0
        self.mm_to_pixel = 0.1
        self.setSizePolicy(
            QtWidgets.QSizePolicy.MinimumExpanding,
            QtWidgets.QSizePolicy.MinimumExpanding
        )

    def handle_lidar_data(self, topic_name, msg, time):
        self.vl53_data.emit(list(msg.distances))

    def handle_data(self, distances):
        self.data = distances
        self.update()

    def set_speed(self, speed):
        self.frequency = speed/60

    def color_from_distance(self, distance):
        v = min(255, distance / 200 * 255)
        c = QtGui.QColor(int(v), 0, 0)
        #color_index = len(self.COLOR_SCALE) - int(distance * len(self.COLOR_SCALE) / 2000) - 1
        #c = QtGui.QColor(self.COLOR_SCALE[color_index])
        return c

    def wheelEvent(self, e: QtGui.QWheelEvent) -> None:
        d = e.angleDelta().y()
        self.mm_to_pixel *= (1 + d / 1000)
        self.update()

    def mousePressEvent(self, a0: QtGui.QMouseEvent) -> None:
        ...

    def paintEvent(self, e: QtGui.QPaintEvent) -> None:
        painter = QtGui.QPainter(self)
        rect = QtCore.QRect(0, 0, painter.device().width(), painter.device().height())

        # paint scale
        for i, color in enumerate(self.COLOR_SCALE):
            scale_rect = QtCore.QRect(rect.right()-50, 10 + i*20, 40, 20)
            painter.setBrush(QtGui.QColor(color))
            painter.setPen(QtCore.Qt.NoPen)
            painter.drawRect(scale_rect)

        px_w = rect.width() // 8
        px_h = rect.height() // 8

        def idx(x, y):
            return (7 - y) * 8 + (7 - x)

        min_idxs = []
        min_dists = []
        for y in range(8):
            if 6 > y > 2:
                index_min = min(range(8), key=lambda x: self.data[idx(x, y)])
                min_idxs.append(index_min)
                min_dists.append(self.data[idx(index_min, y)])
            for x in range(8):
                pixel_rect = QtCore.QRect(x * px_w, y * px_h, px_w, px_h)
                if (6 > y > 2) or True:
                    d = self.data[idx(x, y)]
                    c = self.color_from_distance(d)
                    #if x == index_min:
                    #    c = QtGui.QColor("#00AA00")
                else:
                    c = QtGui.QColor("#0000AA")
                painter.setBrush(c)
                painter.drawRect(pixel_rect)

        avr_x = sum(min_idxs) / len(min_idxs)
        avr_dist = sum(min_dists) / len(min_dists)
        if avr_dist < 200:
            painter.setBrush(QtGui.QColor("#db34eb"))
            painter.drawRect(QtCore.QRect(int((avr_x) * px_w), 0, px_w, rect.height()))

    def sizeHint(self) -> QtCore.QSize:
        return QtCore.QSize(400, 400)


class ApplicationWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self._main = QtWidgets.QWidget()
        self.setCentralWidget(self._main)
        layout = QtWidgets.QVBoxLayout(self._main)

        self.radarView = RadarView(sys.argv[1], parent=self._main)
        layout.addWidget(self.radarView)


if __name__ == "__main__":
    qapp = QtWidgets.QApplication(sys.argv)
    app = ApplicationWindow()
    app.setWindowTitle(f"VL53 visu {sys.argv[1]}")
    app.show()
    app.activateWindow()
    app.raise_()
    qapp.exec_()
