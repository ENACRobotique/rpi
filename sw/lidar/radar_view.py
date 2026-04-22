#!/usr/bin/env python3
from PyQt6 import QtCore, QtWidgets, QtGui
from PyQt6.QtCore import QObject, pyqtSignal
import time
import math
import sys
import argparse
import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Subscriber as ProtoSubscriber
from ecal.msg.common.core import ReceiveCallbackData
import lidar_data_pb2  as pbl

TOLERANCE = 500
ODOM_COLOR = "#4fc3f7"
ODOM_PREDICTION_STYLE = "arc"
ODOM_ARC_RADIUS_DIFF = 300
ODOM_ARC_ANGLE_DEG = 16

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

    def __init__(self, topic, no_loca, parent=None):
        QtWidgets.QWidget.__init__(self, parent)
        self.no_loca = no_loca
        self._owns_ecal = not ecal_core.is_initialized()
        if self._owns_ecal:
            ecal_core.initialize("RadarQt receiver")
        self.lidar_sub = ProtoSubscriber(pbl.Lidar, topic)
        self.lidar_sub.set_receive_callback(self.handle_lidar_data)
        if no_loca:
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
        self.pan_offset = QtCore.QPointF(0, 0)
        self._dragging = False
        self._last_mouse_pos = QtCore.QPointF()
        self.show_odom_prediction = True
        self.show_found_beacons = True
        self.odom_prediction_style = ODOM_PREDICTION_STYLE
        self._resources_released = False
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.MinimumExpanding,
            QtWidgets.QSizePolicy.Policy.MinimumExpanding
        )
        self._init_overlay_controls()
        app = QtWidgets.QApplication.instance()
        if app is not None:
            app.aboutToQuit.connect(self.release_resources)

    def release_resources(self) -> None:
        if self._resources_released:
            return

        self._resources_released = True
        if self.no_loca:
            self.lidar_amalgames_sub.remove_receive_callback()
            self.lidar_balises_odom_sub.remove_receive_callback()
            self.lidar_balises_nearodom_sub.remove_receive_callback()
        self.lidar_sub.remove_receive_callback()

        if self._owns_ecal and ecal_core.is_initialized():
            ecal_core.finalize()

    def _init_overlay_controls(self) -> None:
        self.controls_widget = QtWidgets.QFrame(self)
        self.controls_widget.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.controls_widget.setStyleSheet(
            "QFrame { background-color: rgba(0, 0, 0, 160); border: 1px solid rgba(255, 255, 255, 70); border-radius: 6px; }"
            "QToolButton { color: white; background-color: rgba(255, 255, 255, 22); border: 1px solid rgba(255, 255, 255, 70); border-radius: 22px; padding: 8px; font-weight: 600; }"
            "QToolButton:pressed { background-color: rgba(255, 255, 255, 34); }"
            "QCheckBox { color: white; spacing: 8px; }"
            "QCheckBox::indicator { width: 16px; height: 16px; border-radius: 3px; border: 1px solid rgba(255, 255, 255, 160); background-color: rgba(255, 255, 255, 30); }"
            "QCheckBox::indicator:checked { background-color: #4fc3f7; border: 1px solid #b3e5fc; }"
            "QCheckBox::indicator:unchecked { background-color: rgba(255, 255, 255, 20); }"
            "QCheckBox::indicator:disabled { background-color: rgba(255, 255, 255, 10); border: 1px solid rgba(255, 255, 255, 60); }"
        )
        self.overlay_controls_expanded = True
        self.controls_widget.installEventFilter(self)

        controls_layout = QtWidgets.QVBoxLayout(self.controls_widget)
        controls_layout.setContentsMargins(10, 8, 10, 8)
        controls_layout.setSpacing(6)

        self.controls_content = QtWidgets.QWidget(self.controls_widget)
        self.controls_content.installEventFilter(self)
        content_layout = QtWidgets.QVBoxLayout(self.controls_content)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(6)

        self.show_odom_checkbox = QtWidgets.QCheckBox("Show odom prediction", self.controls_content)
        self.show_odom_checkbox.setChecked(self.show_odom_prediction)
        self.show_odom_checkbox.toggled.connect(self.set_show_odom_prediction)
        self.show_odom_checkbox.installEventFilter(self)
        content_layout.addWidget(self.show_odom_checkbox)

        self.show_found_checkbox = QtWidgets.QCheckBox("Show found beacons", self.controls_content)
        self.show_found_checkbox.setChecked(self.show_found_beacons)
        self.show_found_checkbox.toggled.connect(self.set_show_found_beacons)
        self.show_found_checkbox.installEventFilter(self)
        content_layout.addWidget(self.show_found_checkbox)

        self.arc_prediction_checkbox = QtWidgets.QCheckBox("Use arc prediction", self.controls_content)
        self.arc_prediction_checkbox.setChecked(self.odom_prediction_style == "arc")
        self.arc_prediction_checkbox.toggled.connect(self.set_use_arc_prediction)
        self.arc_prediction_checkbox.installEventFilter(self)
        content_layout.addWidget(self.arc_prediction_checkbox)
        controls_layout.addWidget(self.controls_content)

        self.collapsed_controls_button = QtWidgets.QToolButton(self)
        self.collapsed_controls_button.setText("⋯")
        self.collapsed_controls_button.setAutoRaise(False)
        self.collapsed_controls_button.setFixedSize(44, 44)
        self.collapsed_controls_button.clicked.connect(self.expand_overlay_controls)
        self.collapsed_controls_button.setStyleSheet(
            "QToolButton {"
            " color: white;"
            " background-color: rgba(0, 0, 0, 140);"
            " border: 1px solid rgba(255, 255, 255, 110);"
            " border-radius: 12px;"
            " font-size: 24px;"
            " font-weight: 700;"
            " padding-bottom: 3px;"
            "}"
            "QToolButton:pressed {"
            " background-color: rgba(255, 255, 255, 55);"
            " border: 1px solid rgba(255, 255, 255, 160);"
            "}"
        )
        self.collapsed_controls_button.hide()

        if not self.no_loca:
            self.show_odom_checkbox.setChecked(False)
            self.show_odom_checkbox.setEnabled(False)
            self.show_found_checkbox.setChecked(False)
            self.show_found_checkbox.setEnabled(False)
            self.arc_prediction_checkbox.setChecked(False)
            self.arc_prediction_checkbox.setEnabled(False)

        self._update_overlay_controls_ui()
        self.controls_widget.adjustSize()
        self._position_overlay_controls()

    def _position_overlay_controls(self) -> None:
        margin = 12
        self.controls_widget.adjustSize()
        self.controls_widget.move(margin, margin)
        self.collapsed_controls_button.move(margin, margin)

    def _update_overlay_controls_ui(self) -> None:
        if self.overlay_controls_expanded:
            self.controls_widget.show()
            self.collapsed_controls_button.hide()
        else:
            self.controls_widget.hide()
            self.collapsed_controls_button.show()

    def toggle_overlay_controls(self) -> None:
        self.overlay_controls_expanded = not self.overlay_controls_expanded
        self._update_overlay_controls_ui()
        self._position_overlay_controls()

    def expand_overlay_controls(self) -> None:
        self.overlay_controls_expanded = True
        self._update_overlay_controls_ui()
        self._position_overlay_controls()

    def eventFilter(self, obj: QObject, e: QtCore.QEvent) -> bool:
        if (
            self.overlay_controls_expanded
            and e.type() == QtCore.QEvent.Type.MouseButtonPress
            and obj in (self.controls_widget, self.controls_content)
        ):
            child = obj.childAt(e.position().toPoint())
            if not isinstance(child, QtWidgets.QCheckBox):
                self.toggle_overlay_controls()
                return True

        if (
            self.overlay_controls_expanded
            and e.type() == QtCore.QEvent.Type.MouseButtonPress
            and isinstance(obj, QtWidgets.QCheckBox)
        ):
            option = QtWidgets.QStyleOptionButton()
            obj.initStyleOption(option)
            indicator_rect = obj.style().subElementRect(
                QtWidgets.QStyle.SubElement.SE_CheckBoxIndicator,
                option,
                obj
            )
            if not indicator_rect.contains(e.position().toPoint()):
                self.toggle_overlay_controls()
                return True
        return super().eventFilter(obj, e)

    def color_from_quality(self, quality):
        #color_index = quality - 15 + len(self.COLOR_SCALE) / 2
        color_index = len(self.COLOR_SCALE) - int(quality * len(self.COLOR_SCALE) / 255) - 1
        c = QtGui.QColor(self.COLOR_SCALE[color_index])
        return c

    def wheelEvent(self, e: QtGui.QWheelEvent) -> None:
        d = e.angleDelta().y()
        self.mm_to_pixel *= (1 + d / 1000)
        self.update()

    def mousePressEvent(self, e: QtGui.QMouseEvent) -> None:
        if e.button() == QtCore.Qt.MouseButton.LeftButton:
            self._dragging = True
            self._last_mouse_pos = e.position()
            self.setCursor(QtCore.Qt.CursorShape.ClosedHandCursor)
            e.accept()
            return
        super().mousePressEvent(e)

    def mouseMoveEvent(self, e: QtGui.QMouseEvent) -> None:
        if self._dragging:
            delta = e.position() - self._last_mouse_pos
            self.pan_offset += delta
            self._last_mouse_pos = e.position()
            self.update()
            e.accept()
            return
        super().mouseMoveEvent(e)

    def mouseReleaseEvent(self, e: QtGui.QMouseEvent) -> None:
        if e.button() == QtCore.Qt.MouseButton.LeftButton and self._dragging:
            self._dragging = False
            self.setCursor(QtCore.Qt.CursorShape.ArrowCursor)
            e.accept()
            return
        super().mouseReleaseEvent(e)

    def resizeEvent(self, e: QtGui.QResizeEvent) -> None:
        self._position_overlay_controls()
        super().resizeEvent(e)

    def closeEvent(self, e: QtGui.QCloseEvent) -> None:
        self.release_resources()
        super().closeEvent(e)

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

    def set_show_odom_prediction(self, enabled: bool) -> None:
        self.show_odom_prediction = enabled
        self.update()

    def set_show_found_beacons(self, enabled: bool) -> None:
        self.show_found_beacons = enabled
        self.update()

    def set_use_arc_prediction(self, enabled: bool) -> None:
        self.odom_prediction_style = "arc" if enabled else "circle"
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

    def draw_upright_text(self, painter: QtGui.QPainter, world_pos: QtCore.QPointF, text: str, color: QtGui.QColor, font: QtGui.QFont, offset: QtCore.QPointF) -> None:
        screen_pos = painter.worldTransform().map(world_pos)
        painter.save()
        painter.resetTransform()
        painter.setPen(color)
        painter.setFont(font)
        painter.drawText(screen_pos + offset, text)
        painter.restore()

    def draw_odom_prediction_zone(self, painter: QtGui.QPainter, pos: QtCore.QPointF, color: QtGui.QColor) -> None:
        if self.odom_prediction_style == "circle":
            painter.drawEllipse(pos, TOLERANCE * self.mm_to_pixel, TOLERANCE * self.mm_to_pixel)
            return

        if self.odom_prediction_style != "arc":
            raise ValueError(f"Unsupported odom prediction style: {self.odom_prediction_style}")

        radius = math.hypot(pos.x(), pos.y())
        radial_half_width = ODOM_ARC_RADIUS_DIFF * self.mm_to_pixel / 2
        inner_radius = max(0, radius - radial_half_width)
        outer_radius = radius + radial_half_width

        angle_center_deg = -math.degrees(math.atan2(pos.y(), pos.x()))
        start_angle_deg = angle_center_deg - ODOM_ARC_ANGLE_DEG / 2

        outer_rect = QtCore.QRectF(-outer_radius, -outer_radius, 2 * outer_radius, 2 * outer_radius)
        inner_rect = QtCore.QRectF(-inner_radius, -inner_radius, 2 * inner_radius, 2 * inner_radius)

        path = QtGui.QPainterPath()
        path.arcMoveTo(outer_rect, start_angle_deg)
        path.arcTo(outer_rect, start_angle_deg, ODOM_ARC_ANGLE_DEG)
        if inner_radius > 0:
            path.arcTo(inner_rect, start_angle_deg + ODOM_ARC_ANGLE_DEG, -ODOM_ARC_ANGLE_DEG)
        else:
            path.lineTo(0, 0)
        path.closeSubpath()

        painter.save()
        fill_color = QtGui.QColor(color)
        fill_color.setAlpha(48)
        painter.setBrush(fill_color)
        painter.drawPath(path)
        painter.restore()

    def odom_label_position(self, pos: QtCore.QPointF, margin_px: float = 12) -> QtCore.QPointF:
        radius = math.hypot(pos.x(), pos.y())
        if radius == 0:
            return QtCore.QPointF(margin_px, -margin_px)

        if self.odom_prediction_style == "circle":
            direction_x = pos.x() / radius
            direction_y = pos.y() / radius
            offset = TOLERANCE * self.mm_to_pixel + margin_px
            return QtCore.QPointF(
                pos.x() + direction_x * offset,
                pos.y() + direction_y * offset
            )
        elif self.odom_prediction_style == "arc":
            outer_radius = radius + ODOM_ARC_RADIUS_DIFF * self.mm_to_pixel / 2
        else:
            raise ValueError(f"Unsupported odom prediction style: {self.odom_prediction_style}")

        scale = (outer_radius + margin_px) / radius
        return QtCore.QPointF(pos.x() * scale, pos.y() * scale)

    def paintEvent(self, e: QtGui.QPaintEvent) -> None:
        painter = QtGui.QPainter(self)

        # paint background
        brush = QtGui.QBrush()
        brush.setColor(QtGui.QColor('black'))
        brush.setStyle(QtCore.Qt.BrushStyle.SolidPattern)
        rect = QtCore.QRect(0, 0, painter.device().width(), painter.device().height())
        painter.fillRect(rect, brush)

        # paint frequency
        painter.setPen(QtCore.Qt.GlobalColor.white)
        txt = "{:0>5.2f} Hz".format(1/self.period)
        painter.drawText(rect.right() - 110, 20, txt)

        # paint scale
        for i, color in enumerate(self.COLOR_SCALE):
            scale_rect = QtCore.QRect(rect.right()-50, 10 + i*20, 40, 20)
            painter.setBrush(QtGui.QColor(color))
            painter.setPen(QtCore.Qt.PenStyle.NoPen)
            painter.drawRect(scale_rect)

        # translate and rotate painter
        painter.translate(
            painter.device().width()/2 + self.pan_offset.x(),
            painter.device().height()/2 + self.pan_offset.y()
        )
        painter.rotate(-90)

        # draw 0° line
        painter.setPen(QtCore.Qt.GlobalColor.gray)
        painter.drawLine(QtCore.QPoint(0, 0), QtCore.QPoint(1000, 0))

        # paint circles
        painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        painter.setPen(QtCore.Qt.GlobalColor.gray)
        painter.drawEllipse(QtCore.QPointF(0, 0), self.mm_to_pixel * 500, self.mm_to_pixel * 500)
        painter.drawEllipse(QtCore.QPointF(0, 0), self.mm_to_pixel * 1500, self.mm_to_pixel * 1500)
        painter.drawEllipse(QtCore.QPointF(0, 0), self.mm_to_pixel * 2500, self.mm_to_pixel * 2500)
        painter.drawEllipse(QtCore.QPointF(0, 0), self.mm_to_pixel * 3500, self.mm_to_pixel * 3500)

        painter.setPen(QtCore.Qt.GlobalColor.white)
        painter.drawEllipse(QtCore.QPointF(0, 0), self.mm_to_pixel * 1000, self.mm_to_pixel * 1000)
        painter.drawEllipse(QtCore.QPointF(0, 0), self.mm_to_pixel * 2000, self.mm_to_pixel * 2000)
        painter.drawEllipse(QtCore.QPointF(0, 0), self.mm_to_pixel * 3000, self.mm_to_pixel * 3000)


        # draw center
        painter.setBrush(QtCore.Qt.GlobalColor.green)
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.drawEllipse(QtCore.QPoint(0, 0), 5, 5)

        # paint points
        painter.setBrush(QtCore.Qt.GlobalColor.yellow)
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        for angle, distance, quality in self.data:
            if quality != 0 and distance != 0:
                pos = QtCore.QPointF(self.mm_to_pixel * distance * math.cos(-angle), self.mm_to_pixel * distance * math.sin(-angle))
                size = 5
                c = self.color_from_quality(quality)
                painter.setBrush(c)
                painter.drawEllipse(pos, size, size)
        
        pen = QtGui.QPen(QtGui.QColor(255, 0, 255), 2)
        painter.setPen(pen)
        painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        for x, y, size in self.amalgame_data:
            if size < 200:
                pos = QtCore.QPointF(self.mm_to_pixel * x, -self.mm_to_pixel * y)
                size *= self.mm_to_pixel
                painter.drawEllipse(pos, size/2, size/2)

        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        
        font = QtGui.QFont()
        font.setPointSize(20)  # Augmente la taille ici si besoin
        if self.show_odom_prediction:
            pen = QtGui.QPen(QtGui.QColor(ODOM_COLOR))
            pen.setWidth(4)
            painter.setPen(pen)
            # Position estimée des balises à partir de l'odométrie
            for index,x,y in self.balise_odom_data:
                pos = QtCore.QPointF(self.mm_to_pixel * x, -self.mm_to_pixel * y)
                size = 8
                self.draw_odom_prediction_zone(painter, pos, QtGui.QColor(ODOM_COLOR))
                painter.drawLine(
                    QtCore.QPointF(pos.x() - size, pos.y()),
                    QtCore.QPointF(pos.x() + size, pos.y())
                )
                painter.drawLine(
                    QtCore.QPointF(pos.x(), pos.y() - size),
                    QtCore.QPointF(pos.x(), pos.y() + size)
                )
                label_pos = self.odom_label_position(pos)
                self.draw_upright_text(
                    painter,
                    label_pos,
                    str(index),
                    QtGui.QColor(ODOM_COLOR),
                    font,
                    QtCore.QPointF(0, 0)
                )

        if self.show_found_beacons:
            pen = QtGui.QPen(QtGui.QColor("red"))
            pen.setWidth(4)
            painter.setPen(pen)
            # Position estimée des balises par les moindres carrés
            for index,x,y in self.nearodom_data:
                pos = QtCore.QPointF(self.mm_to_pixel * x, -self.mm_to_pixel * y)
                size = 8
                painter.drawLine(
                    QtCore.QPointF(pos.x() - size, pos.y()),
                    QtCore.QPointF(pos.x() + size, pos.y())
                )
                painter.drawLine(
                    QtCore.QPointF(pos.x(), pos.y() - size),
                    QtCore.QPointF(pos.x(), pos.y() + size)
                )
                self.draw_upright_text(
                    painter,
                    pos,
                    str(index),
                    QtGui.QColor("red"),
                    font,
                    QtCore.QPointF(10, -10)
                )




    def sizeHint(self) -> QtCore.QSize:
        return QtCore.QSize(400, 400)
