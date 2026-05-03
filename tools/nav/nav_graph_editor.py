#!/usr/bin/env python3
"""PyQt6 editor for the 2026 navigation graph."""

from __future__ import annotations

import math
import sys
from collections import OrderedDict
from dataclasses import dataclass, field
from pathlib import Path

from PyQt6.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt6.QtGui import QAction, QColor, QIcon, QKeySequence, QPainter, QPainterPath, QPen, QPixmap
from PyQt6.QtWidgets import (
    QApplication,
    QAbstractItemView,
    QCheckBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGraphicsEllipseItem,
    QGraphicsItem,
    QGraphicsLineItem,
    QGraphicsPixmapItem,
    QGraphicsScene,
    QGraphicsSimpleTextItem,
    QGraphicsView,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSplitter,
    QSpinBox,
    QToolBar,
    QVBoxLayout,
    QWidget,
)


TABLE_WIDTH_MM = 3000
TABLE_HEIGHT_MM = 2000
SCENE_MARGIN_MM = 250
DEFAULT_ROBOT_DIAMETER_MM = 300
ROOT_DIR = Path(__file__).resolve().parents[2]
DEFAULT_GRAPH_PATH = ROOT_DIR / "sw" / "nav" / "graph2026.txt"
DEFAULT_TABLE_PATH = Path(__file__).resolve().parent / "table2026.png"
APP_ICON_PATH = Path(__file__).resolve().parent / "icon.png"


def graph_to_scene_y(y: float) -> float:
    return TABLE_HEIGHT_MM - y


def scene_to_graph_y(y: float) -> float:
    return TABLE_HEIGHT_MM - y


@dataclass
class Node:
    name: str
    x: float
    y: float
    neighbors: list[str] = field(default_factory=list)


class GraphModel:
    def __init__(self) -> None:
        self.nodes: OrderedDict[str, Node] = OrderedDict()

    @classmethod
    def load(cls, path: Path) -> "GraphModel":
        model = cls()
        with path.open("r", encoding="utf-8") as graph_file:
            for line_number, line in enumerate(graph_file, start=1):
                stripped = line.strip()
                if not stripped:
                    continue
                parts = stripped.split()
                if len(parts) < 4:
                    raise ValueError(f"{path}:{line_number}: expected 'name x y neighbors'")
                name = parts[0]
                if name in model.nodes:
                    raise ValueError(f"{path}:{line_number}: duplicate node '{name}'")
                neighbors = [n for n in parts[3].split(",") if n]
                model.nodes[name] = Node(name, float(parts[1]), float(parts[2]), neighbors)
        model.remove_unknown_neighbors()
        return model

    def save(self, path: Path) -> None:
        with path.open("w", encoding="utf-8") as graph_file:
            for node in self.nodes.values():
                neighbors = ",".join(node.neighbors)
                graph_file.write(f"{node.name} {node.x:g} {node.y:g} {neighbors}\n")

    def snapshot(self) -> OrderedDict[str, Node]:
        return OrderedDict(
            (name, Node(node.name, node.x, node.y, list(node.neighbors)))
            for name, node in self.nodes.items()
        )

    def restore_snapshot(self, snapshot: OrderedDict[str, Node]) -> None:
        self.nodes = OrderedDict(
            (name, Node(node.name, node.x, node.y, list(node.neighbors)))
            for name, node in snapshot.items()
        )

    def remove_unknown_neighbors(self) -> None:
        names = set(self.nodes)
        for node in self.nodes.values():
            node.neighbors = [n for n in dict.fromkeys(node.neighbors) if n in names and n != node.name]

    def add_node(self, name: str, x: float, y: float) -> None:
        self.validate_node_name(name)
        if name in self.nodes:
            raise ValueError(f"Point '{name}' already exists")
        self.nodes[name] = Node(name, x, y)

    def delete_node(self, name: str) -> None:
        self.nodes.pop(name, None)
        for node in self.nodes.values():
            node.neighbors = [n for n in node.neighbors if n != name]

    def rename_node(self, old_name: str, new_name: str) -> None:
        if old_name == new_name:
            return
        self.validate_node_name(new_name)
        if new_name in self.nodes:
            raise ValueError(f"Point '{new_name}' already exists")
        renamed: OrderedDict[str, Node] = OrderedDict()
        for name, node in self.nodes.items():
            if name == old_name:
                node.name = new_name
                renamed[new_name] = node
            else:
                renamed[name] = node
        self.nodes = renamed
        for node in self.nodes.values():
            node.neighbors = [new_name if n == old_name else n for n in node.neighbors]

    def set_edge(self, a: str, b: str, enabled: bool) -> None:
        if a == b or a not in self.nodes or b not in self.nodes:
            return
        self._set_directed_edge(a, b, enabled)
        self._set_directed_edge(b, a, enabled)

    def delete_edge(self, edge: tuple[str, str]) -> None:
        self.set_edge(edge[0], edge[1], False)

    def edge_distance(self, a: str, b: str) -> float:
        node_a = self.nodes[a]
        node_b = self.nodes[b]
        return math.hypot(node_a.x - node_b.x, node_a.y - node_b.y)

    def unique_edges(self) -> list[tuple[str, str]]:
        edges: list[tuple[str, str]] = []
        seen: set[frozenset[str]] = set()
        for name, node in self.nodes.items():
            for neighbor in node.neighbors:
                key = frozenset((name, neighbor))
                if len(key) == 2 and key not in seen and neighbor in self.nodes:
                    seen.add(key)
                    edges.append((name, neighbor))
        return edges

    def next_default_name(self) -> str:
        index = 1
        while f"Point{index}" in self.nodes:
            index += 1
        return f"Point{index}"

    @staticmethod
    def validate_node_name(name: str) -> None:
        if not name:
            raise ValueError("Point name cannot be empty")
        if any(character.isspace() for character in name) or "," in name:
            raise ValueError("Point name cannot contain spaces or commas")

    def _set_directed_edge(self, a: str, b: str, enabled: bool) -> None:
        neighbors = self.nodes[a].neighbors
        if enabled and b not in neighbors:
            neighbors.append(b)
        elif not enabled:
            self.nodes[a].neighbors = [n for n in neighbors if n != b]


class NodeItem(QGraphicsEllipseItem):
    def __init__(self, node_name: str, editor: "MapScene") -> None:
        super().__init__(-12, -12, 24, 24)
        self.node_name = node_name
        self.editor = editor
        self.setBrush(QColor(240, 80, 52))
        self.setPen(QPen(QColor(80, 20, 16), 2))
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges)
        self.setZValue(10)
        self.label = QGraphicsSimpleTextItem(node_name, self)
        self.label.setBrush(QColor(20, 20, 20))
        self.label.setPos(14, -28)
        self._apply_selection_style(False)
        self._drag_start_pos: QPointF | None = None

    def set_name(self, name: str) -> None:
        self.node_name = name
        self.label.setText(name)

    def shape(self) -> QPainterPath:
        path = QPainterPath()
        path.addEllipse(QPointF(0, 0), 26, 26)
        return path

    def boundingRect(self) -> QRectF:
        return QRectF(-26, -26, 52, 52)

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            if self.editor.node_was_clicked(self.node_name):
                event.accept()
                return
            self._drag_start_pos = self.pos()
            self.editor.node_drag_started(self.node_name)
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        super().mouseReleaseEvent(event)
        if event.button() == Qt.MouseButton.LeftButton and self._drag_start_pos is not None:
            self.editor.node_drag_finished(self.node_name, self._drag_start_pos, self.pos())
            self._drag_start_pos = None

    def itemChange(self, change: QGraphicsItem.GraphicsItemChange, value):
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionChange:
            point = value
            return QPointF(
                min(max(point.x(), 0), TABLE_WIDTH_MM),
                min(max(point.y(), 0), TABLE_HEIGHT_MM),
            )
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged:
            self.editor.node_was_moved(self.node_name, self.pos())
        if change == QGraphicsItem.GraphicsItemChange.ItemSelectedHasChanged and value:
            self.editor.node_was_selected(self.node_name)
        if change == QGraphicsItem.GraphicsItemChange.ItemSelectedHasChanged:
            self._apply_selection_style(bool(value))
        return super().itemChange(change, value)

    def _apply_selection_style(self, selected: bool) -> None:
        if selected:
            self.setRect(-18, -18, 36, 36)
            self.setBrush(QColor(255, 216, 64))
            self.setPen(QPen(QColor(10, 95, 82), 4))
            self.label.setBrush(QColor(0, 70, 60))
            self.label.setPos(20, -34)
        else:
            self.setRect(-12, -12, 24, 24)
            self.setBrush(QColor(240, 80, 52))
            self.setPen(QPen(QColor(80, 20, 16), 2))
            self.label.setBrush(QColor(20, 20, 20))
            self.label.setPos(14, -28)


class MapScene(QGraphicsScene):
    nodeMoved = pyqtSignal(str, float, float)
    nodeSelected = pyqtSignal(str)
    nodeClicked = pyqtSignal(str)
    nodeDragStarted = pyqtSignal(str)
    nodeDragFinished = pyqtSignal(str, float, float, float, float)
    addPointRequested = pyqtSignal(float, float)
    mapClicked = pyqtSignal(float, float)

    def __init__(self) -> None:
        super().__init__(
            -SCENE_MARGIN_MM,
            -SCENE_MARGIN_MM,
            TABLE_WIDTH_MM + 2 * SCENE_MARGIN_MM,
            TABLE_HEIGHT_MM + 2 * SCENE_MARGIN_MM,
        )
        self.add_mode = False
        self.create_path_mode = False

    def mousePressEvent(self, event) -> None:
        if self.add_mode and event.button() == Qt.MouseButton.LeftButton:
            point = event.scenePos()
            self.add_mode = False
            self.addPointRequested.emit(point.x(), point.y())
            event.accept()
            return
        if event.button() == Qt.MouseButton.LeftButton:
            clicked_items = self.items(event.scenePos())
            hit_node = any(
                isinstance(item, NodeItem) or isinstance(item.parentItem(), NodeItem)
                for item in clicked_items
            )
            if not hit_node:
                point = event.scenePos()
                self.mapClicked.emit(point.x(), point.y())
                event.accept()
                return
        super().mousePressEvent(event)

    def node_was_moved(self, node_name: str, point: QPointF) -> None:
        self.nodeMoved.emit(node_name, point.x(), point.y())

    def node_was_selected(self, node_name: str) -> None:
        self.nodeSelected.emit(node_name)

    def node_was_clicked(self, node_name: str) -> bool:
        self.nodeClicked.emit(node_name)
        return self.create_path_mode

    def node_drag_started(self, node_name: str) -> None:
        self.nodeDragStarted.emit(node_name)

    def node_drag_finished(self, node_name: str, start: QPointF, end: QPointF) -> None:
        self.nodeDragFinished.emit(node_name, start.x(), start.y(), end.x(), end.y())


class MapView(QGraphicsView):
    def __init__(self, scene: QGraphicsScene) -> None:
        super().__init__(scene)
        self._panning = False
        self._last_pan_pos = None
        self._pan_button: Qt.MouseButton | None = None
        self._pan_start_pos = None
        self._pan_start_scene_pos: QPointF | None = None
        self._pan_moved = False
        self.is_pan_target = lambda _point: False
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setBackgroundBrush(QColor(165, 165, 165))
        self.setDragMode(QGraphicsView.DragMode.RubberBandDrag)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            scene = self.scene()
            scene_point = self.mapToScene(event.position().toPoint())
            if not getattr(scene, "add_mode", False) and not self.is_pan_target(scene_point):
                self._start_pan(event.button(), event.position().toPoint(), scene_point)
                event.accept()
                return
        if event.button() in (Qt.MouseButton.MiddleButton, Qt.MouseButton.RightButton):
            self._start_pan(event.button(), event.position().toPoint(), None)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if self._panning and self._last_pan_pos is not None:
            current_pos = event.position().toPoint()
            delta = current_pos - self._last_pan_pos
            self._last_pan_pos = current_pos
            if self._pan_start_pos is not None and (current_pos - self._pan_start_pos).manhattanLength() > 3:
                self._pan_moved = True
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - delta.x())
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - delta.y())
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        if self._panning and event.button() == self._pan_button:
            scene_point = self._pan_start_scene_pos
            was_left_click = self._pan_button == Qt.MouseButton.LeftButton and not self._pan_moved
            self._stop_pan()
            if was_left_click and scene_point is not None:
                scene = self.scene()
                if isinstance(scene, MapScene):
                    scene.mapClicked.emit(scene_point.x(), scene_point.y())
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event) -> None:
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        self.scale(factor, factor)

    def _start_pan(self, button: Qt.MouseButton, pos, scene_pos: QPointF | None) -> None:
        self._panning = True
        self._pan_button = button
        self._last_pan_pos = pos
        self._pan_start_pos = pos
        self._pan_start_scene_pos = scene_pos
        self._pan_moved = False
        self.setCursor(Qt.CursorShape.ClosedHandCursor)

    def _stop_pan(self) -> None:
        self._panning = False
        self._last_pan_pos = None
        self._pan_button = None
        self._pan_start_pos = None
        self._pan_start_scene_pos = None
        self._pan_moved = False
        self.unsetCursor()


class Inspector(QWidget):
    nameChanged = pyqtSignal(str)
    positionChanged = pyqtSignal(float, float)
    neighborChanged = pyqtSignal(str, bool)
    deleteRequested = pyqtSignal()

    def __init__(self) -> None:
        super().__init__()
        self._updating = False
        self.current_name: str | None = None
        layout = QVBoxLayout(self)

        self.title = QLabel("No point selected")
        self.title.setStyleSheet("font-weight: 600;")
        layout.addWidget(self.title)

        form = QFormLayout()
        self.name_button = QPushButton("Rename")
        self.name_button.clicked.connect(self._rename)
        self.x_spin = self._make_coord_spin(TABLE_WIDTH_MM)
        self.y_spin = self._make_coord_spin(TABLE_HEIGHT_MM)
        self.x_spin.valueChanged.connect(self._position_changed)
        self.y_spin.valueChanged.connect(self._position_changed)
        form.addRow("Point", self.name_button)
        position_layout = QHBoxLayout()
        position_layout.addWidget(QLabel("X"))
        position_layout.addWidget(self.x_spin)
        position_layout.addWidget(QLabel("Y"))
        position_layout.addWidget(self.y_spin)
        form.addRow("Position (mm)", position_layout)
        layout.addLayout(form)

        self.distance_label = QLabel("Edge weights use geometric distance.")
        self.distance_label.setWordWrap(True)
        layout.addWidget(self.distance_label)

        layout.addWidget(QLabel("Neighbors"))
        self.neighbor_area = QScrollArea()
        self.neighbor_area.setWidgetResizable(True)
        self.neighbor_container = QWidget()
        self.neighbor_layout = QVBoxLayout(self.neighbor_container)
        self.neighbor_layout.addStretch()
        self.neighbor_area.setWidget(self.neighbor_container)
        layout.addWidget(self.neighbor_area, 1)

        self.delete_button = QPushButton("Delete Point")
        self.delete_button.clicked.connect(self.deleteRequested.emit)
        layout.addWidget(self.delete_button)
        self.set_enabled(False)

    def set_enabled(self, enabled: bool) -> None:
        self.name_button.setEnabled(enabled)
        self.x_spin.setEnabled(enabled)
        self.y_spin.setEnabled(enabled)
        self.delete_button.setEnabled(enabled)
        self.neighbor_area.setEnabled(enabled)

    def set_point(self, model: GraphModel, name: str | None) -> None:
        self._updating = True
        self.current_name = name
        self._clear_neighbors()
        if name is None or name not in model.nodes:
            self.title.setText("No point selected")
            self.set_enabled(False)
            self._updating = False
            return
        node = model.nodes[name]
        self.title.setText(name)
        self.name_button.setText(name)
        self.x_spin.setValue(round(node.x))
        self.y_spin.setValue(round(node.y))
        for other_name in model.nodes:
            if other_name == name:
                continue
            distance = model.edge_distance(name, other_name)
            checkbox = QCheckBox(f"{other_name} ({distance:.0f} mm)")
            checkbox.setChecked(other_name in node.neighbors)
            checkbox.toggled.connect(
                lambda checked, neighbor=other_name: self.neighborChanged.emit(neighbor, checked)
            )
            self.neighbor_layout.insertWidget(self.neighbor_layout.count() - 1, checkbox)
        self.set_enabled(True)
        self._updating = False

    def _make_coord_spin(self, maximum: int) -> QSpinBox:
        spin = QSpinBox()
        spin.setRange(0, maximum)
        spin.setSingleStep(10)
        return spin

    def _position_changed(self) -> None:
        if not self._updating:
            self.positionChanged.emit(self.x_spin.value(), self.y_spin.value())

    def _rename(self) -> None:
        if not self.current_name:
            return
        new_name, accepted = QInputDialog.getText(self, "Rename point", "Point name:", text=self.current_name)
        if accepted:
            self.nameChanged.emit(new_name.strip())

    def _clear_neighbors(self) -> None:
        while self.neighbor_layout.count() > 1:
            item = self.neighbor_layout.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()


class MainWindow(QMainWindow):
    def __init__(self, graph_path: Path = DEFAULT_GRAPH_PATH, table_path: Path = DEFAULT_TABLE_PATH) -> None:
        super().__init__()
        self.graph_path = graph_path
        self.table_path = table_path
        self.model = GraphModel.load(graph_path)
        self.selected_name: str | None = None
        self.selected_edge: tuple[str, str] | None = None
        self._last_edge_click: tuple[float, float] | None = None
        self._last_edge_candidates: list[tuple[str, str]] = []
        self._last_edge_candidate_index = -1
        self._undo_stack: list[OrderedDict[str, Node]] = []
        self._pending_drag_snapshot: OrderedDict[str, Node] | None = None
        self._path_start_name: str | None = None
        self.node_items: dict[str, NodeItem] = {}
        self.edge_items: list[QGraphicsItem] = []
        self._refreshing = False

        self.setWindowTitle("Navigation Graph Editor")
        if APP_ICON_PATH.exists():
            icon = QIcon(str(APP_ICON_PATH))
            self.setWindowIcon(icon)
            app = QApplication.instance()
            if app:
                app.setWindowIcon(icon)
        self.resize(1400, 900)

        self.scene = MapScene()
        self.scene.nodeMoved.connect(self._node_moved_on_map)
        self.scene.nodeSelected.connect(self.select_node)
        self.scene.nodeClicked.connect(self._node_clicked_on_map)
        self.scene.nodeDragStarted.connect(self._node_drag_started)
        self.scene.nodeDragFinished.connect(self._node_drag_finished)
        self.scene.addPointRequested.connect(self._add_point_at)
        self.scene.mapClicked.connect(self._map_clicked)
        self.view = MapView(self.scene)
        self.view.is_pan_target = self._is_map_target_at
        self.inspector = Inspector()
        self.inspector.positionChanged.connect(self._position_changed_in_inspector)
        self.inspector.nameChanged.connect(self._rename_selected)
        self.inspector.neighborChanged.connect(self._neighbor_changed)
        self.inspector.deleteRequested.connect(self._delete_selected)
        self.node_list = QListWidget()
        self.node_list.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.node_list.currentTextChanged.connect(self.select_node)

        side_panel = QWidget()
        side_layout = QVBoxLayout(side_panel)
        side_layout.addWidget(QLabel("Points"))
        side_layout.addWidget(self.node_list, 1)
        side_layout.addWidget(self.inspector, 2)

        splitter = QSplitter()
        splitter.addWidget(self.view)
        splitter.addWidget(side_panel)
        splitter.setStretchFactor(0, 5)
        splitter.setStretchFactor(1, 1)
        self.setCentralWidget(splitter)

        self.robot_spin = QDoubleSpinBox()
        self.robot_spin.setRange(50, 1000)
        self.robot_spin.setDecimals(0)
        self.robot_spin.setSingleStep(25)
        self.robot_spin.setValue(DEFAULT_ROBOT_DIAMETER_MM)
        self.robot_spin.setSuffix(" mm")
        self.robot_spin.valueChanged.connect(self.refresh_edges)
        self.undo_action: QAction | None = None
        self.add_path_action: QAction | None = None
        self.delete_path_action: QAction | None = None
        self.file_label: QLabel | None = None
        self._build_toolbar()
        self._build_status_bar()
        self.refresh_all()
        self.view.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    def _build_toolbar(self) -> None:
        toolbar = QToolBar("Tools")
        self.addToolBar(toolbar)

        open_action = QAction("Open", self)
        open_action.setShortcut(QKeySequence.StandardKey.Open)
        open_action.triggered.connect(self._open)
        toolbar.addAction(open_action)

        save_action = QAction("Save", self)
        save_action.setShortcut(QKeySequence.StandardKey.Save)
        save_action.triggered.connect(self._save)
        toolbar.addAction(save_action)

        save_as_action = QAction("Save As", self)
        save_as_action.triggered.connect(self._save_as)
        toolbar.addAction(save_as_action)

        self.undo_action = QAction("Undo", self)
        self.undo_action.setShortcut(QKeySequence.StandardKey.Undo)
        self.undo_action.setEnabled(False)
        self.undo_action.triggered.connect(self._undo)
        toolbar.addAction(self.undo_action)

        add_action = QAction("Add Point", self)
        add_action.triggered.connect(self._enable_add_mode)
        toolbar.addAction(add_action)

        self.add_path_action = QAction("Add Path", self)
        self.add_path_action.setCheckable(True)
        self.add_path_action.toggled.connect(self._set_create_path_mode)
        toolbar.addAction(self.add_path_action)

        self.delete_path_action = QAction("Delete Path", self)
        self.delete_path_action.setShortcut(QKeySequence.StandardKey.Delete)
        self.delete_path_action.setEnabled(False)
        self.delete_path_action.triggered.connect(self._delete_selected_path)
        toolbar.addAction(self.delete_path_action)

        toolbar.addSeparator()
        toolbar.addWidget(QLabel("Robot diameter "))
        toolbar.addWidget(self.robot_spin)
        self._update_file_label()

    def _build_status_bar(self) -> None:
        self.file_label = QLabel()
        self.file_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.statusBar().addPermanentWidget(self.file_label, 1)
        self._update_file_label()

    def refresh_all(self) -> None:
        self._refreshing = True
        self.scene.clear()
        self.edge_items = []
        self.node_items = {}
        if self.table_path.exists():
            pixmap_item = QGraphicsPixmapItem(QPixmap(str(self.table_path)))
            pixmap_item.setZValue(-20)
            self.scene.addItem(pixmap_item)
        self.refresh_edges()
        for node in self.model.nodes.values():
            item = NodeItem(node.name, self.scene)
            item.setPos(node.x, graph_to_scene_y(node.y))
            self.scene.addItem(item)
            self.node_items[node.name] = item
        self._refresh_node_list()
        self._refreshing = False
        self.inspector.set_point(self.model, self.selected_name)
        self._update_create_path_status()

    def refresh_edges(self) -> None:
        for item in self.edge_items:
            self.scene.removeItem(item)
        self.edge_items = []
        robot_pen = QPen(QColor(40, 150, 220, 70), self.robot_spin.value())
        robot_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        center_pen = QPen(QColor(20, 20, 20, 190), 3)
        selected_robot_pen = QPen(QColor(255, 195, 0, 130), self.robot_spin.value())
        selected_robot_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        selected_center_pen = QPen(QColor(0, 95, 82), 8)
        selected_center_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        for a, b in self.model.unique_edges():
            node_a = self.model.nodes[a]
            node_b = self.model.nodes[b]
            is_selected = self._same_edge(self.selected_edge, (a, b))
            y_a = graph_to_scene_y(node_a.y)
            y_b = graph_to_scene_y(node_b.y)
            wide = QGraphicsLineItem(node_a.x, y_a, node_b.x, y_b)
            wide.setPen(selected_robot_pen if is_selected else robot_pen)
            wide.setZValue(-3 if is_selected else -5)
            center = QGraphicsLineItem(node_a.x, y_a, node_b.x, y_b)
            center.setPen(selected_center_pen if is_selected else center_pen)
            center.setZValue(-2 if is_selected else -4)
            self.scene.addItem(wide)
            self.scene.addItem(center)
            self.edge_items.extend((wide, center))

    def select_node(self, name: str) -> None:
        if self._refreshing or not name or name not in self.model.nodes:
            return
        self.selected_name = name
        self.selected_edge = None
        self._reset_edge_click_cycle()
        self._update_delete_path_action()
        self.refresh_edges()
        self._refreshing = True
        matching_items = self.node_list.findItems(name, Qt.MatchFlag.MatchExactly)
        if matching_items:
            self.node_list.setCurrentItem(matching_items[0])
        for item_name, item in self.node_items.items():
            item.setSelected(item_name == name)
        self._refreshing = False
        self.inspector.set_point(self.model, name)

    def _refresh_node_list(self) -> None:
        self.node_list.blockSignals(True)
        self.node_list.clear()
        for name in self.model.nodes:
            self.node_list.addItem(QListWidgetItem(name))
        self.node_list.blockSignals(False)
        if self.selected_name in self.model.nodes:
            matches = self.node_list.findItems(self.selected_name, Qt.MatchFlag.MatchExactly)
            if matches:
                self.node_list.setCurrentItem(matches[0])

    def _node_moved_on_map(self, name: str, x: float, y: float) -> None:
        if self._refreshing or name not in self.model.nodes:
            return
        x = round(x)
        scene_y = round(y)
        graph_y = round(scene_to_graph_y(scene_y))
        self.model.nodes[name].x = x
        self.model.nodes[name].y = graph_y
        item = self.node_items[name]
        if item.pos().x() != x or item.pos().y() != scene_y:
            self._refreshing = True
            item.setPos(x, scene_y)
            self._refreshing = False
        self.refresh_edges()
        if name == self.selected_name:
            self.inspector.set_point(self.model, name)

    def _position_changed_in_inspector(self, x: float, y: float) -> None:
        if not self.selected_name or self.selected_name not in self.model.nodes:
            return
        node = self.model.nodes[self.selected_name]
        if node.x == x and node.y == y:
            return
        self._push_undo()
        node.x = x
        node.y = y
        item = self.node_items[self.selected_name]
        self._refreshing = True
        item.setPos(x, graph_to_scene_y(y))
        self._refreshing = False
        self.refresh_edges()

    def _neighbor_changed(self, neighbor: str, enabled: bool) -> None:
        if not self.selected_name:
            return
        is_enabled = neighbor in self.model.nodes[self.selected_name].neighbors
        if is_enabled == enabled:
            return
        self._push_undo()
        self.model.set_edge(self.selected_name, neighbor, enabled)
        changed_edge = (self.selected_name, neighbor)
        if self._same_edge(self.selected_edge, changed_edge) and not enabled:
            self.selected_edge = None
        self.refresh_edges()
        self._update_delete_path_action()
        self.inspector.set_point(self.model, self.selected_name)

    def _rename_selected(self, new_name: str) -> None:
        if not self.selected_name or not new_name:
            return
        old_name = self.selected_name
        if old_name == new_name:
            return
        try:
            self._push_undo()
            self.model.rename_node(old_name, new_name)
        except ValueError as exc:
            self._pop_empty_undo()
            QMessageBox.warning(self, "Rename failed", str(exc))
            return
        item = self.node_items.pop(old_name)
        item.set_name(new_name)
        self.node_items[new_name] = item
        self.selected_name = new_name
        self.refresh_all()
        self.select_node(new_name)

    def _delete_selected(self) -> None:
        if not self.selected_name:
            return
        name = self.selected_name
        answer = QMessageBox.question(self, "Delete point", f"Delete point '{name}'?")
        if answer != QMessageBox.StandardButton.Yes:
            return
        self._push_undo()
        self.model.delete_node(name)
        self.selected_name = None
        self.selected_edge = None
        self._update_delete_path_action()
        self.refresh_all()

    def _enable_add_mode(self) -> None:
        self._set_create_path_mode(False)
        self.scene.add_mode = True
        self.statusBar().showMessage("Click the map to place the new point.", 5000)

    def _add_point_at(self, x: float, y: float) -> None:
        default_name = self.model.next_default_name()
        name, accepted = QInputDialog.getText(self, "Add point", "Point name:", text=default_name)
        if not accepted:
            return
        name = name.strip()
        if not name:
            return
        try:
            self._push_undo()
            self.model.add_node(name, round(x), round(scene_to_graph_y(y)))
        except ValueError as exc:
            self._pop_empty_undo()
            QMessageBox.warning(self, "Add point failed", str(exc))
            return
        self.selected_name = name
        self.refresh_all()
        self.select_node(name)

    def _node_clicked_on_map(self, name: str) -> None:
        if not self.scene.create_path_mode:
            return
        if name not in self.model.nodes:
            return
        if self._path_start_name is None:
            self._path_start_name = name
            self.select_node(name)
            self.statusBar().showMessage("Click the second point for the new path.", 5000)
            return
        if self._path_start_name == name:
            self.statusBar().showMessage("Click a different point to create a path.", 3000)
            return
        already_connected = name in self.model.nodes[self._path_start_name].neighbors
        if not already_connected:
            self._push_undo()
            self.model.set_edge(self._path_start_name, name, True)
        self.selected_edge = (self._path_start_name, name)
        self._clear_point_selection()
        self._set_create_path_mode(False)
        self._update_delete_path_action()
        self.refresh_edges()
        self.statusBar().showMessage(f"Created path {self.selected_edge[0]} - {self.selected_edge[1]}", 3000)

    def _node_drag_started(self, name: str) -> None:
        if self._refreshing or name not in self.model.nodes:
            return
        self._pending_drag_snapshot = self.model.snapshot()

    def _node_drag_finished(self, name: str, start_x: float, start_y: float, end_x: float, end_y: float) -> None:
        if self._pending_drag_snapshot is None:
            return
        moved = round(start_x) != round(end_x) or round(start_y) != round(end_y)
        if moved:
            self._undo_stack.append(self._pending_drag_snapshot)
            self._update_undo_action()
        self._pending_drag_snapshot = None

    def _map_clicked(self, x: float, y: float) -> None:
        if self.scene.create_path_mode:
            self._path_start_name = None
            self._clear_selection()
            self._update_create_path_status()
            return
        candidates = self._edge_candidates_at(x, y)
        if not candidates:
            self._clear_selection()
            return

        self._clear_point_selection()
        candidate_edges = [edge for edge, _distance in candidates]
        same_click_area = (
            self._last_edge_click is not None
            and math.hypot(x - self._last_edge_click[0], y - self._last_edge_click[1]) <= 20
            and candidate_edges == self._last_edge_candidates
        )
        if same_click_area:
            self._last_edge_candidate_index = (self._last_edge_candidate_index + 1) % len(candidate_edges)
        else:
            self._last_edge_click = (x, y)
            self._last_edge_candidates = candidate_edges
            self._last_edge_candidate_index = 0

        self.selected_edge = candidate_edges[self._last_edge_candidate_index]
        self._update_delete_path_action()
        self.refresh_edges()
        self.statusBar().showMessage(f"Selected path {self.selected_edge[0]} - {self.selected_edge[1]}", 3000)

    def _clear_selection(self) -> None:
        self._clear_point_selection()
        self.selected_edge = None
        self._reset_edge_click_cycle()
        self._update_delete_path_action()
        self.refresh_edges()

    def _clear_point_selection(self) -> None:
        self.selected_name = None
        self._refreshing = True
        self.node_list.clearSelection()
        self.node_list.setCurrentRow(-1)
        for item in self.node_items.values():
            item.setSelected(False)
        self._refreshing = False
        self.inspector.set_point(self.model, None)

    def _reset_edge_click_cycle(self) -> None:
        self._last_edge_click = None
        self._last_edge_candidates = []
        self._last_edge_candidate_index = -1

    def _edge_candidates_at(self, x: float, y: float) -> list[tuple[tuple[str, str], float]]:
        tolerance = max(24.0, self.robot_spin.value() / 2)
        candidates: list[tuple[tuple[str, str], float]] = []
        for edge in self.model.unique_edges():
            a, b = edge
            node_a = self.model.nodes[a]
            node_b = self.model.nodes[b]
            distance = self._distance_to_segment(
                x,
                y,
                node_a.x,
                graph_to_scene_y(node_a.y),
                node_b.x,
                graph_to_scene_y(node_b.y),
            )
            if distance <= tolerance:
                candidates.append((edge, distance))
        return sorted(candidates, key=lambda candidate: candidate[1])

    def _is_map_target_at(self, point: QPointF) -> bool:
        for item in self.scene.items(point):
            if isinstance(item, NodeItem) or isinstance(item.parentItem(), NodeItem):
                return True
        return bool(self._edge_candidates_at(point.x(), point.y()))

    def _delete_selected_path(self) -> None:
        if not self.selected_edge:
            return
        a, b = self.selected_edge
        self._push_undo()
        self.model.delete_edge(self.selected_edge)
        self.selected_edge = None
        self._reset_edge_click_cycle()
        self._update_delete_path_action()
        self.refresh_edges()
        if self.selected_name:
            self.inspector.set_point(self.model, self.selected_name)
        self.statusBar().showMessage(f"Deleted path {a} - {b}", 3000)

    def _update_delete_path_action(self) -> None:
        if self.delete_path_action:
            self.delete_path_action.setEnabled(self.selected_edge is not None)

    def _set_create_path_mode(self, enabled: bool) -> None:
        self.scene.create_path_mode = enabled
        self._path_start_name = None
        if self.add_path_action and self.add_path_action.isChecked() != enabled:
            self.add_path_action.setChecked(enabled)
        if enabled:
            self.scene.add_mode = False
            self._clear_selection()
        self._update_create_path_status()

    def _update_create_path_status(self) -> None:
        if self.scene.create_path_mode:
            self.statusBar().showMessage("Click the first point for the new path.", 5000)

    def _push_undo(self) -> None:
        self._undo_stack.append(self.model.snapshot())
        self._update_undo_action()

    def _pop_empty_undo(self) -> None:
        if self._undo_stack:
            self._undo_stack.pop()
        self._update_undo_action()

    def _undo(self) -> None:
        if not self._undo_stack:
            return
        snapshot = self._undo_stack.pop()
        self.model.restore_snapshot(snapshot)
        self.selected_name = self.selected_name if self.selected_name in self.model.nodes else None
        if self.selected_edge and not all(name in self.model.nodes for name in self.selected_edge):
            self.selected_edge = None
        if self.selected_edge and not self._edge_exists(self.selected_edge):
            self.selected_edge = None
        self._pending_drag_snapshot = None
        self._set_create_path_mode(False)
        self._reset_edge_click_cycle()
        self._update_undo_action()
        self._update_delete_path_action()
        self.refresh_all()
        if self.selected_name:
            self.select_node(self.selected_name)

    def _update_undo_action(self) -> None:
        if self.undo_action:
            self.undo_action.setEnabled(bool(self._undo_stack))

    def _edge_exists(self, edge: tuple[str, str]) -> bool:
        return edge[0] in self.model.nodes and edge[1] in self.model.nodes[edge[0]].neighbors

    def _open(self) -> None:
        file_name, _ = QFileDialog.getOpenFileName(
            self,
            "Open graph",
            str(self.graph_path),
            "Graph files (*.txt);;All files (*)",
        )
        if not file_name:
            return
        path = Path(file_name)
        try:
            model = GraphModel.load(path)
        except (OSError, ValueError) as exc:
            QMessageBox.warning(self, "Open failed", str(exc))
            return

        self.graph_path = path
        self.model = model
        self.selected_name = None
        self.selected_edge = None
        self._reset_edge_click_cycle()
        self._undo_stack = []
        self._pending_drag_snapshot = None
        self._set_create_path_mode(False)
        self._update_undo_action()
        self._update_delete_path_action()
        self._update_file_label()
        self.refresh_all()
        self.statusBar().showMessage(f"Opened {self.graph_path}", 4000)

    def _update_file_label(self) -> None:
        if self.file_label:
            path = str(self.graph_path.resolve())
            self.file_label.setText(path)
            self.file_label.setToolTip(path)
        self.setWindowTitle(f"Navigation Graph Editor - {self.graph_path.name}")

    @staticmethod
    def _same_edge(edge_a: tuple[str, str] | None, edge_b: tuple[str, str] | None) -> bool:
        return edge_a is not None and edge_b is not None and frozenset(edge_a) == frozenset(edge_b)

    @staticmethod
    def _distance_to_segment(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
        dx = bx - ax
        dy = by - ay
        length_squared = dx * dx + dy * dy
        if length_squared == 0:
            return math.hypot(px - ax, py - ay)
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_squared))
        projection_x = ax + t * dx
        projection_y = ay + t * dy
        return math.hypot(px - projection_x, py - projection_y)

    def _save(self) -> None:
        self.model.save(self.graph_path)
        self._update_file_label()
        self.statusBar().showMessage(f"Saved {self.graph_path}", 4000)

    def _save_as(self) -> None:
        file_name, _ = QFileDialog.getSaveFileName(
            self,
            "Save graph",
            str(self.graph_path),
            "Graph files (*.txt);;All files (*)",
        )
        if not file_name:
            return
        self.graph_path = Path(file_name)
        self._update_file_label()
        self._save()


def main() -> int:
    app = QApplication(sys.argv)
    if APP_ICON_PATH.exists():
        app.setWindowIcon(QIcon(str(APP_ICON_PATH)))
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
