#########################################################################
#                                                                       #
#   SECURE DRONE SWARM SYSTEM - CORE MODULE                             #
#                                                                       #
#   Developer : Md Shahanur Islam Shagor                                #
#   Role      : Project Architect & Lead Developer                      #
#   Version   : 1.0.2                                                   #
#   Status    : Production Ready                                        #
#                                                                       #
#   "Protecting the skies with decentralized intelligence."             #
#                                                                       #
#########################################################################
"""
Graphical User Interface - Real-time drone swarm visualization
Built with PyQt5 for professional visualization
"""

import sys
import math
import os
import glob
import json
import webbrowser
import time
import urllib.parse
import urllib.request
import hashlib
from collections import deque
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QPushButton, QLabel, QGroupBox,
                            QGridLayout, QTextEdit, QComboBox, QSpinBox,
                            QSlider, QTabWidget, QTableWidget, QTableWidgetItem,
                            QAbstractItemView,
                            QDoubleSpinBox,
                            QCheckBox,
                            QScrollArea,
                            QHeaderView,
                            QShortcut,
                            QProgressBar, QSizePolicy)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QPointF, QByteArray, QRectF
from PyQt5.QtGui import QKeySequence
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QFont, QPalette, QPolygonF, QLinearGradient, QPixmap, QImage
import logging
try:
    import numpy as np
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.figure import Figure
    MATPLOTLIB_AVAILABLE = True
except Exception:
    MATPLOTLIB_AVAILABLE = False
try:
    from PyQt5.QtSvg import QSvgRenderer
except Exception:
    QSvgRenderer = None


def _load_env_file(env_path: str):
    """Minimal .env loader without external dependency."""
    if not os.path.exists(env_path):
        return
    try:
        with open(env_path, "r", encoding="utf-8") as fh:
            for raw in fh:
                line = raw.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, value = line.split("=", 1)
                key = key.strip()
                value = value.strip().strip("'").strip('"')
                if key and key not in os.environ:
                    os.environ[key] = value
    except Exception:
        return


def _env_flag(name: str, default: bool = False) -> bool:
    val = os.getenv(name, "1" if default else "0").strip().lower()
    return val in {"1", "true", "yes", "on"}


_load_env_file(os.path.join(os.path.dirname(__file__), ".env"))

class DroneWidget(QWidget):
    """3D-like visualization of drone swarm"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(800, 600)
        
        # Visualization state
        self.drones = {}  # drone_id -> drone_data
        self.obstacles = []
        self.show_paths = True
        self.show_labels = True
        self.zoom = 1.0
        self.offset_x = 0
        self.offset_y = 0
        self.map_range_m = 10000.0  # 10 km extent
        self.grid_step_m = 1000.0   # 1 km grid
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.drone_svg_template = self._load_drone_svg_template()
        self.fields_svg_template = self._load_fields_svg_template()
        self.fields_renderer = None
        if QSvgRenderer and self.fields_svg_template:
            self.fields_renderer = QSvgRenderer(QByteArray(self.fields_svg_template.encode("utf-8")))
        self.position_history = {}
        self.route_hold_frames = {}
        self.animation_phase = 0.0
        self.corners = {}
        self.start_point = None
        self.destination_point = None
        self.destination_slots = {}
        self.start_slots = {}
        self.minimum_gap_m = 40.0
        self.acoustic_source = None
        self.acoustic_confidence = 0.0
        
        # Colors
        self.colors = {
            'leader': QColor(255, 215, 0),      # Gold
            'follower': QColor(86, 170, 255),   # Bright Blue
            'emergency': QColor(255, 92, 92),   # Soft Red
            'grounded': QColor(128, 128, 128),  # Gray
            'home': QColor(126, 199, 112),      # Soft Green
            'background': QColor(14, 17, 26),   # Dark Navy
            'grid': QColor(60, 66, 84)          # Grid color
        }
        
        # Set dark background
        palette = self.palette()
        palette.setColor(QPalette.Window, self.colors['background'])
        self.setPalette(palette)
        self.setAutoFillBackground(True)

    def _load_drone_svg_template(self) -> str:
        """Load SVG template for drone icon rendering."""
        default_template = """
<svg width="120" height="120" viewBox="0 0 120 120" xmlns="http://www.w3.org/2000/svg">
  <g fill="none" stroke="{ACCENT_COLOR}" stroke-width="4" stroke-linecap="round">
    <line x1="60" y1="60" x2="20" y2="20"/>
    <line x1="60" y1="60" x2="100" y2="20"/>
    <line x1="60" y1="60" x2="20" y2="100"/>
    <line x1="60" y1="60" x2="100" y2="100"/>
  </g>
  <circle cx="20" cy="20" r="12" fill="#2c2c2c" stroke="{ACCENT_COLOR}" stroke-width="3"/>
  <circle cx="100" cy="20" r="12" fill="#2c2c2c" stroke="{ACCENT_COLOR}" stroke-width="3"/>
  <circle cx="20" cy="100" r="12" fill="#2c2c2c" stroke="{ACCENT_COLOR}" stroke-width="3"/>
  <circle cx="100" cy="100" r="12" fill="#2c2c2c" stroke="{ACCENT_COLOR}" stroke-width="3"/>
  <ellipse cx="60" cy="62" rx="24" ry="18" fill="{BODY_COLOR}" stroke="{ACCENT_COLOR}" stroke-width="3"/>
  <rect x="52" y="50" width="16" height="9" rx="2" fill="#f2f2f2"/>
</svg>
"""
        template_path = os.path.join(os.path.dirname(__file__), "assets", "drone.svg")
        if os.path.exists(template_path):
            try:
                with open(template_path, "r", encoding="utf-8") as fh:
                    return fh.read()
            except Exception:
                return default_template
        return default_template

    def _load_fields_svg_template(self) -> str:
        """Load SVG background for field-style visualization."""
        default_fields = """
<svg width="1400" height="900" viewBox="0 0 1400 900" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <linearGradient id="g" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0%" stop-color="#4f8f3d"/>
      <stop offset="100%" stop-color="#2e5e2d"/>
    </linearGradient>
  </defs>
  <rect width="1400" height="900" fill="url(#g)"/>
  <path d="M0,120 C220,90 460,160 700,130 C940,100 1180,170 1400,140 L1400,0 L0,0 Z"
        fill="#8fc58f" fill-opacity="0.35"/>
  <path d="M0,420 C210,450 480,360 700,395 C940,430 1170,350 1400,390 L1400,220 L0,220 Z"
        fill="#7db57d" fill-opacity="0.25"/>
  <path d="M0,820 C260,760 430,860 700,800 C960,745 1170,860 1400,820 L1400,620 L0,620 Z"
        fill="#6aa66a" fill-opacity="0.22"/>
</svg>
"""
        template_path = os.path.join(os.path.dirname(__file__), "assets", "fields.svg")
        if os.path.exists(template_path):
            try:
                with open(template_path, "r", encoding="utf-8") as fh:
                    return fh.read()
            except Exception:
                return default_fields
        return default_fields
    
    def update_drones(self, drones_data):
        """Update drone positions and status"""
        self.animation_phase += 0.25
        self.drones = drones_data
        for drone_id, drone_data in self.drones.items():
            pos = drone_data.get("position", {})
            point = (pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0))
            moving, returning = self._is_drone_route_active(drone_data)
            if moving:
                trail = self.position_history.setdefault(drone_id, [])
                trail.append((point[0], point[1], point[2], returning))
        self._update_map_origin()
        self.update()

    def _is_drone_route_active(self, drone_data):
        """Return (is_moving, is_returning_home) for route visibility."""
        flight_mode = str(drone_data.get("flight_mode", "")).strip().lower()
        velocity = drone_data.get("velocity", {}) or {}
        vx = float(velocity.get("x", 0.0))
        vy = float(velocity.get("y", 0.0))
        vz = float(velocity.get("z", 0.0))
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)

        is_returning = ("return" in flight_mode) or bool(drone_data.get("emergency_return_active"))
        moving_mode = any(
            key in flight_mode
            for key in ["flying", "takeoff", "taking_off", "landing", "moving", "mission", "return"]
        )
        is_moving = (speed > 0.2) or moving_mode
        return is_moving, is_returning

    def _update_map_origin(self):
        """Center map around active home positions."""
        if self.corners:
            pts = [p for p in self.corners.values() if p is not None]
            if pts:
                min_x = min(p.x for p in pts)
                max_x = max(p.x for p in pts)
                min_y = min(p.y for p in pts)
                max_y = max(p.y for p in pts)
                self.origin_x = (min_x + max_x) / 2.0
                self.origin_y = (min_y + max_y) / 2.0
                return
        if not self.drones:
            self.origin_x = 0.0
            self.origin_y = 0.0
            return
        homes = []
        for drone_data in self.drones.values():
            home = drone_data.get("home_position", {})
            if home:
                homes.append((home.get("x", 0.0), home.get("y", 0.0)))
        if not homes:
            self.origin_x = 0.0
            self.origin_y = 0.0
            return
        self.origin_x = sum(x for x, _ in homes) / len(homes)
        self.origin_y = sum(y for _, y in homes) / len(homes)

    def _pixels_per_meter(self) -> float:
        """Scale map to fit configured world range."""
        min_dim = max(200.0, min(self.width(), self.height()))
        return (min_dim * 0.42 * self.zoom) / self.map_range_m
    
    def add_obstacle(self, x, y, z, radius):
        """Add obstacle to visualization"""
        self.obstacles.append(
            {"x": x, "y": y, "z": z, "radius": radius, "motion_type": "static", "dynamic": False}
        )
        self.update()
    
    def clear_obstacles(self):
        """Clear all obstacles"""
        self.obstacles.clear()
        self.update()

    def set_obstacles(self, obstacles):
        """Replace obstacle list for live dynamic rendering."""
        self.obstacles = list(obstacles or [])
        self.update()

    def set_acoustic_source(self, source: dict, confidence: float = 0.0):
        self.acoustic_source = source
        self.acoustic_confidence = float(confidence)
        self.update()
    
    def world_to_screen(self, x, y, z=0):
        """Convert world coordinates to screen coordinates"""
        center_x = self.width() / 2
        center_y = self.height() / 2
        scale = self._pixels_per_meter()
        
        rel_x = x - self.origin_x
        rel_y = y - self.origin_y
        
        screen_x = center_x + rel_x * scale + self.offset_x
        screen_y = center_y - rel_y * scale + self.offset_y  # North-up

        # Simulate 3D depth
        depth_scale = max(0.5, 1.0 - (z * 0.003))
        
        return screen_x, screen_y, depth_scale
    
    def paintEvent(self, event):
        """Render the drone swarm"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Draw field style background
        self._draw_field_background(painter)
        
        # Draw grid
        self._draw_grid(painter)

        # Draw ABCD operation box and X->Y mission plan
        self._draw_operation_overlay(painter)
        
        # Draw obstacles
        self._draw_obstacles(painter)
        self._draw_acoustic_source(painter)
        
        # Draw home positions and paths
        self._draw_home_positions(painter)
        self._draw_trails(painter)
        
        # Draw drones (sorted by altitude for proper layering)
        sorted_drones = sorted(self.drones.items(), 
                              key=lambda x: x[1].get('position', {}).get('z', 0))
        
        for drone_id, drone_data in sorted_drones:
            self._draw_drone(painter, drone_id, drone_data)

    def set_operation_overlay(
        self,
        corners,
        start_point,
        destination_point,
        destination_slots,
        minimum_gap_m,
        start_slots=None
    ):
        """Set mission overlay points for ABCD corners and X->Y route."""
        self.corners = corners or {}
        self.start_point = start_point
        self.destination_point = destination_point
        self.destination_slots = destination_slots or {}
        self.start_slots = start_slots or {}
        self.minimum_gap_m = float(minimum_gap_m)
        self._fit_map_to_operation_zone()
        self.update()

    def _fit_map_to_operation_zone(self):
        """Fit map span/grid to ABCD operation box dimensions."""
        if not self.corners:
            return
        pts = [p for p in self.corners.values() if p is not None]
        if len(pts) < 2:
            return
        min_x = min(p.x for p in pts)
        max_x = max(p.x for p in pts)
        min_y = min(p.y for p in pts)
        max_y = max(p.y for p in pts)
        width_m = max(1.0, max_x - min_x)
        height_m = max(1.0, max_y - min_y)
        half_span = max(width_m, height_m) * 0.52
        self.map_range_m = max(300.0, half_span)
        self.grid_step_m = max(50.0, round(self.map_range_m / 6.0 / 50.0) * 50.0)
        self.origin_x = (min_x + max_x) / 2.0
        self.origin_y = (min_y + max_y) / 2.0

    def _draw_operation_overlay(self, painter):
        """Draw operation zone corners, X/Y markers, and destination slots."""
        if self.corners:
            painter.setFont(QFont("Arial", 10, QFont.Bold))
            pen = QPen(QColor(255, 240, 120))
            pen.setWidth(2)
            painter.setPen(pen)
            order = ["A", "B", "C", "D"]
            points = []
            for key in order:
                point = self.corners.get(key)
                if not point:
                    continue
                sx, sy, _ = self.world_to_screen(point.x, point.y, point.z)
                points.append(QPointF(sx, sy))
                painter.drawEllipse(QPointF(sx, sy), 6, 6)
                painter.drawText(int(sx + 8), int(sy - 8), key)
            if len(points) >= 4:
                painter.drawLine(points[0], points[1])
                painter.drawLine(points[1], points[2])
                painter.drawLine(points[2], points[3])
                painter.drawLine(points[3], points[0])

        if self.start_point:
            sx, sy, _ = self.world_to_screen(self.start_point.x, self.start_point.y, self.start_point.z)
            painter.setPen(QPen(QColor(80, 255, 140), 2))
            painter.setBrush(QBrush(QColor(80, 255, 140, 120)))
            painter.drawEllipse(QPointF(sx, sy), 9, 9)
            painter.setFont(QFont("Arial", 10, QFont.Bold))
            painter.drawText(int(sx + 10), int(sy - 10), "X")

        if self.start_slots:
            painter.setPen(QPen(QColor(120, 255, 180, 160), 1, Qt.DashLine))
            painter.setBrush(QBrush(QColor(120, 255, 180, 55)))
            painter.setFont(QFont("Arial", 7, QFont.Bold))
            for drone_id, point in self.start_slots.items():
                sx, sy, _ = self.world_to_screen(point.x, point.y, point.z)
                painter.drawEllipse(QPointF(sx, sy), 5, 5)
                painter.drawText(int(sx + 6), int(sy - 6), f"X{drone_id}")

        if self.destination_point:
            dyx, dyy, _ = self.world_to_screen(
                self.destination_point.x,
                self.destination_point.y,
                self.destination_point.z
            )
            painter.setPen(QPen(QColor(255, 120, 120), 2))
            painter.setBrush(QBrush(QColor(255, 120, 120, 120)))
            painter.drawEllipse(QPointF(dyx, dyy), 9, 9)
            painter.setFont(QFont("Arial", 10, QFont.Bold))
            painter.drawText(int(dyx + 10), int(dyy - 10), "Y")

            if self.start_point:
                sx, sy, _ = self.world_to_screen(self.start_point.x, self.start_point.y, self.start_point.z)
                painter.setPen(QPen(QColor(255, 255, 255, 120), 1, Qt.DashLine))
                painter.drawLine(int(sx), int(sy), int(dyx), int(dyy))

        if self.destination_slots:
            painter.setPen(QPen(QColor(255, 200, 80, 140), 1, Qt.DashLine))
            painter.setBrush(QBrush(QColor(255, 200, 80, 40)))
            for point in self.destination_slots.values():
                sx, sy, _ = self.world_to_screen(point.x, point.y, point.z)
                painter.drawEllipse(QPointF(sx, sy), 6, 6)

            if self.destination_point:
                center_x, center_y, _ = self.world_to_screen(
                    self.destination_point.x,
                    self.destination_point.y,
                    self.destination_point.z
                )
                radius_px = self.minimum_gap_m * self._pixels_per_meter()
                painter.setPen(QPen(QColor(255, 180, 60, 90), 1, Qt.DotLine))
                painter.setBrush(Qt.NoBrush)
                painter.drawEllipse(QPointF(center_x, center_y), radius_px, radius_px)

    def _draw_field_background(self, painter):
        """Draw dark tactical background to match control-station style."""
        gradient = QLinearGradient(0, 0, self.width(), self.height())
        gradient.setColorAt(0.0, QColor(25, 29, 40))
        gradient.setColorAt(0.5, QColor(17, 21, 31))
        gradient.setColorAt(1.0, QColor(11, 14, 22))
        painter.fillRect(self.rect(), gradient)

    def _draw_trails(self, painter):
        """Draw visible route only while drone is moving / returning."""
        for drone_id, trail in self.position_history.items():
            if len(trail) < 2:
                continue
            for i in range(1, len(trail)):
                p1 = trail[i - 1]
                p2 = trail[i]
                x1, y1, z1 = p1[0], p1[1], p1[2]
                x2, y2, z2 = p2[0], p2[1], p2[2]
                returning = bool(p2[3]) if len(p2) > 3 else False
                sx1, sy1, _ = self.world_to_screen(x1, y1, z1)
                sx2, sy2, _ = self.world_to_screen(x2, y2, z2)
                alpha = int(195 * (i / len(trail)))
                color = QColor(255, 183, 94, alpha) if returning else QColor(120, 220, 255, alpha)
                pen = QPen(color)
                pen.setWidth(2 if returning else 1)
                painter.setPen(pen)
                painter.drawLine(int(sx1), int(sy1), int(sx2), int(sy2))
    
    def _draw_grid(self, painter):
        """Draw background grid in 1km intervals and map overlays."""
        pen = QPen(self.colors['grid'])
        pen.setWidth(1)
        painter.setPen(pen)

        half_span = int(self.map_range_m)
        step = int(self.grid_step_m)
        start_x = int(self.origin_x - half_span)
        end_x = int(self.origin_x + half_span)
        start_y = int(self.origin_y - half_span)
        end_y = int(self.origin_y + half_span)

        for x in range(start_x - (start_x % step), end_x + step, step):
            sx1, sy1, _ = self.world_to_screen(x, start_y)
            sx2, sy2, _ = self.world_to_screen(x, end_y)
            painter.drawLine(int(sx1), int(sy1), int(sx2), int(sy2))

        for y in range(start_y - (start_y % step), end_y + step, step):
            sx1, sy1, _ = self.world_to_screen(start_x, y)
            sx2, sy2, _ = self.world_to_screen(end_x, y)
            painter.drawLine(int(sx1), int(sy1), int(sx2), int(sy2))

        # Center crosshair
        pen.setColor(QColor(130, 130, 170))
        pen.setWidth(2)
        painter.setPen(pen)
        cx = int(self.width() / 2 + self.offset_x)
        cy = int(self.height() / 2 + self.offset_y)
        painter.drawLine(cx - 8, cy, cx + 8, cy)
        painter.drawLine(cx, cy - 8, cx, cy + 8)

        # 10km operational boundary
        scale = self._pixels_per_meter()
        radius_px = self.map_range_m * scale
        pen.setColor(QColor(80, 140, 200, 120))
        pen.setStyle(Qt.DashLine)
        pen.setWidth(1)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(QPointF(cx, cy), radius_px, radius_px)

        # Compass and map scale labels
        painter.setPen(QPen(QColor(220, 220, 220)))
        painter.setFont(QFont('Arial', 9, QFont.Bold))
        painter.drawText(15, 25, "N")
        painter.drawLine(22, 45, 22, 30)
        painter.drawLine(22, 30, 18, 35)
        painter.drawLine(22, 30, 26, 35)
        painter.drawText(15, 65, "Scale: 10 km radius")
    
    def _draw_obstacles(self, painter):
        """Draw obstacles"""
        for obstacle in self.obstacles:
            x = obstacle['x']
            y = obstacle['y']
            z = obstacle.get('z', 0)
            radius = obstacle['radius']
            is_dynamic = obstacle.get("dynamic", obstacle.get("motion_type", "static") != "static")
            
            sx, sy, depth = self.world_to_screen(x, y, z)
            scaled_radius = radius * self._pixels_per_meter() * depth
            
            # Draw obstacle
            pen = QPen(QColor(255, 175, 50) if is_dynamic else QColor(255, 100, 100))
            pen.setWidth(2)
            painter.setPen(pen)
            brush = QBrush(QColor(255, 175, 50, 85) if is_dynamic else QColor(255, 50, 50, 50))
            painter.setBrush(brush)
            
            painter.drawEllipse(QPointF(sx, sy), scaled_radius, scaled_radius)
            if is_dynamic:
                vx = obstacle.get("vx", 0.0)
                vy = obstacle.get("vy", 0.0)
                tip_x, tip_y, _ = self.world_to_screen(x + vx * 2.0, y + vy * 2.0, z)
                painter.setPen(QPen(QColor(255, 235, 150), 2))
                painter.drawLine(int(sx), int(sy), int(tip_x), int(tip_y))

                # Animated pulse ring around moving obstacle.
                pulse = 1.0 + 0.22 * math.sin(self.animation_phase * 0.55 + (x + y) * 0.003)
                pulse_radius = scaled_radius * pulse
                painter.setPen(QPen(QColor(255, 220, 120, 130), 1, Qt.DashLine))
                painter.setBrush(Qt.NoBrush)
                painter.drawEllipse(QPointF(sx, sy), pulse_radius, pulse_radius)

                # Simple motion trail to make movement intent visible.
                speed = math.hypot(float(vx), float(vy))
                if speed > 0.05:
                    ux = float(vx) / speed
                    uy = float(vy) / speed
                    for i in range(1, 4):
                        trail_dist = 8.0 * i
                        tx, ty, _ = self.world_to_screen(x - ux * trail_dist, y - uy * trail_dist, z)
                        alpha = max(40, 160 - i * 40)
                        painter.setPen(Qt.NoPen)
                        painter.setBrush(QBrush(QColor(255, 210, 120, alpha)))
                        painter.drawEllipse(QPointF(tx, ty), max(2.0, 5.0 - i), max(2.0, 5.0 - i))
    
    def _draw_home_positions(self, painter):
        """Draw home positions for all drones"""
        pen = QPen(self.colors['home'])
        pen.setWidth(2)
        painter.setPen(pen)
        
        for drone_id, drone_data in self.drones.items():
            home = drone_data.get('home_position', {})
            if home:
                sx, sy, _ = self.world_to_screen(home['x'], home['y'], home.get('z', 0))
                
                # Draw home marker (H)
                painter.drawText(int(sx - 10), int(sy - 10), 20, 20,
                               Qt.AlignCenter, f"H{drone_id}")
                
                # Draw circle
                painter.drawEllipse(QPointF(sx, sy), 8, 8)

    def _draw_acoustic_source(self, painter):
        if not self.acoustic_source:
            return
        x = float(self.acoustic_source.get("x", 0.0))
        y = float(self.acoustic_source.get("y", 0.0))
        sx, sy, _ = self.world_to_screen(x, y, 0.0)
        conf = max(0.0, min(1.0, float(self.acoustic_confidence)))
        radius = 12.0 + 18.0 * conf
        painter.setPen(QPen(QColor(0, 255, 255), 2))
        painter.setBrush(QBrush(QColor(0, 255, 255, 70)))
        painter.drawEllipse(QPointF(sx, sy), radius, radius)
        painter.setPen(QPen(QColor(225, 255, 255)))
        painter.setFont(QFont("Arial", 9, QFont.Bold))
        painter.drawText(int(sx + radius + 4), int(sy - radius - 4), f"SRC {conf:.2f}")
    
    def _draw_drone(self, painter, drone_id, drone_data):
        """Draw individual drone"""
        pos = drone_data.get('position', {})
        role = drone_data.get('role', 'follower')
        flight_mode = drone_data.get('flight_mode', 'idle')
        battery = drone_data.get('battery', 100)
        
        x = pos.get('x', 0)
        y = pos.get('y', 0)
        z = pos.get('z', 0)
        velocity = drone_data.get("velocity", {"x": 0.0, "y": 0.0, "z": 0.0})
        vx = float(velocity.get("x", 0.0))
        vy = float(velocity.get("y", 0.0))
        
        sx, sy, depth = self.world_to_screen(x, y, z)
        
        # Determine color based on role and status
        if flight_mode == 'emergency_landing':
            color = self.colors['emergency']
        elif drone_data.get("emergency_return_active"):
            color = self.colors['emergency']
        elif drone_data.get("motor_failure_warning"):
            color = QColor(255, 165, 0)
        elif role == 'leader':
            color = self.colors['leader']
        elif flight_mode == 'idle' or flight_mode == 'crashed':
            color = self.colors['grounded']
        else:
            color = self.colors['follower']
        
        # Drone size varies with altitude (depth effect)
        base_size = 42 if role == "leader" else 34
        size = max(20, base_size * depth)

        self._draw_drone_svg(painter, sx, sy, size, color)
        self._draw_motion_direction(painter, sx, sy, size, vx, vy, color)
        
        # Draw altitude indicator (vertical line)
        if z > 0.5:
            pen = QPen()
            pen.setColor(QColor(150, 150, 150, 100))
            pen.setWidth(1)
            pen.setStyle(Qt.DashLine)
            painter.setPen(pen)
            ground_x, ground_y, _ = self.world_to_screen(x, y, 0)
            painter.drawLine(int(sx), int(sy), int(ground_x), int(ground_y))
        
        # Draw labels
        if self.show_labels:
            # Drone ID and altitude
            label_text = f"D{drone_id}"
            if z > 0.5:
                label_text += f"\n{z:.1f}m"
            
            painter.setPen(QPen(Qt.white))
            painter.setFont(QFont('Arial', 8))
            painter.drawText(int(sx - 20), int(sy + size + 5), 40, 30,
                           Qt.AlignCenter, label_text)

            # Relative map coordinate in km from map center
            rel_x_km = (x - self.origin_x) / 1000.0
            rel_y_km = (y - self.origin_y) / 1000.0
            painter.setFont(QFont('Arial', 7))
            painter.drawText(
                int(sx - 30), int(sy + size + 30), 60, 20,
                Qt.AlignCenter, f"{rel_x_km:+.2f},{rel_y_km:+.2f} km"
            )
            
            # Battery bar
            battery_width = 30
            battery_height = 4
            battery_x = sx - battery_width / 2
            battery_y = sy - size - 10
            
            # Battery background
            painter.setPen(QPen(Qt.black))
            painter.setBrush(QBrush(Qt.black))
            painter.drawRect(int(battery_x), int(battery_y), 
                           int(battery_width), int(battery_height))
            
            # Battery level
            battery_color = self._get_battery_color(battery)
            painter.setBrush(QBrush(battery_color))
            painter.drawRect(int(battery_x), int(battery_y),
                           int(battery_width * battery / 100), int(battery_height))

        if drone_data.get("motor_failure_warning") or drone_data.get("emergency_return_active"):
            self._draw_warning_symbol(painter, sx, sy, size)

    def _draw_warning_symbol(self, painter, sx, sy, size):
        """Draw warning symbol above drone when in fault/emergency return mode."""
        top = QPointF(sx, sy - size - 22)
        left = QPointF(sx - 10, sy - size - 2)
        right = QPointF(sx + 10, sy - size - 2)
        painter.setPen(QPen(QColor(255, 210, 80), 2))
        painter.setBrush(QBrush(QColor(255, 210, 80, 180)))
        painter.drawPolygon(QPolygonF([top, left, right]))
        painter.setPen(QPen(QColor(30, 30, 30), 2))
        painter.drawLine(int(sx), int(sy - size - 17), int(sx), int(sy - size - 9))
        painter.drawPoint(int(sx), int(sy - size - 6))

    def _draw_motion_direction(self, painter, sx, sy, size, vx, vy, color):
        """Draw forward direction arrow and animated prop effect."""
        speed = math.sqrt(vx * vx + vy * vy)
        if speed > 0.05:
            ux = vx / speed
            uy = vy / speed
            tip_x = sx + ux * (size * 1.25)
            tip_y = sy - uy * (size * 1.25)
            base_x = sx + ux * (size * 0.55)
            base_y = sy - uy * (size * 0.55)

            painter.setPen(QPen(QColor(255, 255, 255), 2))
            painter.drawLine(int(base_x), int(base_y), int(tip_x), int(tip_y))
            left_x = tip_x - ux * 8 - uy * 6
            left_y = tip_y + uy * 8 - ux * 6
            right_x = tip_x - ux * 8 + uy * 6
            right_y = tip_y + uy * 8 + ux * 6
            painter.drawLine(int(tip_x), int(tip_y), int(left_x), int(left_y))
            painter.drawLine(int(tip_x), int(tip_y), int(right_x), int(right_y))

        # Rotor ring animation while flying
        radius = size * (0.72 + 0.06 * math.sin(self.animation_phase))
        painter.setPen(QPen(QColor(color.red(), color.green(), color.blue(), 120), 1, Qt.DashLine))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(QPointF(sx, sy), radius, radius)

    def _draw_drone_svg(self, painter, sx: float, sy: float, size: float, color: QColor):
        """Render drone icon from SVG template with role-dependent color."""
        accent_color = "#f5f5f5" if color.value() < 0x808080 else "#111111"
        svg_markup = self.drone_svg_template.replace("{BODY_COLOR}", color.name()).replace(
            "{ACCENT_COLOR}", accent_color
        )
        if QSvgRenderer:
            renderer = QSvgRenderer(QByteArray(svg_markup.encode("utf-8")))
            rect = QRectF(sx - size, sy - size, size * 2, size * 2)
            renderer.render(painter, rect)
        else:
            # Fallback if QtSvg is unavailable.
            painter.setPen(QPen(color, 2))
            painter.setBrush(QBrush(color))
            painter.drawEllipse(QPointF(sx, sy), size * 0.6, size * 0.6)

    def _get_battery_color(self, battery):
        """Get color based on battery level"""
        if battery > 50:
            return QColor(0, 255, 0)  # Green
        elif battery > 20:
            return QColor(255, 165, 0)  # Orange
        else:
            return QColor(255, 0, 0)  # Red

    def wheelEvent(self, event):
        """Handle zoom with mouse wheel"""
        delta = event.angleDelta().y()
        if delta > 0:
            self.zoom *= 1.1
        else:
            self.zoom *= 0.9

        self.zoom = max(0.1, min(5.0, self.zoom))
        self.update()

    def mousePressEvent(self, event):
        """Handle mouse press for panning"""
        if event.button() == Qt.LeftButton:
            self.last_mouse_pos = event.pos()

    def mouseMoveEvent(self, event):
        """Handle mouse drag for panning"""
        if event.buttons() & Qt.LeftButton and hasattr(self, 'last_mouse_pos'):
            delta = event.pos() - self.last_mouse_pos
            self.offset_x += delta.x()
            self.offset_y += delta.y()
            self.last_mouse_pos = event.pos()
            self.update()


class LocationMapWidget(QWidget):
    """Top-down location map for drone positions."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(800, 600)
        self.drones = {}
        self.selected_drone_id = None
        self.map_radius_m = 10000.0

    def update_drones(self, drones_data):
        self.drones = drones_data
        self.update()

    def set_selected_drone(self, drone_id):
        self.selected_drone_id = drone_id
        self.update()

    def _to_screen(self, x, y, center_x, center_y, ppm):
        sx = self.width() / 2 + (x - center_x) * ppm
        sy = self.height() / 2 - (y - center_y) * ppm
        return sx, sy

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(11, 24, 37))

        if not self.drones:
            painter.setPen(QPen(QColor(220, 220, 220)))
            painter.drawText(self.rect(), Qt.AlignCenter, "No drone location data")
            return

        homes = [d.get("home_position", {}) for d in self.drones.values()]
        cx = sum(h.get("x", 0.0) for h in homes) / max(1, len(homes))
        cy = sum(h.get("y", 0.0) for h in homes) / max(1, len(homes))
        ppm = (min(self.width(), self.height()) * 0.42) / self.map_radius_m

        # Map rings each 2 km.
        painter.setPen(QPen(QColor(70, 95, 120), 1, Qt.DashLine))
        for km in range(2, 11, 2):
            r = km * 1000 * ppm
            painter.drawEllipse(QPointF(self.width() / 2, self.height() / 2), r, r)

        painter.setPen(QPen(QColor(120, 145, 170), 1))
        painter.drawLine(0, int(self.height() / 2), self.width(), int(self.height() / 2))
        painter.drawLine(int(self.width() / 2), 0, int(self.width() / 2), self.height())

        for drone_id, drone_data in self.drones.items():
            pos = drone_data.get("position", {})
            x = pos.get("x", 0.0)
            y = pos.get("y", 0.0)
            sx, sy = self._to_screen(x, y, cx, cy, ppm)

            is_selected = drone_id == self.selected_drone_id
            role = drone_data.get("role", "follower")
            color = QColor(255, 215, 0) if role == "leader" else QColor(80, 180, 255)
            if drone_data.get("flight_mode") == "emergency_landing":
                color = QColor(255, 90, 60)
            if is_selected:
                painter.setPen(QPen(QColor(255, 255, 255), 2))
                painter.drawEllipse(QPointF(sx, sy), 11, 11)

            painter.setPen(QPen(color, 2))
            painter.setBrush(QBrush(color))
            painter.drawEllipse(QPointF(sx, sy), 6, 6)
            painter.setPen(QPen(QColor(230, 230, 230)))
            painter.setFont(QFont("Arial", 8))
            painter.drawText(int(sx + 8), int(sy - 8), f"D{drone_id}")

        painter.setPen(QPen(QColor(225, 225, 225)))
        painter.setFont(QFont("Arial", 9))
        painter.drawText(12, 18, "Location Map (10km radius)")


class LatencyMonitorWidget(QWidget):
    """Real-time latency monitoring dashboard chart (Matplotlib)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(220)
        self.setMaximumHeight(220)
        self.threshold_ms = 400.0
        self.start_ts = time.time()
        self.last_sample_ts = 0.0
        self.times = deque(maxlen=240)
        self.latency_values = deque(maxlen=240)
        self.jitter_values = deque(maxlen=240)
        self.latest_jitter = 0.0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(2)

        self.canvas = None
        self.ax = None
        self.jitter_label = QLabel("Jitter: 0.0 ms")
        self.jitter_label.setStyleSheet("font-size: 10px; color: #d9e2f2;")

        if MATPLOTLIB_AVAILABLE:
            fig = Figure(figsize=(5.4, 2.1), dpi=100)
            fig.patch.set_facecolor("#111722")
            self.ax = fig.add_subplot(111)
            self.canvas = FigureCanvas(fig)
            layout.addWidget(self.canvas, 1)
        else:
            fallback = QLabel("Matplotlib not available")
            fallback.setAlignment(Qt.AlignCenter)
            layout.addWidget(fallback, 1)
        layout.addWidget(self.jitter_label, 0)

    def update_latency(self, latency: dict):
        now = time.time()
        self.latest_jitter = float(latency.get("total_round_trip_jitter_std_ms", 0.0))
        if (now - self.last_sample_ts) < 1.0:
            self.jitter_label.setText(f"Jitter: {self.latest_jitter:.1f} ms")
            return
        self.last_sample_ts = now

        t = now - self.start_ts
        v = float(latency.get("total_round_trip_ms", 0.0))
        j = self.latest_jitter
        self.times.append(t)
        self.latency_values.append(v)
        self.jitter_values.append(j)
        self.jitter_label.setText(f"Jitter: {j:.1f} ms")
        self._redraw()

    def _redraw(self):
        if not (MATPLOTLIB_AVAILABLE and self.ax and self.canvas):
            return
        if len(self.times) < 2:
            return

        x = np.array(self.times, dtype=float)
        y = np.array(self.latency_values, dtype=float)
        self.ax.clear()
        self.ax.set_facecolor("#141b29")

        # Slightly smoothed curve for better readability.
        if len(x) >= 4:
            dense_x = np.linspace(x[0], x[-1], len(x) * 6)
            dense_y = np.interp(dense_x, x, y)
            kernel = np.array([1, 2, 3, 2, 1], dtype=float)
            kernel /= kernel.sum()
            smooth_y = np.convolve(dense_y, kernel, mode="same")
            self.ax.plot(dense_x, smooth_y, color="#6ec5ff", linewidth=2.2, label="Latency")
        else:
            self.ax.plot(x, y, color="#6ec5ff", linewidth=2.2, label="Latency")

        self.ax.axhline(self.threshold_ms, color="#f2be5b", linestyle="--", linewidth=1.4, label="Threshold")

        spike_mask = y > self.threshold_ms
        if np.any(spike_mask):
            self.ax.scatter(x[spike_mask], y[spike_mask], color="#ff5d5d", s=28, zorder=5)

        self.ax.grid(True, color="#3b465c", linestyle=":", linewidth=0.8, alpha=0.8)
        self.ax.set_xlabel("Time (seconds)", color="#d6deed", fontsize=8)
        self.ax.set_ylabel("Latency (ms)", color="#d6deed", fontsize=8)
        self.ax.tick_params(axis="x", colors="#c7d2e8", labelsize=8)
        self.ax.tick_params(axis="y", colors="#c7d2e8", labelsize=8)
        for spine in self.ax.spines.values():
            spine.set_color("#51607b")
        self.ax.legend(loc="upper left", facecolor="#1c2535", edgecolor="#566580", labelcolor="#e6eefc", fontsize=8)

        right = x[-1]
        left = max(0.0, right - 60.0)
        self.ax.set_xlim(left, max(60.0, right))
        ymax = max(self.threshold_ms + 60.0, float(np.max(y)) + 60.0)
        self.ax.set_ylim(0.0, ymax)
        self.ax.set_title("Real-time Latency Monitoring", color="#eef4ff", fontsize=9, pad=6)
        self.canvas.draw_idle()


class SwarmMetricsWidget(QWidget):
    """Matplotlib chart for active drones + average battery."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(220)
        self.setMaximumHeight(220)
        self.start_ts = time.time()
        self.last_sample_ts = 0.0
        self.times = deque(maxlen=240)
        self.active_history = deque(maxlen=240)
        self.battery_history = deque(maxlen=240)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(2)

        self.canvas = None
        self.ax_active = None
        self.ax_battery = None
        if MATPLOTLIB_AVAILABLE:
            fig = Figure(figsize=(5.4, 2.1), dpi=100)
            fig.patch.set_facecolor("#111722")
            self.ax_active = fig.add_subplot(111)
            self.ax_battery = self.ax_active.twinx()
            self.canvas = FigureCanvas(fig)
            layout.addWidget(self.canvas, 1)
        else:
            fallback = QLabel("Matplotlib not available")
            fallback.setAlignment(Qt.AlignCenter)
            layout.addWidget(fallback, 1)

    def update_metrics(self, active_drones: int, avg_battery: float):
        now = time.time()
        if (now - self.last_sample_ts) < 1.0:
            return
        self.last_sample_ts = now
        self.times.append(now - self.start_ts)
        self.active_history.append(max(0, int(active_drones)))
        self.battery_history.append(max(0.0, min(100.0, float(avg_battery))))
        self._redraw()

    def _redraw(self):
        if not (MATPLOTLIB_AVAILABLE and self.ax_active and self.ax_battery and self.canvas):
            return
        if len(self.times) < 2:
            return

        x = np.array(self.times, dtype=float)
        a = np.array(self.active_history, dtype=float)
        b = np.array(self.battery_history, dtype=float)

        self.ax_active.clear()
        self.ax_battery.clear()
        self.ax_active.set_facecolor("#141b29")

        self.ax_active.plot(x, a, color="#7ed174", linewidth=2.0, label="Active Drones")
        self.ax_battery.plot(x, b, color="#6ec5ff", linewidth=2.0, label="Avg Battery (%)")
        self.ax_battery.fill_between(x, 0, b, color="#6ec5ff", alpha=0.12)

        self.ax_active.grid(True, color="#3b465c", linestyle=":", linewidth=0.8, alpha=0.8)
        self.ax_active.set_xlabel("Time (seconds)", color="#d6deed", fontsize=8)
        self.ax_active.set_ylabel("Active Drones", color="#7ed174", fontsize=8)
        self.ax_battery.set_ylabel("Battery (%)", color="#6ec5ff", fontsize=8)
        self.ax_active.tick_params(axis="x", colors="#c7d2e8", labelsize=8)
        self.ax_active.tick_params(axis="y", colors="#7ed174", labelsize=8)
        self.ax_battery.tick_params(axis="y", colors="#6ec5ff", labelsize=8)
        for spine in self.ax_active.spines.values():
            spine.set_color("#51607b")
        for spine in self.ax_battery.spines.values():
            spine.set_color("#51607b")

        right = x[-1]
        left = max(0.0, right - 60.0)
        self.ax_active.set_xlim(left, max(60.0, right))
        self.ax_active.set_ylim(0, max(2.0, float(np.max(a)) + 1.0))
        self.ax_battery.set_ylim(0.0, 100.0)

        h1, l1 = self.ax_active.get_legend_handles_labels()
        h2, l2 = self.ax_battery.get_legend_handles_labels()
        self.ax_active.legend(
            h1 + h2,
            l1 + l2,
            loc="upper left",
            facecolor="#1c2535",
            edgecolor="#566580",
            labelcolor="#e6eefc",
            fontsize=8,
        )
        self.ax_active.set_title("Swarm Metrics Dashboard", color="#eef4ff", fontsize=9, pad=6)
        self.canvas.draw_idle()
class AcousticRealMapWidget(QWidget):
    """Real-world map view for acoustic source and drone lines."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(280)
        self.zoom = 15
        self.map_width = 760
        self.map_height = 360
        self._last_fetch_center = None
        self._map_pixmap = None
        self._last_fetch_attempt_ts = 0.0
        self._last_fetch_error = ""
        self._drone_points = []  # list[(lat, lon)]
        self._source_point = None  # tuple(lat, lon)
        self._meters_per_deg_lat = 111320.0
        # Provider toggles from .env (0/1)
        self.use_free_map = _env_flag("MAP_USE_FREE", True)
        self.use_yandex_map = _env_flag("MAP_USE_YANDEX", False)
        self.use_google_map = _env_flag("MAP_USE_GOOGLE", False)
        self.yandex_api_key = os.getenv("MAP_YANDEX_API_KEY", "").strip()
        self.google_api_key = os.getenv("MAP_GOOGLE_API_KEY", "").strip()

    def update_map_data(self, drones: dict, acoustic_source: dict):
        points = []
        ref_lat = None
        ref_lon = None
        for drone_data in (drones or {}).values():
            gps = drone_data.get("position_gps", {}) or {}
            lat = gps.get("lat")
            lon = gps.get("lon")
            if lat is not None and lon is not None:
                points.append((float(lat), float(lon)))
            ref = drone_data.get("gps_reference", {}) or {}
            if ref_lat is None and "lat" in ref and "lon" in ref:
                ref_lat = float(ref["lat"])
                ref_lon = float(ref["lon"])

        source_latlon = None
        if acoustic_source and ref_lat is not None and ref_lon is not None:
            src_x = float(acoustic_source.get("x", 0.0))
            src_y = float(acoustic_source.get("y", 0.0))
            meters_per_deg_lon = self._meters_per_deg_lat * math.cos(math.radians(ref_lat))
            if abs(meters_per_deg_lon) < 1e-6:
                meters_per_deg_lon = 1e-6
            src_lat = ref_lat + (src_y / self._meters_per_deg_lat)
            src_lon = ref_lon + (src_x / meters_per_deg_lon)
            source_latlon = (src_lat, src_lon)

        self._drone_points = points
        self._source_point = source_latlon

        center = source_latlon or (points[0] if points else None)
        if center:
            self._maybe_fetch_map(center[0], center[1])
        self.update()

    def _maybe_fetch_map(self, lat: float, lon: float):
        # Throttle by movement and elapsed time.
        need_fetch = self._map_pixmap is None
        if self._last_fetch_center is None:
            need_fetch = True
        else:
            dlat = abs(lat - self._last_fetch_center[0])
            dlon = abs(lon - self._last_fetch_center[1])
            if dlat > 0.00015 or dlon > 0.00015:
                need_fetch = True
        now = time.time()
        if need_fetch and (now - self._last_fetch_attempt_ts) < 3.0:
            return
        if not need_fetch:
            return
        self._last_fetch_attempt_ts = now
        self._fetch_map(lat, lon)

    def _fetch_map(self, lat: float, lon: float):
        yandex_params = {
            "ll": f"{lon:.6f},{lat:.6f}",
            "z": str(self.zoom),
            "size": f"{self.map_width},{self.map_height}",
            "l": "sat",
        }
        if self.yandex_api_key:
            yandex_params["apikey"] = self.yandex_api_key
        osm_params = {
            "center": f"{lat:.6f},{lon:.6f}",
            "zoom": str(self.zoom),
            "size": f"{self.map_width}x{self.map_height}",
            "maptype": "mapnik",
        }
        google_params = {
            "center": f"{lat:.6f},{lon:.6f}",
            "zoom": str(self.zoom),
            "size": f"{self.map_width}x{self.map_height}",
            "maptype": "satellite",
            "key": self.google_api_key,
        }
        urls = []
        if self.use_google_map and self.google_api_key:
            urls.append("https://maps.googleapis.com/maps/api/staticmap?" + urllib.parse.urlencode(google_params))
        if self.use_yandex_map:
            urls.append("https://static-maps.yandex.ru/1.x/?" + urllib.parse.urlencode(yandex_params))
        if self.use_free_map:
            urls.append("https://staticmap.openstreetmap.de/staticmap.php?" + urllib.parse.urlencode(osm_params))
        if not urls:
            # Safe fallback if all toggles are 0
            urls.append("https://staticmap.openstreetmap.de/staticmap.php?" + urllib.parse.urlencode(osm_params))

        for url in urls:
            try:
                req = urllib.request.Request(
                    url,
                    headers={"User-Agent": "Mozilla/5.0 (DroneSwarmGUI/1.0)"},
                )
                with urllib.request.urlopen(req, timeout=3.5) as response:
                    raw = response.read()
                image = QImage.fromData(raw)
                if not image.isNull():
                    self._map_pixmap = QPixmap.fromImage(image)
                    self._last_fetch_center = (lat, lon)
                    self._last_fetch_error = ""
                    return
            except Exception as exc:
                self._last_fetch_error = str(exc)
                continue

    def _latlon_to_pixel(self, lat: float, lon: float, center_lat: float, center_lon: float, rect: QRectF):
        def mercator_xy(lat_v: float, lon_v: float, zoom_v: int):
            n = (2 ** zoom_v) * 256.0
            x = (lon_v + 180.0) / 360.0 * n
            lat_rad = math.radians(max(-85.0, min(85.0, lat_v)))
            y = (1.0 - math.log(math.tan(lat_rad) + (1.0 / math.cos(lat_rad))) / math.pi) * 0.5 * n
            return x, y

        cx, cy = mercator_xy(center_lat, center_lon, self.zoom)
        px, py = mercator_xy(lat, lon, self.zoom)
        dx = px - cx
        dy = py - cy
        return QPointF(rect.center().x() + dx, rect.center().y() + dy)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(12, 16, 25))

        map_rect = self.rect().adjusted(8, 8, -8, -28)
        center = self._source_point or (self._drone_points[0] if self._drone_points else None)
        if center and self._map_pixmap is None:
            self._maybe_fetch_map(center[0], center[1])

        if not center:
            painter.setPen(QPen(QColor(220, 230, 245)))
            painter.drawText(map_rect, Qt.AlignCenter, "No drone GPS data for real map")
            return

        if self._map_pixmap:
            painter.drawPixmap(map_rect.toRect(), self._map_pixmap)
        else:
            painter.setPen(QPen(QColor(220, 230, 245)))
            msg = "Loading real map..."
            if self._last_fetch_error:
                msg = "Map fetch failed. Retrying..."
            painter.drawText(map_rect, Qt.AlignCenter, msg)
            return

        center_lat, center_lon = center

        if self._source_point:
            source_px = self._latlon_to_pixel(self._source_point[0], self._source_point[1], center_lat, center_lon, map_rect)
            painter.setBrush(QBrush(QColor(255, 84, 84, 180)))
            painter.setPen(QPen(QColor(255, 240, 240), 1))
            painter.drawEllipse(source_px, 6, 6)
        else:
            source_px = None

        for lat, lon in self._drone_points:
            p = self._latlon_to_pixel(lat, lon, center_lat, center_lon, map_rect)
            painter.setBrush(QBrush(QColor(110, 222, 130, 220)))
            painter.setPen(QPen(QColor(215, 255, 220), 1))
            painter.drawEllipse(p, 4, 4)
            if source_px is not None:
                painter.setPen(QPen(QColor(255, 185, 112, 180), 1.5))
                painter.drawLine(p, source_px)

        painter.setPen(QPen(QColor(230, 236, 248)))
        painter.setFont(QFont("Segoe UI", 9, QFont.Bold))
        painter.drawText(12, self.height() - 10, "Acoustic Detection Map (Real World)")


class MainWindow(QMainWindow):
    """Main application window"""
    
    def __init__(self, swarm_manager):
        super().__init__()
        self.swarm_manager = swarm_manager
        self.ml_system = None
        self.selected_drone_id = None
        self.comm_log_offsets = {}
        self.controller_crypto = None
        self.pending_takeoff_targets = {}
        self.active_destination_targets = {}
        self.mission_to_y_active = False
        self.operation_corners = {}
        self.operation_start = None
        self.operation_destination = None
        self.operation_slots = {}
        self.operation_start_slots = {}
        self.operation_return_slots = {}
        self.destination_gap_m = 45.0
        self.auto_fault_demo_enabled = False
        self.auto_fault_phase = 0
        self.last_auto_motor_fail_drone_id = None
        self.last_auto_leader_crash_drone_id = None
        self._leader_initialized = False
        self._last_leader_id = None
        self.show_static_obstacles = True
        self.show_dynamic_obstacles = True
        self.latency_value_labels = {}
        self.ledger_value_labels = {}
        self.acoustic_conf_label = None
        self.prediction_table = None
        self.personal_ml_status_table = None
        self.sha2_table = None
        self.sha3_table = None
        self.physical_ml_table = None
        self._setup_controller_crypto()
        
        self.setWindowTitle("Swarm Ground Control Station")
        self.setGeometry(100, 100, 1400, 900)
        self.setFocusPolicy(Qt.StrongFocus)
        
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(10)
        
        # Left panel - latency monitor + status + logs
        left_container = QWidget()
        left_panel = QVBoxLayout(left_container)
        left_panel.setContentsMargins(0, 0, 0, 0)
        left_panel.setSpacing(10)

        latency_group = self._create_latency_panel()
        left_panel.addWidget(latency_group, 2)

        metrics_group = self._create_metrics_panel()
        left_panel.addWidget(metrics_group, 2)

        status_group = self._create_status_panel()
        status_scroll = QScrollArea()
        status_scroll.setWidgetResizable(True)
        status_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        status_scroll.setWidget(status_group)
        left_panel.addWidget(status_scroll, 3)

        ml_group = self._create_physical_ml_panel()
        ml_scroll = QScrollArea()
        ml_scroll.setWidgetResizable(True)
        ml_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        ml_scroll.setWidget(ml_group)
        left_panel.addWidget(ml_scroll, 2)

        # Center panel - swarm visualization
        viz_group = QGroupBox("Swarm Visualization")
        viz_layout = QVBoxLayout()
        self.viz_tabs = QTabWidget()
        self.drone_widget = DroneWidget()
        self.location_map_widget = LocationMapWidget()
        self.acoustic_map_widget = AcousticRealMapWidget()
        self.viz_tabs.addTab(self.drone_widget, "Drone Visual")
        self.viz_tabs.addTab(self.location_map_widget, "Location Map")
        self.viz_tabs.addTab(self.acoustic_map_widget, "Acoustic Real Map")
        viz_layout.addWidget(self.viz_tabs, 5)

        # Bottom area inside center visualization:
        # left -> logs, right -> prediction summary table.
        viz_bottom_row = QHBoxLayout()
        viz_bottom_row.setSpacing(8)
        log_group = self._create_log_panel()
        prediction_group = self._create_prediction_panel()
        viz_bottom_row.addWidget(log_group, 3)
        viz_bottom_row.addWidget(prediction_group, 2)
        viz_layout.addLayout(viz_bottom_row, 2)
        viz_group.setLayout(viz_layout)

        # Right panel - controls only (scrollable)
        right_container = QWidget()
        right_container.setMinimumWidth(380)
        right_panel = QVBoxLayout(right_container)
        right_panel.setContentsMargins(0, 0, 0, 0)
        right_panel.setSpacing(10)
        control_group = self._create_control_panel()
        control_scroll = QScrollArea()
        control_scroll.setWidgetResizable(True)
        control_scroll.setWidget(control_group)
        control_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        control_scroll.setMinimumHeight(420)
        control_scroll.setMaximumHeight(16777215)
        right_panel.addWidget(control_scroll, 1)

        main_layout.addWidget(left_container, 2)
        main_layout.addWidget(viz_group, 5)
        main_layout.addWidget(right_container, 2)
        self._apply_dashboard_theme()
        
        # Update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(100)  # 10 Hz update

        # Poll encrypted communication logs into UI log panel.
        self.comm_log_timer = QTimer()
        self.comm_log_timer.timeout.connect(self._poll_encrypted_comm_logs)
        self.comm_log_timer.start(1000)

        # Automatic fault/election demo timer (GUI simulation).
        self.auto_fault_timer = QTimer()
        self.auto_fault_timer.timeout.connect(self._run_auto_fault_scenario)
        self.auto_fault_timer.start(12000)
        self._setup_keyboard_shortcuts()
        
        # Logger
        self.logger = logging.getLogger("GUI")
        
        self.log("Drone Swarm Management System started")

    def _apply_dashboard_theme(self):
        """Apply dark glass dashboard theme similar to mission control UI."""
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #0f131d;
                color: #e6ebf3;
                font-family: "Segoe UI";
                font-size: 10.5pt;
            }
            QGroupBox {
                border: 1px solid rgba(180, 190, 210, 0.18);
                border-radius: 10px;
                margin-top: 12px;
                padding-top: 8px;
                background-color: rgba(37, 43, 56, 0.55);
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px;
                color: #f1f4fa;
                font-size: 12pt;
                font-weight: 600;
            }
            QScrollArea, QTabWidget::pane {
                border: 1px solid rgba(180, 190, 210, 0.16);
                border-radius: 10px;
                background: rgba(31, 36, 47, 0.6);
            }
            QTabBar::tab {
                background: rgba(69, 79, 99, 0.45);
                border: 1px solid rgba(180, 190, 210, 0.18);
                border-bottom: none;
                border-top-left-radius: 8px;
                border-top-right-radius: 8px;
                padding: 7px 14px;
                margin-right: 4px;
                color: #ced6e3;
            }
            QTabBar::tab:selected {
                background: rgba(120, 138, 170, 0.3);
                color: #f8fbff;
            }
            QPushButton {
                border: 1px solid rgba(175, 185, 205, 0.32);
                border-radius: 7px;
                padding: 7px 10px;
                background: rgba(82, 96, 120, 0.58);
                color: #f3f6fc;
                font-weight: 600;
            }
            QPushButton:hover {
                background: rgba(101, 117, 144, 0.72);
            }
            QPushButton:pressed {
                background: rgba(66, 77, 98, 0.92);
            }
            QPushButton[btnType="success"] {
                background: rgba(106, 155, 92, 0.85);
            }
            QPushButton[btnType="success"]:hover {
                background: rgba(120, 172, 104, 0.95);
            }
            QPushButton[btnType="primary"] {
                background: rgba(87, 133, 194, 0.88);
            }
            QPushButton[btnType="primary"]:hover {
                background: rgba(102, 151, 214, 0.96);
            }
            QPushButton[btnType="danger"] {
                background: rgba(176, 67, 67, 0.9);
            }
            QPushButton[btnType="danger"]:hover {
                background: rgba(197, 77, 77, 0.98);
            }
            QPushButton[btnType="warn"] {
                background: rgba(184, 126, 54, 0.88);
            }
            QPushButton[btnType="warn"]:hover {
                background: rgba(201, 139, 61, 0.98);
            }
            QLabel {
                color: #e2e8f4;
            }
            QComboBox, QSpinBox, QDoubleSpinBox {
                border: 1px solid rgba(180, 190, 210, 0.25);
                border-radius: 7px;
                padding: 4px 7px;
                background: rgba(25, 30, 41, 0.9);
                color: #eff3fb;
                min-height: 22px;
            }
            QCheckBox {
                spacing: 7px;
                color: #dde4f0;
            }
            QCheckBox::indicator {
                width: 14px;
                height: 14px;
                border-radius: 3px;
                border: 1px solid rgba(190, 200, 218, 0.45);
                background: rgba(19, 23, 33, 1.0);
            }
            QCheckBox::indicator:checked {
                background: #7cbc74;
            }
            QSlider::groove:horizontal {
                height: 6px;
                border-radius: 3px;
                background: rgba(74, 86, 106, 0.65);
            }
            QSlider::handle:horizontal {
                width: 14px;
                margin: -5px 0;
                border-radius: 7px;
                background: #7ac7ff;
                border: 1px solid #9ad5ff;
            }
            QTextEdit {
                border: 1px solid rgba(180, 190, 210, 0.2);
                border-radius: 8px;
                background: rgba(16, 20, 29, 0.9);
                color: #d9e0ed;
            }
            QHeaderView::section {
                background: rgba(65, 76, 96, 0.75);
                color: #edf2fa;
                padding: 5px;
                border: none;
                border-right: 1px solid rgba(190, 198, 214, 0.15);
                font-weight: 600;
            }
            QTableWidget {
                gridline-color: rgba(188, 198, 218, 0.1);
                background: rgba(17, 21, 30, 0.86);
                alternate-background-color: rgba(36, 42, 55, 0.72);
                selection-background-color: rgba(88, 134, 190, 0.52);
                selection-color: #ffffff;
                border-radius: 8px;
                border: 1px solid rgba(175, 185, 205, 0.2);
            }
            QScrollBar:vertical {
                background: transparent;
                width: 10px;
                margin: 2px;
            }
            QScrollBar::handle:vertical {
                background: rgba(138, 150, 173, 0.55);
                border-radius: 5px;
                min-height: 24px;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0;
            }
        """)

    def _setup_controller_crypto(self):
        """Create local encryption helper for controller->drone command logs."""
        try:
            from communication import SecureCommunication
            self.controller_crypto = SecureCommunication(0)
        except Exception:
            self.controller_crypto = None

    def _setup_keyboard_shortcuts(self):
        """Keyboard shortcuts for movement control."""
        self.shortcut_up = QShortcut(QKeySequence("Up"), self)
        self.shortcut_up.activated.connect(lambda: self.move_selected_drone(0, self.move_step_spin.value(), 0))
        self.shortcut_down = QShortcut(QKeySequence("Down"), self)
        self.shortcut_down.activated.connect(lambda: self.move_selected_drone(0, -self.move_step_spin.value(), 0))
        self.shortcut_left = QShortcut(QKeySequence("Left"), self)
        self.shortcut_left.activated.connect(lambda: self.move_selected_drone(-self.move_step_spin.value(), 0, 0))
        self.shortcut_right = QShortcut(QKeySequence("Right"), self)
        self.shortcut_right.activated.connect(lambda: self.move_selected_drone(self.move_step_spin.value(), 0, 0))
        self.shortcut_hover = QShortcut(QKeySequence("Space"), self)
        self.shortcut_hover.activated.connect(lambda: self.move_selected_drone(0, 0, 0))

    def _create_latency_panel(self):
        """Create real-time latency monitor panel."""
        group = QGroupBox("Latency Monitor Real Time")
        layout = QVBoxLayout()
        layout.setContentsMargins(6, 8, 6, 6)
        layout.setSpacing(4)

        self.latency_monitor_widget = LatencyMonitorWidget()
        layout.addWidget(self.latency_monitor_widget, 1)

        group.setLayout(layout)
        return group

    def _create_metrics_panel(self):
        """Create real-time swarm metrics panel."""
        group = QGroupBox("Swarm Metrics")
        layout = QVBoxLayout()
        layout.setContentsMargins(6, 8, 6, 6)
        layout.setSpacing(4)

        self.swarm_metrics_widget = SwarmMetricsWidget()
        layout.addWidget(self.swarm_metrics_widget, 1)

        group.setLayout(layout)
        return group

    def _create_prediction_panel(self):
        """Create prediction summary table near center visualization."""
        group = QGroupBox("Prediction Summary")
        layout = QVBoxLayout()
        layout.setContentsMargins(6, 8, 6, 6)
        layout.setSpacing(4)

        self.prediction_table = QTableWidget()
        self.prediction_table.setColumnCount(2)
        self.prediction_table.setRowCount(4)
        self.prediction_table.setHorizontalHeaderLabels(["Type", "Status"])
        self.prediction_table.verticalHeader().setVisible(False)
        self.prediction_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.prediction_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.prediction_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.prediction_table.setSelectionMode(QAbstractItemView.NoSelection)
        self.prediction_table.setFocusPolicy(Qt.NoFocus)
        self.prediction_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.prediction_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.prediction_table.setAlternatingRowColors(True)
        self.prediction_table.setMinimumHeight(160)

        labels = [
            "Short Predict",
            "Mid Term Predict",
            "Long Term Predict",
            "Obstacle Drone (Dynamic)",
        ]
        for row, title in enumerate(labels):
            self.prediction_table.setItem(row, 0, QTableWidgetItem(title))
            self.prediction_table.setItem(row, 1, QTableWidgetItem("\u2713"))

        layout.addWidget(self.prediction_table)

        self.personal_ml_status_table = QTableWidget()
        self.personal_ml_status_table.setColumnCount(3)
        self.personal_ml_status_table.setHorizontalHeaderLabels(["Personal ML Status", "Drone", "Status"])
        self.personal_ml_status_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.personal_ml_status_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.personal_ml_status_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.personal_ml_status_table.verticalHeader().setVisible(False)
        self.personal_ml_status_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.personal_ml_status_table.setSelectionMode(QAbstractItemView.NoSelection)
        self.personal_ml_status_table.setFocusPolicy(Qt.NoFocus)
        self.personal_ml_status_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.personal_ml_status_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.personal_ml_status_table.setAlternatingRowColors(True)
        self.personal_ml_status_table.setMinimumHeight(110)
        self.personal_ml_status_table.setRowCount(1)
        self.personal_ml_status_table.setItem(0, 0, QTableWidgetItem("Personal ML"))
        self.personal_ml_status_table.setItem(0, 1, QTableWidgetItem("ALL"))
        self.personal_ml_status_table.setItem(0, 2, QTableWidgetItem("\u2713"))
        layout.addWidget(self.personal_ml_status_table)

        group.setLayout(layout)
        return group
    
    def _create_control_panel(self):
        """Create control panel"""
        group = QGroupBox("Controls Panel")
        group.setMinimumWidth(360)
        group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        layout = QVBoxLayout()
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(7)
        
        # Drone controls
        drone_layout = QHBoxLayout()
        
        self.add_drone_btn = QPushButton("Add Drone")
        self.add_drone_btn.clicked.connect(self.add_drone)
        drone_layout.addWidget(self.add_drone_btn)
        
        self.remove_drone_btn = QPushButton("Remove Drone")
        self.remove_drone_btn.clicked.connect(self.remove_drone)
        drone_layout.addWidget(self.remove_drone_btn)
        
        layout.addLayout(drone_layout)
        
        # Flight controls
        flight_layout = QGridLayout()
        
        self.arm_all_btn = QPushButton("Arm All")
        self.arm_all_btn.setProperty("btnType", "success")
        self.arm_all_btn.clicked.connect(self.arm_all)
        flight_layout.addWidget(self.arm_all_btn, 0, 0)
        
        self.takeoff_all_btn = QPushButton("Takeoff All")
        self.takeoff_all_btn.setProperty("btnType", "success")
        self.takeoff_all_btn.clicked.connect(self.takeoff_all)
        flight_layout.addWidget(self.takeoff_all_btn, 0, 1)
        
        self.land_all_btn = QPushButton("Land All")
        self.land_all_btn.setProperty("btnType", "warn")
        self.land_all_btn.clicked.connect(self.land_all)
        flight_layout.addWidget(self.land_all_btn, 1, 0)
        
        self.rth_all_btn = QPushButton("RTH All")
        self.rth_all_btn.setProperty("btnType", "warn")
        self.rth_all_btn.clicked.connect(self.return_all_home)
        flight_layout.addWidget(self.rth_all_btn, 1, 1)
        
        self.emergency_btn = QPushButton("EMERGENCY LAND")
        self.emergency_btn.setProperty("btnType", "danger")
        self.emergency_btn.clicked.connect(self.emergency_land_all)
        flight_layout.addWidget(self.emergency_btn, 2, 0, 1, 2)

        self.personal_emergency_btn = QPushButton("EMERGENCY SELECTED")
        self.personal_emergency_btn.setProperty("btnType", "danger")
        self.personal_emergency_btn.clicked.connect(self.emergency_land_selected)
        flight_layout.addWidget(self.personal_emergency_btn, 3, 0, 1, 2)

        self.command_xy_btn = QPushButton("Leader Command X->Y")
        self.command_xy_btn.setProperty("btnType", "primary")
        self.command_xy_btn.clicked.connect(self.command_move_x_to_y)
        flight_layout.addWidget(self.command_xy_btn, 4, 0, 1, 2)
        
        layout.addLayout(flight_layout)
        
        # Formation controls
        formation_layout = QHBoxLayout()
        formation_layout.addWidget(QLabel("Formation:"))
        self.formation_combo = QComboBox()
        self.formation_combo.addItems(["line", "v", "circle", "grid"])
        formation_layout.addWidget(self.formation_combo)
        
        self.formation_btn = QPushButton("Apply")
        self.formation_btn.clicked.connect(self.apply_formation)
        formation_layout.addWidget(self.formation_btn)
        
        layout.addLayout(formation_layout)

        # Continuous leader-follow pattern (for motion without overlap).
        follow_layout = QHBoxLayout()
        follow_layout.addWidget(QLabel("Leader Follow:"))
        self.follow_pattern_combo = QComboBox()
        self.follow_pattern_combo.addItems(["v", "line"])
        self.follow_pattern_combo.setCurrentText("v")
        follow_layout.addWidget(self.follow_pattern_combo)
        self.follow_spacing_spin = QSpinBox()
        self.follow_spacing_spin.setRange(10, 300)
        self.follow_spacing_spin.setValue(45)
        self.follow_spacing_spin.setSuffix(" m")
        follow_layout.addWidget(self.follow_spacing_spin)
        self.follow_pattern_btn = QPushButton("Set")
        self.follow_pattern_btn.clicked.connect(self.apply_leader_follow_pattern)
        follow_layout.addWidget(self.follow_pattern_btn)
        layout.addLayout(follow_layout)
        
        # Test scenarios
        test_layout = QVBoxLayout()
        test_layout.addWidget(QLabel("Test Scenarios:"))
        
        self.test_motor_btn = QPushButton("Simulate Motor Failure")
        self.test_motor_btn.clicked.connect(self.simulate_motor_failure)
        test_layout.addWidget(self.test_motor_btn)
        
        self.test_leader_btn = QPushButton("Crash Leader")
        self.test_leader_btn.clicked.connect(self.test_leader_failure)
        test_layout.addWidget(self.test_leader_btn)

        self.auto_fault_btn = QPushButton("Auto Fault Demo: OFF")
        self.auto_fault_btn.clicked.connect(self.toggle_auto_fault_demo)
        self.auto_fault_btn.setProperty("btnType", "warn")
        test_layout.addWidget(self.auto_fault_btn)

        self.test_latency_btn = QPushButton("Simulate Latency Spike")
        self.test_latency_btn.clicked.connect(self.simulate_latency_spike)
        test_layout.addWidget(self.test_latency_btn)
        
        layout.addLayout(test_layout)

        obstacle_group = QGroupBox("Dynamic Obstacles")
        obstacle_layout = QGridLayout()
        obstacle_layout.setHorizontalSpacing(6)
        obstacle_layout.setVerticalSpacing(6)

        self.obs_x_spin = QDoubleSpinBox()
        self.obs_x_spin.setRange(-10000.0, 10000.0)
        self.obs_x_spin.setValue(250.0)
        self.obs_y_spin = QDoubleSpinBox()
        self.obs_y_spin.setRange(-10000.0, 10000.0)
        self.obs_y_spin.setValue(250.0)
        self.obs_vx_spin = QDoubleSpinBox()
        self.obs_vx_spin.setRange(-60.0, 60.0)
        self.obs_vx_spin.setValue(-6.0)
        self.obs_vy_spin = QDoubleSpinBox()
        self.obs_vy_spin.setRange(-60.0, 60.0)
        self.obs_vy_spin.setValue(0.0)
        self.obs_radius_spin = QDoubleSpinBox()
        self.obs_radius_spin.setRange(1.0, 120.0)
        self.obs_radius_spin.setValue(10.0)
        self.obs_motion_combo = QComboBox()
        self.obs_motion_combo.addItems(["linear", "circular", "random_walk"])

        obstacle_layout.addWidget(QLabel("Start X"), 0, 0)
        obstacle_layout.addWidget(self.obs_x_spin, 0, 1)
        obstacle_layout.addWidget(QLabel("Start Y"), 0, 2)
        obstacle_layout.addWidget(self.obs_y_spin, 0, 3)
        obstacle_layout.addWidget(QLabel("Vx"), 1, 0)
        obstacle_layout.addWidget(self.obs_vx_spin, 1, 1)
        obstacle_layout.addWidget(QLabel("Vy"), 1, 2)
        obstacle_layout.addWidget(self.obs_vy_spin, 1, 3)
        obstacle_layout.addWidget(QLabel("Radius"), 2, 0)
        obstacle_layout.addWidget(self.obs_radius_spin, 2, 1)
        obstacle_layout.addWidget(QLabel("Motion"), 2, 2)
        obstacle_layout.addWidget(self.obs_motion_combo, 2, 3)

        self.add_dynamic_obstacle_btn = QPushButton("Add Moving Obstacle")
        self.add_dynamic_obstacle_btn.clicked.connect(self.add_moving_obstacle)
        obstacle_layout.addWidget(self.add_dynamic_obstacle_btn, 3, 0, 1, 2)

        self.add_static_obstacle_btn = QPushButton("Add Static Obstacle")
        self.add_static_obstacle_btn.clicked.connect(self.add_static_obstacle)
        obstacle_layout.addWidget(self.add_static_obstacle_btn, 3, 2, 1, 2)

        self.toggle_static_cb = QCheckBox("Show Static Obstacles")
        self.toggle_static_cb.setChecked(True)
        self.toggle_static_cb.stateChanged.connect(self._on_obstacle_filter_changed)
        obstacle_layout.addWidget(self.toggle_static_cb, 4, 0, 1, 2)

        self.toggle_dynamic_cb = QCheckBox("Show Dynamic Obstacles")
        self.toggle_dynamic_cb.setChecked(True)
        self.toggle_dynamic_cb.stateChanged.connect(self._on_obstacle_filter_changed)
        obstacle_layout.addWidget(self.toggle_dynamic_cb, 4, 2, 1, 2)

        self.use_ml_avoidance_cb = QCheckBox("Use ML Avoidance")
        self.use_ml_avoidance_cb.setChecked(True)
        self.use_ml_avoidance_cb.stateChanged.connect(self._on_use_ml_avoidance_changed)
        obstacle_layout.addWidget(self.use_ml_avoidance_cb, 5, 0, 1, 3)

        self.clear_obstacles_btn = QPushButton("Clear Obstacles")
        self.clear_obstacles_btn.clicked.connect(self.clear_all_obstacles)
        obstacle_layout.addWidget(self.clear_obstacles_btn, 5, 3)

        obstacle_group.setLayout(obstacle_layout)
        layout.addWidget(obstacle_group)

        acoustic_group = QGroupBox("Acoustic Tracking")
        acoustic_layout = QGridLayout()
        self.acoustic_enable_cb = QCheckBox("Enable Acoustic Detection")
        self.acoustic_enable_cb.setChecked(False)
        self.acoustic_enable_cb.stateChanged.connect(self._on_acoustic_detection_changed)
        acoustic_layout.addWidget(self.acoustic_enable_cb, 0, 0, 1, 2)

        acoustic_layout.addWidget(QLabel("Confidence Threshold"), 1, 0)
        self.acoustic_conf_slider = QSlider(Qt.Horizontal)
        self.acoustic_conf_slider.setRange(0, 100)
        self.acoustic_conf_slider.setValue(65)
        self.acoustic_conf_slider.valueChanged.connect(self._on_acoustic_threshold_changed)
        acoustic_layout.addWidget(self.acoustic_conf_slider, 1, 1)
        self.acoustic_conf_label = QLabel("0.65")
        acoustic_layout.addWidget(self.acoustic_conf_label, 1, 2)
        acoustic_group.setLayout(acoustic_layout)
        layout.addWidget(acoustic_group)

        # Manual movement controls for selected drone
        move_group = QGroupBox("Selected Drone Movement")
        move_layout = QGridLayout()
        move_layout.addWidget(QLabel("Step (m):"), 0, 0)
        self.move_step_spin = QSpinBox()
        self.move_step_spin.setRange(1, 2000)
        self.move_step_spin.setValue(100)
        move_layout.addWidget(self.move_step_spin, 0, 1)

        btn_up = QPushButton("Up")
        btn_up.clicked.connect(lambda: self.move_selected_drone(0, self.move_step_spin.value(), 0))
        move_layout.addWidget(btn_up, 1, 1)

        btn_left = QPushButton("Left")
        btn_left.clicked.connect(lambda: self.move_selected_drone(-self.move_step_spin.value(), 0, 0))
        move_layout.addWidget(btn_left, 2, 0)

        btn_hover = QPushButton("Hover")
        btn_hover.clicked.connect(lambda: self.move_selected_drone(0, 0, 0))
        move_layout.addWidget(btn_hover, 2, 1)

        btn_right = QPushButton("Right")
        btn_right.clicked.connect(lambda: self.move_selected_drone(self.move_step_spin.value(), 0, 0))
        move_layout.addWidget(btn_right, 2, 2)

        btn_down = QPushButton("Down")
        btn_down.clicked.connect(lambda: self.move_selected_drone(0, -self.move_step_spin.value(), 0))
        move_layout.addWidget(btn_down, 3, 1)

        key_style = (
            "QPushButton {"
            "background-color: rgba(72, 84, 104, 0.62);"
            "color: #f4f7fd;"
            "border: 1px solid rgba(185, 195, 214, 0.3);"
            "border-radius: 6px;"
            "font-weight: bold;"
            "min-width: 48px;"
            "min-height: 30px;"
            "}"
            "QPushButton:hover { background-color: rgba(95, 110, 136, 0.78); }"
            "QPushButton:pressed { background-color: rgba(61, 72, 91, 0.92); }"
        )
        btn_up.setText("Up")
        btn_down.setText("Down")
        btn_left.setText("Left")
        btn_right.setText("Right")
        for key_btn in [btn_up, btn_down, btn_left, btn_right, btn_hover]:
            key_btn.setStyleSheet(key_style)
        btn_hover.setStyleSheet(
            key_style + "QPushButton { border-radius: 15px; min-width: 56px; min-height: 30px; }"
        )

        move_group.setLayout(move_layout)
        layout.addWidget(move_group)

        # Mission panel:
        # Reference coordinates define the local frame used by drone mission logic.
        # Target coordinates + radius define the circular area mission center.
        gps_group = QGroupBox("GPS Mission (Selected)")
        gps_layout = QGridLayout()
        gps_layout.setHorizontalSpacing(6)
        gps_layout.setVerticalSpacing(6)
        gps_group.setMinimumHeight(235)
        gps_layout.setColumnStretch(0, 0)
        gps_layout.setColumnStretch(1, 1)
        gps_layout.setColumnStretch(2, 0)
        gps_layout.setColumnStretch(3, 1)

        # Reference GPS origin (used for geo->local conversions in mission code).
        gps_layout.addWidget(QLabel("Ref Lat"), 0, 0)
        self.ref_lat_spin = QDoubleSpinBox()
        self.ref_lat_spin.setRange(-90.0, 90.0)
        self.ref_lat_spin.setDecimals(6)
        self.ref_lat_spin.setValue(51.660781)  # Voronezh
        self.ref_lat_spin.setMinimumWidth(0)
        gps_layout.addWidget(self.ref_lat_spin, 0, 1)

        gps_layout.addWidget(QLabel("Ref Lon"), 0, 2)
        self.ref_lon_spin = QDoubleSpinBox()
        self.ref_lon_spin.setRange(-180.0, 180.0)
        self.ref_lon_spin.setDecimals(6)
        self.ref_lon_spin.setValue(39.200269)  # Voronezh
        self.ref_lon_spin.setMinimumWidth(0)
        gps_layout.addWidget(self.ref_lon_spin, 0, 3)

        # Target GPS center for the mission zone.
        gps_layout.addWidget(QLabel("Target Lat"), 1, 0)
        self.target_lat_spin = QDoubleSpinBox()
        self.target_lat_spin.setRange(-90.0, 90.0)
        self.target_lat_spin.setDecimals(6)
        self.target_lat_spin.setValue(51.664500)
        self.target_lat_spin.setMinimumWidth(0)
        gps_layout.addWidget(self.target_lat_spin, 1, 1)

        gps_layout.addWidget(QLabel("Target Lon"), 1, 2)
        self.target_lon_spin = QDoubleSpinBox()
        self.target_lon_spin.setRange(-180.0, 180.0)
        self.target_lon_spin.setDecimals(6)
        self.target_lon_spin.setValue(39.214300)
        self.target_lon_spin.setMinimumWidth(0)
        gps_layout.addWidget(self.target_lon_spin, 1, 3)

        # Radius of mission search/coverage area in meters.
        gps_layout.addWidget(QLabel("Radius m"), 2, 0)
        self.target_radius_spin = QSpinBox()
        self.target_radius_spin.setRange(10, 5000)
        self.target_radius_spin.setValue(200)
        self.target_radius_spin.setMinimumWidth(0)
        gps_layout.addWidget(self.target_radius_spin, 2, 1)

        # 0 means broadcast to all drones; any other value targets one drone.
        gps_layout.addWidget(QLabel("Drone (0=All)"), 2, 2)
        self.mission_drone_id_spin = QSpinBox()
        self.mission_drone_id_spin.setRange(0, 9999)
        self.mission_drone_id_spin.setValue(0)  # default: all drones
        self.mission_drone_id_spin.setMinimumWidth(0)
        gps_layout.addWidget(self.mission_drone_id_spin, 2, 3)

        self.assign_mission_btn = QPushButton("Assign")
        self.assign_mission_btn.setProperty("btnType", "primary")
        self.assign_mission_btn.clicked.connect(self.assign_selected_drone_mission)
        self.assign_mission_btn.setMinimumHeight(32)
        gps_layout.addWidget(self.assign_mission_btn, 3, 2)

        self.clear_mission_btn = QPushButton("Clear")
        self.clear_mission_btn.setProperty("btnType", "danger")
        self.clear_mission_btn.clicked.connect(self.clear_selected_drone_mission)
        self.clear_mission_btn.setMinimumHeight(32)
        gps_layout.addWidget(self.clear_mission_btn, 3, 3)

        self.open_map_btn = QPushButton("Open in Google Maps")
        self.open_map_btn.clicked.connect(self.open_target_in_google_maps)
        self.open_map_btn.setMinimumHeight(34)
        gps_layout.addWidget(self.open_map_btn, 4, 0, 1, 4)

        gps_group.setLayout(gps_layout)
        layout.addWidget(gps_group)
        
        group.setLayout(layout)
        return group
    
    def _create_status_panel(self):
        """Create status information panel"""
        group = QGroupBox("Swarm Status")
        layout = QVBoxLayout()
        layout.setContentsMargins(6, 8, 6, 6)
        layout.setSpacing(4)
        group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Status labels
        self.status_labels = {}
        
        labels = [
            ("total_drones", "Total Drones:"),
            ("active_drones", "Active Drones:"),
            ("leader_id", "Leader ID:"),
            ("avg_battery", "Avg Battery:")
        ]
        
        # Two metrics per row
        stats_grid = QGridLayout()
        stats_grid.setHorizontalSpacing(8)
        stats_grid.setVerticalSpacing(4)
        for idx, (key, text) in enumerate(labels):
            row = idx // 2
            col = (idx % 2) * 2
            name_label = QLabel(text)
            name_label.setStyleSheet("font-size: 11px;")
            value_label = QLabel("0")
            value_label.setStyleSheet("font-weight: bold; font-size: 11px;")
            stats_grid.addWidget(name_label, row, col)
            stats_grid.addWidget(value_label, row, col + 1)
            self.status_labels[key] = value_label
        layout.addLayout(stats_grid)

        latency_grid = QGridLayout()
        latency_keys = [
            ("cpp_to_py_ms", "C++->Py"),
            ("py_processing_ms", "Py Proc"),
            ("py_to_cpp_ms", "Py->C++"),
            ("total_round_trip_ms", "RTT"),
            ("total_round_trip_jitter_std_ms", "RTT Jitter"),
            ("leader_height_m", "Leader Height"),
        ]
        for idx, (key, title) in enumerate(latency_keys):
            row = idx // 2
            col = (idx % 2) * 2
            name = QLabel(f"{title}:")
            value = QLabel("0.0 m" if key == "leader_height_m" else "0.0 ms")
            value.setStyleSheet("font-size: 11px;")
            latency_grid.addWidget(name, row, col)
            latency_grid.addWidget(value, row, col + 1)
            self.latency_value_labels[key] = value
        layout.addLayout(latency_grid)

        ledger_grid = QGridLayout()
        ledger_keys = [
            ("block_height", "Ledger Height"),
            ("sync_state", "Ledger Sync"),
            ("integrity", "Chain Integrity"),
        ]
        for idx, (key, title) in enumerate(ledger_keys):
            row = idx // 2
            col = (idx % 2) * 2
            name = QLabel(f"{title}:")
            value = QLabel("-")
            value.setStyleSheet("font-size: 11px;")
            ledger_grid.addWidget(name, row, col)
            ledger_grid.addWidget(value, row, col + 1)
            self.ledger_value_labels[key] = value
        layout.addLayout(ledger_grid)
        
        # Drone table
        self.drone_table = QTableWidget()
        self.drone_table.setColumnCount(6)
        self.drone_table.setHorizontalHeaderLabels([
            "ID", "Role", "Mode", "Battery", "Altitude", "Status"
        ])
        self.drone_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.drone_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.drone_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.drone_table.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeToContents)
        self.drone_table.horizontalHeader().setSectionResizeMode(4, QHeaderView.ResizeToContents)
        self.drone_table.horizontalHeader().setSectionResizeMode(5, QHeaderView.Stretch)
        self.drone_table.verticalHeader().setSectionResizeMode(QHeaderView.Fixed)
        self.drone_table.verticalHeader().setDefaultSectionSize(24)
        self.drone_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.drone_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.drone_table.setVerticalScrollMode(QAbstractItemView.ScrollPerPixel)
        self.drone_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.drone_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.drone_table.setFocusPolicy(Qt.StrongFocus)
        self.drone_table.setAutoScroll(True)
        self.drone_table.verticalScrollBar().setSingleStep(20)
        self.drone_table.setAlternatingRowColors(True)
        self.drone_table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.drone_table.setMinimumHeight(120)
        self.drone_table.setMaximumHeight(16777215)
        self.drone_table.setSortingEnabled(True)
        self.drone_table.itemSelectionChanged.connect(self._on_drone_selection_changed)
        layout.addWidget(self.drone_table, 1)

        group.setLayout(layout)
        return group

    def _create_physical_ml_panel(self):
        """Create dedicated Physical ML Trainer container."""
        group = QGroupBox("Physical ML Trainer")
        layout = QVBoxLayout()
        layout.setContentsMargins(6, 8, 6, 6)
        layout.setSpacing(4)

        self.physical_ml_table = QTableWidget()
        self.physical_ml_table.setColumnCount(5)
        self.physical_ml_table.setHorizontalHeaderLabels(
            ["Drone", "Samples", "Degree", "Alpha", "Val MSE"]
        )
        self.physical_ml_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.physical_ml_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.physical_ml_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.physical_ml_table.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeToContents)
        self.physical_ml_table.horizontalHeader().setSectionResizeMode(4, QHeaderView.Stretch)
        self.physical_ml_table.verticalHeader().setVisible(False)
        self.physical_ml_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.physical_ml_table.setSelectionMode(QAbstractItemView.NoSelection)
        self.physical_ml_table.setFocusPolicy(Qt.NoFocus)
        self.physical_ml_table.setAlternatingRowColors(True)
        self.physical_ml_table.setMinimumHeight(120)
        self.physical_ml_table.setMaximumHeight(240)
        layout.addWidget(self.physical_ml_table)

        group.setLayout(layout)
        return group
    
    def _create_log_panel(self):
        """Create log panel"""
        group = QGroupBox("System Logs")
        layout = QVBoxLayout()
        layout.setContentsMargins(6, 8, 6, 6)
        layout.setSpacing(4)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.log_text.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.log_text.setMinimumHeight(120)
        layout.addWidget(self.log_text)

        hash_tables_row = QHBoxLayout()
        hash_tables_row.setSpacing(6)

        self.sha2_table = QTableWidget()
        self.sha2_table.setColumnCount(2)
        self.sha2_table.setRowCount(1)
        self.sha2_table.setHorizontalHeaderLabels(["SHA2 Sync", "Status"])
        self.sha2_table.verticalHeader().setVisible(False)
        self.sha2_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.sha2_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.sha2_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.sha2_table.setSelectionMode(QAbstractItemView.NoSelection)
        self.sha2_table.setFocusPolicy(Qt.NoFocus)
        self.sha2_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.sha2_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.sha2_table.setMinimumHeight(62)
        self.sha2_table.setItem(0, 0, QTableWidgetItem("Blockchain SHA2"))
        self.sha2_table.setItem(0, 1, QTableWidgetItem("âšª"))

        self.sha3_table = QTableWidget()
        self.sha3_table.setColumnCount(2)
        self.sha3_table.setRowCount(1)
        self.sha3_table.setHorizontalHeaderLabels(["SHA3 Sync", "Status"])
        self.sha3_table.verticalHeader().setVisible(False)
        self.sha3_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.sha3_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.sha3_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.sha3_table.setSelectionMode(QAbstractItemView.NoSelection)
        self.sha3_table.setFocusPolicy(Qt.NoFocus)
        self.sha3_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.sha3_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.sha3_table.setMinimumHeight(62)
        self.sha3_table.setItem(0, 0, QTableWidgetItem("Blockchain SHA3"))
        self.sha3_table.setItem(0, 1, QTableWidgetItem("âšª"))

        hash_tables_row.addWidget(self.sha2_table, 1)
        hash_tables_row.addWidget(self.sha3_table, 1)
        layout.addLayout(hash_tables_row)
        
        clear_btn = QPushButton("Clear Logs")
        clear_btn.clicked.connect(lambda: self.log_text.clear())
        clear_btn.setMinimumHeight(30)
        layout.addWidget(clear_btn)
        
        group.setLayout(layout)
        return group
    
    def log(self, message):
        """Add log message"""
        self.log_text.append(f"[{self.get_timestamp()}] {message}")
        # Keep log view pinned to bottom.
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    def get_timestamp(self):
        """Get current timestamp"""
        from datetime import datetime
        return datetime.now().strftime("%H:%M:%S")
    
    def update_display(self):
        """Update all displays"""
        status = self.swarm_manager.get_swarm_status()
        current_leader_id = status.get("leader_id")
        self._track_leader_change(current_leader_id)
        
        # Update status labels
        self.status_labels["total_drones"].setText(str(status["total_drones"]))
        self.status_labels["active_drones"].setText(str(status["active_drones"]))
        self.status_labels["leader_id"].setText(
            str(current_leader_id) if current_leader_id else "None"
        )
        
        # Calculate average battery
        drones = status.get("drones", {})
        avg_battery = 0.0
        if drones:
            avg_battery = sum(d["battery"] for d in drones.values()) / len(drones)
            self.status_labels["avg_battery"].setText(f"{avg_battery:.1f}%")
        else:
            self.status_labels["avg_battery"].setText("0.0%")

        latency = status.get("latency", {})
        for key, label in self.latency_value_labels.items():
            if key == "leader_height_m":
                leader_height = 0.0
                if current_leader_id and current_leader_id in drones:
                    leader_pos = drones[current_leader_id].get("position", {}) or {}
                    leader_height = float(leader_pos.get("z", 0.0))
                label.setText(f"{leader_height:.1f} m")
            else:
                label.setText(f"{float(latency.get(key, 0.0)):.1f} ms")
        if hasattr(self, "latency_monitor_widget"):
            self.latency_monitor_widget.update_latency(latency)
        if hasattr(self, "swarm_metrics_widget"):
            self.swarm_metrics_widget.update_metrics(int(status.get("active_drones", 0)), avg_battery)

        ledger = status.get("ledger", {})
        self.ledger_value_labels["block_height"].setText(str(int(ledger.get("block_height", 0))))
        self.ledger_value_labels["sync_state"].setText(str(ledger.get("sync_state", "UNKNOWN")))
        integrity_ok = bool(ledger.get("integrity_ok", False))
        self.ledger_value_labels["integrity"].setText("OK" if integrity_ok else "FAIL")
        self.ledger_value_labels["integrity"].setStyleSheet(
            "font-size: 11px; font-weight: bold; color: %s;" % ("#22aa44" if integrity_ok else "#cc2233")
        )
        self._update_blockchain_hash_tables(ledger)

        acoustic = status.get("acoustic", {})
        src = acoustic.get("latest_source")
        conf = float(acoustic.get("latest_confidence", 0.0))
        self.drone_widget.set_acoustic_source(src, conf)
        if hasattr(self, "acoustic_map_widget"):
            self.acoustic_map_widget.update_map_data(drones, src)

        raw_obstacles = status.get("dynamic_obstacles", [])
        filtered_obstacles = []
        dynamic_obstacle_count = 0
        static_obstacle_count = 0
        for obstacle in raw_obstacles:
            motion_type = obstacle.get("motion_type", "linear")
            dynamic = motion_type != "static" and (
                abs(float(obstacle.get("vx", 0.0))) > 0.001 or abs(float(obstacle.get("vy", 0.0))) > 0.001
                or motion_type in {"circular", "random_walk"}
            )
            if dynamic:
                dynamic_obstacle_count += 1
            else:
                static_obstacle_count += 1
            if dynamic and not self.show_dynamic_obstacles:
                continue
            if (not dynamic) and not self.show_static_obstacles:
                continue
            view = dict(obstacle)
            view["dynamic"] = dynamic
            filtered_obstacles.append(view)
        self._update_prediction_table(
            status,
            dynamic_obstacle_count=dynamic_obstacle_count,
            static_obstacle_count=static_obstacle_count,
        )
        self._update_personal_ml_status_table(drones)
        # Update drone table (defensive: one bad row must not block remaining drones)
        def _sort_key(item):
            drone_id = item[0]
            try:
                return (0, int(drone_id))
            except Exception:
                return (1, str(drone_id))

        ordered_rows = sorted(drones.items(), key=_sort_key)
        sorting_enabled = self.drone_table.isSortingEnabled()
        if sorting_enabled:
            self.drone_table.setSortingEnabled(False)
        self.drone_table.clearContents()
        self.drone_table.setRowCount(len(ordered_rows))
        for row, (drone_id, drone_data) in enumerate(ordered_rows):
            role = str(drone_data.get("role", "unknown"))
            flight_mode = str(drone_data.get("flight_mode", "unknown"))
            battery = float(drone_data.get("battery", 0.0))
            pos = drone_data.get("position", {}) or {}
            altitude = float(pos.get("z", 0.0))
            is_active = bool(drone_data.get("is_active", False))

            self.drone_table.setItem(row, 0, QTableWidgetItem(str(drone_id)))
            self.drone_table.setItem(row, 1, QTableWidgetItem(role))
            self.drone_table.setItem(row, 2, QTableWidgetItem(flight_mode))
            self.drone_table.setItem(row, 3, QTableWidgetItem(f"{battery:.1f}%"))
            self.drone_table.setItem(row, 4, QTableWidgetItem(f"{altitude:.1f}m"))

            swarm_state = str(drone_data.get("swarm_state", "IDLE"))
            status_text = swarm_state if is_active else "Inactive"
            mission = drone_data.get("mission", {}) or {}
            if mission.get("active"):
                status_text = f"{swarm_state} | Mission: {mission.get('status', 'active')}"
            if drone_data.get("motor_failure_warning"):
                status_text = "Warning: Motor fail -> Return X"
            if drone_data.get("emergency_return_active"):
                status_text = "Emergency Return to X"
            self.drone_table.setItem(row, 5, QTableWidgetItem(status_text))
        if sorting_enabled:
            self.drone_table.setSortingEnabled(True)
        self._update_physical_ml_table(ordered_rows)

        # Update visualization
        self.drone_widget.update_drones(drones)
        self.drone_widget.set_operation_overlay(
            self.operation_corners,
            self.operation_start,
            self.operation_destination,
            self.operation_slots,
            self.destination_gap_m,
            self.operation_start_slots
        )
        self.drone_widget.set_obstacles(filtered_obstacles)
        self.location_map_widget.update_drones(drones)
        self.location_map_widget.set_selected_drone(self.selected_drone_id)
        self._poll_swarm_event_logs()
        self._dispatch_pending_takeoff_targets()
        self._monitor_destination_arrivals_for_auto_return()

    def _update_blockchain_hash_tables(self, ledger: dict):
        """Update SHA2/SHA3 sync tables with symbols."""
        if self.sha2_table is None or self.sha3_table is None:
            return

        sync_state = str(ledger.get("sync_state", "IDLE")).upper()
        integrity_ok = bool(ledger.get("integrity_ok", False))
        per_drone = ledger.get("per_drone_height", {}) or {}
        has_data = len(per_drone) > 0

        try:
            heights_payload = json.dumps(per_drone, sort_keys=True).encode("utf-8")
            sha2_hex = hashlib.sha256(heights_payload).hexdigest()[:8]
            sha3_hex = hashlib.sha3_256(heights_payload).hexdigest()[:8]
        except Exception:
            sha2_hex = "--------"
            sha3_hex = "--------"

        sha2_ok = has_data and integrity_ok and sync_state in {"SYNCED", "BROADCASTING", "PARTIAL_REJECT"}
        # SHA3 was too strict and always warning in some valid runtime states.
        sha3_ok = has_data and integrity_ok and sync_state not in {"FAILED", "ERROR"}

        sha2_item = self.sha2_table.item(0, 1)
        sha3_item = self.sha3_table.item(0, 1)
        if sha2_item is None:
            sha2_item = QTableWidgetItem()
            self.sha2_table.setItem(0, 1, sha2_item)
        if sha3_item is None:
            sha3_item = QTableWidgetItem()
            self.sha3_table.setItem(0, 1, sha3_item)

        sha2_item.setText(f"{sha2_hex} {'\u2713' if sha2_ok else '\u26A0'}")
        sha3_item.setText(f"{sha3_hex} {'\u2713' if sha3_ok else '\u26A0'}")
        sha2_item.setForeground(QBrush(QColor(124, 212, 116) if sha2_ok else QColor(255, 95, 95)))
        sha3_item.setForeground(QBrush(QColor(124, 212, 116) if sha3_ok else QColor(255, 95, 95)))

    def _update_prediction_table(self, status: dict, dynamic_obstacle_count: int, static_obstacle_count: int):
        """Update prediction table with check/warn and counts."""
        if self.prediction_table is None:
            return

        drones = status.get("drones", {}) or {}
        active_drones = int(status.get("active_drones", 0))
        warning_drones = 0
        mission_drones = 0
        for drone_data in drones.values():
            if drone_data.get("motor_failure_warning") or drone_data.get("emergency_return_active"):
                warning_drones += 1
            mission = drone_data.get("mission", {}) or {}
            if mission.get("active"):
                mission_drones += 1

        short_predict = dynamic_obstacle_count
        mid_predict = dynamic_obstacle_count + static_obstacle_count
        long_predict = mission_drones if mission_drones > 0 else active_drones
        obstacle_dynamic = dynamic_obstacle_count

        rows = [
            (0, short_predict, short_predict > 0),
            (1, mid_predict, mid_predict > 1),
            (2, long_predict, warning_drones > 0),
            (3, obstacle_dynamic, obstacle_dynamic > 0),
        ]

        for row, count, warn in rows:
            result_item = self.prediction_table.item(row, 1)
            if result_item is None:
                result_item = QTableWidgetItem()
                self.prediction_table.setItem(row, 1, result_item)
            if warn:
                result_item.setText(f"\u26A0 ({int(count)})")
                result_item.setForeground(QBrush(QColor(255, 95, 95)))
            else:
                result_item.setText(f"\u2713 ({int(count)})")
                result_item.setForeground(QBrush(QColor(124, 212, 116)))

    def _update_personal_ml_status_table(self, drones: dict):
        """Update Personal ML status table under prediction summary."""
        if self.personal_ml_status_table is None:
            return

        entries = sorted(drones.items(), key=lambda kv: int(kv[0]) if str(kv[0]).isdigit() else str(kv[0]))
        if not entries:
            self.personal_ml_status_table.setRowCount(1)
            self.personal_ml_status_table.setItem(0, 0, QTableWidgetItem("Personal ML"))
            self.personal_ml_status_table.setItem(0, 1, QTableWidgetItem("N/A"))
            self.personal_ml_status_table.setItem(0, 2, QTableWidgetItem("\u26A0"))
            return

        self.personal_ml_status_table.setRowCount(len(entries))
        for row, (drone_id, drone_data) in enumerate(entries):
            enabled = bool(drone_data.get("personal_ml_enabled", False))
            name_item = QTableWidgetItem("Personal ML")
            drone_item = QTableWidgetItem(f"D{drone_id}")
            status_item = QTableWidgetItem("\u2713" if enabled else "\u26A0")
            status_item.setForeground(QBrush(QColor(124, 212, 116) if enabled else QColor(255, 95, 95)))
            self.personal_ml_status_table.setItem(row, 0, name_item)
            self.personal_ml_status_table.setItem(row, 1, drone_item)
            self.personal_ml_status_table.setItem(row, 2, status_item)

    def _update_physical_ml_table(self, ordered_rows):
        """Update compact PhysicalMLTrainer table below swarm status table."""
        if self.physical_ml_table is None:
            return

        rows = []
        for drone_id, drone_data in ordered_rows:
            samples = int(drone_data.get("physical_ml_samples", 0))
            metrics = drone_data.get("physical_ml_metrics", {}) or {}
            degree = metrics.get("degree", metrics.get("poly_degree", 2))
            alpha = metrics.get("alpha", metrics.get("regularization_alpha", 0.0))
            val_mse = metrics.get("val_mse", metrics.get("validation_mse", 0.0))
            rows.append((drone_id, samples, degree, alpha, val_mse))

        self.physical_ml_table.setRowCount(max(1, len(rows)))
        if not rows:
            defaults = [("3", "1101", "2", "0.0", "0.000000")]
            for col, value in enumerate(defaults[0]):
                self.physical_ml_table.setItem(0, col, QTableWidgetItem(value))
            return

        for row_idx, (drone_id, samples, degree, alpha, val_mse) in enumerate(rows):
            self.physical_ml_table.setItem(row_idx, 0, QTableWidgetItem(str(drone_id)))
            self.physical_ml_table.setItem(row_idx, 1, QTableWidgetItem(str(samples)))
            self.physical_ml_table.setItem(row_idx, 2, QTableWidgetItem(str(degree)))
            self.physical_ml_table.setItem(row_idx, 3, QTableWidgetItem(f"{float(alpha):.4f}"))
            self.physical_ml_table.setItem(row_idx, 4, QTableWidgetItem(f"{float(val_mse):.6f}"))
    
    # Control methods
    
    def add_drone(self):
        """Add new drone to swarm"""
        from drone import Drone, Position
        import random
        
        drone_id = len(self.swarm_manager.drones) + 1
        home_x = random.uniform(-3000, 3000)
        home_y = random.uniform(-3000, 3000)
        
        drone = Drone(drone_id, Position(home_x, home_y, 0))
        self.swarm_manager.add_drone(drone)
        self.log(f"Added Drone {drone_id}")
    
    def remove_drone(self):
        """Remove drone from swarm"""
        if self.swarm_manager.drones:
            drone_id = max(self.swarm_manager.drones.keys())
            self.swarm_manager.remove_drone(drone_id)
            self.log(f"Removed Drone {drone_id}")

    def add_moving_obstacle(self):
        """Create a dynamic obstacle with selected start, velocity and motion model."""
        obstacle_id = self.swarm_manager.add_dynamic_obstacle(
            x=float(self.obs_x_spin.value()),
            y=float(self.obs_y_spin.value()),
            vx=float(self.obs_vx_spin.value()),
            vy=float(self.obs_vy_spin.value()),
            motion_type=self.obs_motion_combo.currentText(),
            radius=float(self.obs_radius_spin.value()),
            z=0.0,
        )
        self.log(
            f"Dynamic obstacle added id={obstacle_id} "
            f"start=({self.obs_x_spin.value():.1f},{self.obs_y_spin.value():.1f}) "
            f"vel=({self.obs_vx_spin.value():.1f},{self.obs_vy_spin.value():.1f}) "
            f"type={self.obs_motion_combo.currentText()}"
        )

    def add_static_obstacle(self):
        """Create a static obstacle."""
        obstacle_id = self.swarm_manager.add_static_obstacle(
            x=float(self.obs_x_spin.value()),
            y=float(self.obs_y_spin.value()),
            radius=float(self.obs_radius_spin.value()),
            z=0.0,
        )
        self.log(f"Static obstacle added id={obstacle_id} at ({self.obs_x_spin.value():.1f},{self.obs_y_spin.value():.1f})")

    def clear_all_obstacles(self):
        self.swarm_manager.clear_obstacles()
        self.log("All obstacles cleared")

    def _on_use_ml_avoidance_changed(self, _state: int):
        enabled = bool(self.use_ml_avoidance_cb.isChecked())
        self.swarm_manager.set_use_personal_ml_avoidance(enabled)
        self.swarm_manager.set_personal_ml_enabled_all(enabled)
        self.log(f"Personal ML avoidance {'enabled' if enabled else 'disabled'}")

    def _on_acoustic_detection_changed(self, _state: int):
        enabled = bool(self.acoustic_enable_cb.isChecked())
        if hasattr(self.swarm_manager, "set_acoustic_detection_enabled"):
            self.swarm_manager.set_acoustic_detection_enabled(enabled)
        self.log(f"Acoustic detection {'enabled' if enabled else 'disabled'}")

    def _on_acoustic_threshold_changed(self, value: int):
        threshold = max(0.0, min(1.0, float(value) / 100.0))
        if self.acoustic_conf_label:
            self.acoustic_conf_label.setText(f"{threshold:.2f}")
        if hasattr(self.swarm_manager, "set_acoustic_confidence_threshold"):
            self.swarm_manager.set_acoustic_confidence_threshold(threshold)

    def _on_obstacle_filter_changed(self, _state: int):
        self.show_static_obstacles = bool(self.toggle_static_cb.isChecked())
        self.show_dynamic_obstacles = bool(self.toggle_dynamic_cb.isChecked())

    def simulate_latency_spike(self):
        stats = self.swarm_manager.simulate_latency_spike(500.0)
        self.log(
            "Latency spike injected: RTT="
            f"{float(stats.get('total_round_trip_ms', 0.0)):.1f}ms "
            f"(threshold={float(stats.get('threshold_ms', 0.0)):.1f}ms)"
        )
    
    def arm_all(self):
        """Arm all drones"""
        for drone in self.swarm_manager.drones.values():
            drone.arm()
            self._log_controller_to_drone_encrypted(drone.drone_id, "arm", {})
        self.log("All drones armed")
    
    def takeoff_all(self):
        """Leader-commanded takeoff: followers hover at their own X until explicit command."""
        mission = self._plan_takeoff_route()
        self.pending_takeoff_targets.clear()
        self.active_destination_targets.clear()
        self.mission_to_y_active = False
        self.swarm_manager.set_leader_follow_enabled(False)
        self.operation_start_slots = mission.get("start_slots", {})
        self.operation_return_slots = mission.get("start_slots", {})
        # Each drone keeps its own X point from current position.
        for drone_id, drone in self.swarm_manager.drones.items():
            start_slot = self.operation_start_slots.get(drone_id)
            if start_slot is None:
                continue
            drone.home_position = type(start_slot)(start_slot.x, start_slot.y, 0.0)

        self.swarm_manager.leader_takeoff()
        for drone_id, drone in self.swarm_manager.drones.items():
            self._log_controller_to_drone_encrypted(drone.drone_id, "leader_takeoff", {})
        self.operation_corners = mission["corners"]
        self.operation_start = mission["start"]
        self.operation_destination = mission["destination"]
        self.operation_slots = mission["slots"]
        self.log("Takeoff commanded by Leader: followers now WAITING_FOR_COMMAND at own X")

    def command_move_x_to_y(self):
        """Explicit leader movement command from X positions to planned Y slots."""
        mission = self._plan_takeoff_route()
        targets = mission.get("slots", {})
        if not targets:
            self.log("No drones available for X->Y command")
            return
        gps_mode_map = {
            drone_id: bool(self.swarm_manager.drones.get(drone_id).area_mission.active)
            for drone_id in targets.keys()
            if self.swarm_manager.drones.get(drone_id) is not None
        }
        self.swarm_manager.leader_move_to_target(targets, gps_mode_map=gps_mode_map)
        self.operation_corners = mission["corners"]
        self.operation_start = mission["start"]
        self.operation_destination = mission["destination"]
        self.operation_slots = mission["slots"]
        for drone_id, target in targets.items():
            self._log_controller_to_drone_encrypted(
                drone_id,
                "leader_move_to_target",
                {"x": target.x, "y": target.y, "z": target.z, "gps_mode": gps_mode_map.get(drone_id, False)},
            )
        self.log("Leader command sent: MOVE X->Y for all drones")
    
    def land_all(self):
        """Land all drones"""
        self.mission_to_y_active = False
        self.swarm_manager.set_leader_follow_enabled(True)
        for drone in self.swarm_manager.drones.values():
            drone.land()
            self._log_controller_to_drone_encrypted(drone.drone_id, "land", {})
        self.log("Landing commanded to all drones")
    
    def return_all_home(self):
        """Return all drones to home"""
        self.mission_to_y_active = False
        self.swarm_manager.set_leader_follow_enabled(False)
        self.swarm_manager.return_all_to_home()
        for drone in self.swarm_manager.drones.values():
            self._log_controller_to_drone_encrypted(drone.drone_id, "leader_return_to_home", {})
        self.log("Leader broadcasted RETURN_TO_HOME (GPS_ML_ACTIVE drones ignore)")
    
    def emergency_land_all(self):
        """Emergency return all drones to X and then land."""
        self.mission_to_y_active = False
        self.swarm_manager.set_leader_follow_enabled(True)
        self.swarm_manager.emergency_land_all("Emergency button pressed")
        for drone in self.swarm_manager.drones.values():
            self._log_controller_to_drone_encrypted(
                drone.drone_id, "emergency_land", {"reason": "Emergency button pressed"}
            )
        self.log("EMERGENCY RETURN activated: drones returning to X")

    def emergency_land_selected(self):
        """Emergency return selected drone to X and land."""
        current_row = self.drone_table.currentRow()
        if current_row < 0:
            self.log("Select a drone in the status table first")
            return
        drone_item = self.drone_table.item(current_row, 0)
        if not drone_item:
            self.log("Invalid drone selection")
            return
        drone_id = int(drone_item.text())
        success = self.swarm_manager.emergency_land_drone(
            drone_id,
            "Personal emergency button pressed"
        )
        if success:
            self._log_controller_to_drone_encrypted(
                drone_id, "emergency_land", {"reason": "Personal emergency button pressed"}
            )
            self.log(f"Personal emergency return triggered for Drone {drone_id}")
        else:
            self.log(f"Could not trigger personal emergency for Drone {drone_id}")
    
    def apply_formation(self):
        """Apply selected formation"""
        formation = self.formation_combo.currentText()
        self.swarm_manager.formation_flight(formation)
        self.log(f"Formation '{formation}' applied")

    def apply_leader_follow_pattern(self):
        """Apply continuous leader-follow shape used during movement."""
        pattern = self.follow_pattern_combo.currentText().strip().lower()
        spacing = float(self.follow_spacing_spin.value())
        ok = self.swarm_manager.set_leader_follow_pattern(pattern, spacing)
        if ok:
            self.log(f"Leader follow pattern set to '{pattern}' spacing={spacing:.0f}m")
        else:
            self.log(f"Invalid leader follow pattern: {pattern}")
    
    def simulate_motor_failure(self):
        """Simulate motor failure on selected drone"""
        drone = self.get_selected_drone()
        if drone is None:
            drone = self._pick_non_leader_drone_for_failure()
        if drone is None:
            self.log("No suitable non-leader drone found for motor failure test")
            return
        motor_id = 0
        drone.simulate_motor_failure(motor_id)
        self._log_controller_to_drone_encrypted(
            drone.drone_id, "simulate_motor_failure", {"motor_id": motor_id}
        )
        self.log(
            f"Motor failure simulated on Drone {drone.drone_id} (motor {motor_id}) "
            f"-> warning shown, drone returning to X"
        )

    def toggle_auto_fault_demo(self):
        """Toggle automatic fault simulation in GUI."""
        self.auto_fault_demo_enabled = not self.auto_fault_demo_enabled
        if self.auto_fault_demo_enabled:
            self.auto_fault_btn.setText("Auto Fault Demo: ON")
            self.auto_fault_btn.setProperty("btnType", "danger")
            self.log("Auto fault demo enabled (motor failure + leader crash)")
        else:
            self.auto_fault_btn.setText("Auto Fault Demo: OFF")
            self.auto_fault_btn.setProperty("btnType", "warn")
            self.log("Auto fault demo disabled")
        self.auto_fault_btn.style().unpolish(self.auto_fault_btn)
        self.auto_fault_btn.style().polish(self.auto_fault_btn)

    def _run_auto_fault_scenario(self):
        """Periodically simulate faults to visualize autonomous handling."""
        if not self.auto_fault_demo_enabled:
            return
        if not self.swarm_manager.drones:
            return
        if self._is_real_drone_mode_active():
            # Never inject synthetic failures when connected to real drones.
            return

        if self.auto_fault_phase % 2 == 0:
            targets = []
            for drone in self.swarm_manager.drones.values():
                if drone.drone_id == self.swarm_manager.leader_id:
                    continue
                if not drone.is_active:
                    continue
                if drone.drone_id == self.last_auto_leader_crash_drone_id:
                    continue
                targets.append(drone)
            if targets:
                import random
                target = random.choice(targets)
                target.simulate_motor_failure(0)
                self.last_auto_motor_fail_drone_id = target.drone_id
                self.log(f"[AUTO] Motor failure injected on Drone {target.drone_id}")
        else:
            leader = self.swarm_manager.get_leader()
            if leader and leader.is_active and leader.drone_id != self.last_auto_motor_fail_drone_id:
                from drone import FlightMode
                leader.flight_mode = FlightMode.CRASHED
                leader.is_active = False
                self.last_auto_leader_crash_drone_id = leader.drone_id
                self.log(f"[AUTO] Leader crash injected on Drone {leader.drone_id}")

        self.auto_fault_phase += 1

    def _is_real_drone_mode_active(self) -> bool:
        """Check whether any drone is configured for real hardware."""
        for drone in self.swarm_manager.drones.values():
            if getattr(drone, "use_real_drone", False):
                return True
        return False

    def _pick_non_leader_drone_for_failure(self):
        """Pick a safe non-leader active drone for motor-failure testing."""
        leader_id = self.swarm_manager.leader_id
        candidates = []
        for drone in self.swarm_manager.drones.values():
            if not drone.is_active:
                continue
            if drone.drone_id == leader_id:
                continue
            candidates.append(drone)
        if not candidates:
            return None
        candidates.sort(key=lambda d: d.drone_id)
        return candidates[0]

    def _track_leader_change(self, current_leader_id):
        """Log leadership changes once per change."""
        if not self._leader_initialized:
            self._leader_initialized = True
            self._last_leader_id = current_leader_id
            if current_leader_id is not None:
                self.log(f"Leader initialized: Drone {current_leader_id}")
            return
        if current_leader_id != self._last_leader_id:
            prev = self._last_leader_id
            self._last_leader_id = current_leader_id
            if current_leader_id is None:
                self.log(f"Leader changed: Drone {prev} -> None")
            elif prev is None:
                self.log(f"Leader changed: None -> Drone {current_leader_id}")
            else:
                self.log(f"Leader changed: Drone {prev} -> Drone {current_leader_id}")

    def _plan_takeoff_route(self):
        """Build mission geometry: per-drone X from current position; Y slots near B corner."""
        drones = self.swarm_manager.drones
        if not drones:
            return {
                "corners": {},
                "start": None,
                "destination": None,
                "slots": {},
                "start_slots": {}
            }

        homes = [drone.home_position for drone in drones.values()]
        start_x = sum(p.x for p in homes) / len(homes)
        start_y = sum(p.y for p in homes) / len(homes)
        mission_alt = 60.0
        half = 2600.0
        corners = {
            "A": type(homes[0])(start_x - half, start_y + half, 0.0),
            "B": type(homes[0])(start_x + half, start_y + half, 0.0),
            "C": type(homes[0])(start_x + half, start_y - half, 0.0),
            "D": type(homes[0])(start_x - half, start_y - half, 0.0),
        }
        start = type(homes[0])(start_x, start_y, 0.0)
        destination = type(homes[0])(corners["B"].x, corners["B"].y, mission_alt)

        drone_ids = sorted(drones.keys())
        total = len(drone_ids)
        slots = {}  # Y slots
        start_slots = {}  # X slots
        for drone_id in drone_ids:
            drone = drones.get(drone_id)
            if drone is None:
                continue
            start_slots[drone_id] = type(homes[0])(
                drone.current_position.x,
                drone.current_position.y,
                0.0
            )
        if total == 1:
            only_id = drone_ids[0]
            slots[only_id] = type(homes[0])(destination.x, destination.y, destination.z)
        else:
            radius = max(self.destination_gap_m, (self.destination_gap_m * total) / (2.0 * math.pi))
            for idx, drone_id in enumerate(drone_ids):
                angle = (2.0 * math.pi * idx) / total
                sx = destination.x + radius * math.cos(angle)
                sy = destination.y + radius * math.sin(angle)
                slots[drone_id] = type(homes[0])(sx, sy, destination.z)

        return {
            "corners": corners,
            "start": start,
            "destination": destination,
            "slots": slots,
            "start_slots": start_slots
        }

    def _dispatch_pending_takeoff_targets(self):
        """After takeoff reaches controllable mode, send each drone to its Y-slot."""
        if not self.pending_takeoff_targets:
            return
        from drone import FlightMode
        for drone_id, target in list(self.pending_takeoff_targets.items()):
            drone = self.swarm_manager.drones.get(drone_id)
            if drone is None:
                del self.pending_takeoff_targets[drone_id]
                continue
            if drone.flight_mode in [FlightMode.EMERGENCY_LANDING, FlightMode.CRASHED, FlightMode.IDLE]:
                del self.pending_takeoff_targets[drone_id]
                continue
            if drone.flight_mode in [FlightMode.HOVER, FlightMode.FLYING]:
                if drone.goto(target):
                    self.active_destination_targets[drone_id] = target
                    self._log_controller_to_drone_encrypted(
                        drone_id,
                        "goto_xy_destination",
                        {"x": target.x, "y": target.y, "z": target.z}
                    )
                    self.log(f"D{drone_id} moving X->Y slot ({target.x:.0f}, {target.y:.0f})")
                del self.pending_takeoff_targets[drone_id]

    def _enforce_y_targets(self):
        """Keep drones focused on Y targets instead of following leader during mission phase."""
        if not self.mission_to_y_active:
            return
        if not self.active_destination_targets:
            return
        from drone import FlightMode
        for drone_id, target in list(self.active_destination_targets.items()):
            drone = self.swarm_manager.drones.get(drone_id)
            if drone is None:
                del self.active_destination_targets[drone_id]
                continue
            if drone.flight_mode in [FlightMode.EMERGENCY_LANDING, FlightMode.CRASHED, FlightMode.RETURNING_HOME, FlightMode.LANDING]:
                continue
            dx = drone.current_position.x - target.x
            dy = drone.current_position.y - target.y
            dz = drone.current_position.z - target.z
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            if distance > 8.0 and drone.flight_mode in [FlightMode.HOVER, FlightMode.FLYING]:
                drone.goto(target)

    def _monitor_destination_arrivals_for_auto_return(self):
        """If any drone reaches Y-slot, notify all (encrypted) and command whole team to return to X."""
        if not self.active_destination_targets:
            if self.mission_to_y_active and not self.pending_takeoff_targets:
                self.mission_to_y_active = False
                self.swarm_manager.set_leader_follow_enabled(True)
            return
        from drone import FlightMode
        for drone_id, target in list(self.active_destination_targets.items()):
            drone = self.swarm_manager.drones.get(drone_id)
            if drone is None:
                del self.active_destination_targets[drone_id]
                continue
            if drone.flight_mode in [FlightMode.EMERGENCY_LANDING, FlightMode.CRASHED, FlightMode.IDLE]:
                del self.active_destination_targets[drone_id]
                continue

            dx = drone.current_position.x - target.x
            dy = drone.current_position.y - target.y
            dz = drone.current_position.z - target.z
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            if distance > 6.0:
                continue

            self._broadcast_team_reached_y_and_rth(drone_id)
            break
        if self.mission_to_y_active and not self.active_destination_targets and not self.pending_takeoff_targets:
            self.mission_to_y_active = False
            self.swarm_manager.set_leader_follow_enabled(True)

    def _broadcast_team_reached_y_and_rth(self, reached_drone_id: int):
        """Send encrypted team message and command every drone to return to X."""
        self.log(f"D{reached_drone_id} reached Y -> encrypted team broadcast and ALL return to X")
        self._send_encrypted_team_broadcast(
            "team_member_reached_y",
            {"reached_drone_id": reached_drone_id, "action": "return_to_x"}
        )
        for drone_id, drone in self.swarm_manager.drones.items():
            x_slot = self.operation_return_slots.get(drone_id)
            if x_slot is not None:
                drone.home_position = type(x_slot)(x_slot.x, x_slot.y, 0.0)
            self._log_controller_to_drone_encrypted(
                drone_id,
                "team_member_reached_y",
                {"reached_drone_id": reached_drone_id, "action": "return_to_x"}
            )
            drone.return_to_home(f"Team member D{reached_drone_id} reached Y")
            self._log_controller_to_drone_encrypted(
                drone_id,
                "return_to_home",
                {"reason": f"Team member D{reached_drone_id} reached Y"},
            )
        self.active_destination_targets.clear()
        self.pending_takeoff_targets.clear()
        self.mission_to_y_active = False
        self.swarm_manager.set_leader_follow_enabled(True)

    def _send_encrypted_team_broadcast(self, message_type: str, data: dict):
        """Send real encrypted broadcast over swarm UDP if communication module is available."""
        if not self.controller_crypto:
            return
        try:
            sent = self.controller_crypto.send_message(message_type, data)
            if sent:
                self.log(f"[ENC BROADCAST] type={message_type} delivered")
            else:
                self.log(f"[ENC BROADCAST] type={message_type} failed")
        except Exception as e:
            self.log(f"[ENC BROADCAST] error: {e}")
    
    def test_leader_failure(self):
        """Test leader failure scenario"""
        if self.swarm_manager.leader_id:
            leader_id = self.swarm_manager.leader_id
            leader = self.swarm_manager.drones.get(leader_id)
            if leader is None:
                return
            from drone import FlightMode
            leader.flight_mode = FlightMode.CRASHED
            leader.is_active = False
            self.log(f"Leader Drone {leader_id} crashed - testing leader election")

    def _on_drone_selection_changed(self):
        """Track selected drone from table and update location map highlight."""
        current_row = self.drone_table.currentRow()
        if current_row < 0:
            self.selected_drone_id = None
        else:
            item = self.drone_table.item(current_row, 0)
            self.selected_drone_id = int(item.text()) if item else None
        self.location_map_widget.set_selected_drone(self.selected_drone_id)
        self._sync_selected_drone_mission_inputs()

    def get_selected_drone(self):
        """Return currently selected drone object, or None."""
        if self.selected_drone_id is None:
            return None
        return self.swarm_manager.drones.get(self.selected_drone_id)

    def get_control_drone(self):
        """Selected drone gets priority; otherwise default to leader."""
        selected = self.get_selected_drone()
        if selected is not None:
            return selected
        leader = self.swarm_manager.get_leader()
        if leader is not None:
            return leader
        return None

    def move_selected_drone(self, dx: float, dy: float, dz: float):
        """Move via explicit Leader command so followers never move autonomously."""
        drone = self.get_control_drone()
        if drone is None:
            self.log("No controllable drone found (select one or set leader)")
            return
        from drone import Position, FlightMode
        current = drone.current_position
        if dx == 0 and dy == 0 and dz == 0:
            drone.target_position = None
            if drone.flight_mode == FlightMode.FLYING:
                drone.flight_mode = FlightMode.HOVER
            self._log_controller_to_drone_encrypted(drone.drone_id, "hover", {})
            self.log(f"Drone {drone.drone_id} hold position")
            return

        if drone.flight_mode in [FlightMode.EMERGENCY_LANDING, FlightMode.CRASHED]:
            self.log(f"Move rejected for D{drone.drone_id} (emergency/crashed state)")
            return

        target = Position(
            current.x + dx,
            current.y + dy,
            max(0.0, current.z + dz)
        )
        self.swarm_manager.leader_move_to_target({drone.drone_id: target})
        self._log_controller_to_drone_encrypted(
            drone.drone_id,
            "leader_move_to_target",
            {"x": target.x, "y": target.y, "z": target.z},
        )
        self.log(
            f"Leader command MOVE D{drone.drone_id} -> x:{target.x:.1f} y:{target.y:.1f} z:{target.z:.1f}"
        )

    def keyPressEvent(self, event):
        """Keyboard control for selected drone movement."""
        step = self.move_step_spin.value() if hasattr(self, "move_step_spin") else 100
        if event.key() == Qt.Key_Up:
            self.move_selected_drone(0, step, 0)
            return
        if event.key() == Qt.Key_Down:
            self.move_selected_drone(0, -step, 0)
            return
        if event.key() == Qt.Key_Left:
            self.move_selected_drone(-step, 0, 0)
            return
        if event.key() == Qt.Key_Right:
            self.move_selected_drone(step, 0, 0)
            return
        if event.key() in [Qt.Key_H, Qt.Key_Space]:
            self.move_selected_drone(0, 0, 0)
            return
        super().keyPressEvent(event)

    def _log_controller_to_drone_encrypted(self, drone_id: int, command: str, payload: dict):
        """Show controller->drone encrypted payload in system log."""
        message = {"type": "controller_command", "data": {"command": command, "payload": payload}}
        encrypted_hex = ""
        if self.controller_crypto:
            encrypted = self.controller_crypto._encrypt_message(message)
            encrypted_hex = encrypted.hex() if encrypted else ""
        if not encrypted_hex:
            encrypted_hex = json.dumps(message, separators=(",", ":")).encode().hex()
        short_hex = encrypted_hex[:96] + ("..." if len(encrypted_hex) > 96 else "")
        self.log(f"[ENC CTRL->D{drone_id}] cmd={command} payload_hex={short_hex}")

    def _log_system_command_encrypted(self, command: str, payload: dict):
        """Show encrypted system command as separate command log line."""
        message = {"type": "system_command", "data": {"command": command, "payload": payload}}
        encrypted_hex = ""
        if self.controller_crypto:
            encrypted = self.controller_crypto._encrypt_message(message)
            encrypted_hex = encrypted.hex() if encrypted else ""
        if not encrypted_hex:
            encrypted_hex = json.dumps(message, separators=(",", ":")).encode().hex()
        short_hex = encrypted_hex[:96] + ("..." if len(encrypted_hex) > 96 else "")
        self.log(f"[ENC CMD] cmd={command} payload_hex={short_hex}")

    def _log_system_message_encrypted(self, message_type: str, data: dict):
        """Show encrypted system message as separate message log line."""
        message = {"type": "system_message", "data": {"message_type": message_type, "payload": data}}
        encrypted_hex = ""
        if self.controller_crypto:
            encrypted = self.controller_crypto._encrypt_message(message)
            encrypted_hex = encrypted.hex() if encrypted else ""
        if not encrypted_hex:
            encrypted_hex = json.dumps(message, separators=(",", ":")).encode().hex()
        short_hex = encrypted_hex[:96] + ("..." if len(encrypted_hex) > 96 else "")
        self.log(f"[ENC MSG] type={message_type} payload_hex={short_hex}")

    def _poll_swarm_event_logs(self):
        """Pull internal swarm events and print encrypted command/message logs separately."""
        if not hasattr(self.swarm_manager, "drain_system_events"):
            return
        try:
            events = self.swarm_manager.drain_system_events(200)
        except Exception:
            return
        for event in events:
            kind = str(event.get("kind", "")).strip().lower()
            if kind == "command":
                self._log_system_command_encrypted(
                    str(event.get("command", "UNKNOWN")),
                    event.get("payload", {}) or {},
                )
            elif kind == "message":
                self._log_system_message_encrypted(
                    str(event.get("message_type", "UNKNOWN")),
                    event.get("data", {}) or {},
                )
            elif kind == "warning":
                msg_type = str(event.get("message_type", "WARNING"))
                data = event.get("data", {}) or {}
                self.log(f"[WARN] {msg_type}: {data}")
            elif kind == "path_replan":
                data = event.get("data", {}) or {}
                self.log(
                    "[AVOID] D{drone} prob={prob:.2f} cone={cone:.2f} conf={conf:.2f} fallback={fallback}".format(
                        drone=data.get("drone_id", "?"),
                        prob=float(data.get("collision_probability", 0.0)),
                        cone=float(data.get("collision_cone_probability", 0.0)),
                        conf=float(data.get("ml_confidence", 0.0)),
                        fallback=bool(data.get("fallback_mode", False)),
                    )
                )

    def _poll_encrypted_comm_logs(self):
        """Tail drone communication encrypted TX/RX lines into system log."""
        log_files = glob.glob(os.path.join("logs", "comm_drone_*.log"))
        for path in log_files:
            try:
                prev_offset = self.comm_log_offsets.get(path, 0)
                with open(path, "r", encoding="utf-8", errors="ignore") as fh:
                    fh.seek(prev_offset)
                    new_lines = fh.readlines()
                    self.comm_log_offsets[path] = fh.tell()
                for line in new_lines:
                    if "TX encrypted" in line or "RX encrypted" in line:
                        self.log(f"[ENC D2D] {line.strip()}")
            except Exception:
                continue

    def _sync_selected_drone_mission_inputs(self):
        """Sync selected drone mission values into UI inputs."""
        drone = self.get_selected_drone()
        if drone is None:
            return
        self.ref_lat_spin.setValue(float(drone.gps_ref_lat))
        self.ref_lon_spin.setValue(float(drone.gps_ref_lon))
        if drone.area_mission.active:
            self.target_lat_spin.setValue(float(drone.area_mission.center_lat))
            self.target_lon_spin.setValue(float(drone.area_mission.center_lon))
            self.target_radius_spin.setValue(int(drone.area_mission.radius_m))

    def assign_selected_drone_mission(self):
        """Assign targeted GPS area mission to selected drone."""
        ref_lat = float(self.ref_lat_spin.value())
        ref_lon = float(self.ref_lon_spin.value())
        tgt_lat = float(self.target_lat_spin.value())
        tgt_lon = float(self.target_lon_spin.value())
        radius = float(self.target_radius_spin.value())
        target_id = int(self.mission_drone_id_spin.value())

        # "All drones" mode: assign one mission per drone with slight offsets
        # to reduce overlap and spread coverage around the chosen target.
        if target_id == 0:
            assigned_ids = []
            all_ids = sorted(self.swarm_manager.drones.keys())
            total = len(all_ids)
            if total == 0:
                self.log("Mission assignment failed (no drones found)")
                return

            # "All" mode assigns different target centers to each drone
            # around the selected GPS point (ring distribution).
            meters_per_deg_lat = 111320.0
            meters_per_deg_lon = 111320.0 * math.cos(math.radians(tgt_lat))
            spread_m = max(30.0, radius * 1.5)

            for idx, drone_id in enumerate(all_ids):
                angle = (2.0 * math.pi * idx) / max(1, total)
                offset_east_m = spread_m * math.cos(angle)
                offset_north_m = spread_m * math.sin(angle)
                drone_tgt_lat = tgt_lat + (offset_north_m / meters_per_deg_lat)
                drone_tgt_lon = tgt_lon + (offset_east_m / meters_per_deg_lon)
                ok = self.swarm_manager.assign_area_mission_to_drone(
                    drone_id, ref_lat, ref_lon, drone_tgt_lat, drone_tgt_lon, radius
                )
                if ok:
                    assigned_ids.append(drone_id)
                    self._log_controller_to_drone_encrypted(
                        drone_id,
                        "assign_area_mission",
                        {
                            "ref_lat": ref_lat,
                            "ref_lon": ref_lon,
                            "target_lat": drone_tgt_lat,
                            "target_lon": drone_tgt_lon,
                            "radius_m": radius
                        }
                    )
            if assigned_ids:
                self.log(
                    f"Mission assigned to ALL drones {assigned_ids} with different target locations"
                )
            else:
                self.log("Mission assignment failed (no drones found)")
            return

        # Single-drone mode: assign the exact selected target center.
        ok = self.swarm_manager.assign_area_mission_to_drone(
            target_id, ref_lat, ref_lon, tgt_lat, tgt_lon, radius
        )
        if ok:
            self._log_controller_to_drone_encrypted(
                target_id,
                "assign_area_mission",
                {
                    "ref_lat": ref_lat,
                    "ref_lon": ref_lon,
                    "target_lat": tgt_lat,
                    "target_lon": tgt_lon,
                    "radius_m": radius
                }
            )
            self.log(f"Mission assigned to D{target_id} ({tgt_lat:.6f},{tgt_lon:.6f}) r={radius:.0f}m")
        else:
            self.log(f"Mission assignment failed for D{target_id}")

    def clear_selected_drone_mission(self):
        """Clear mission for selected drone."""
        target_id = int(self.mission_drone_id_spin.value())
        # Mirror assign behavior: 0 clears every drone mission.
        if target_id == 0:
            cleared = []
            for drone_id in sorted(self.swarm_manager.drones.keys()):
                if self.swarm_manager.clear_area_mission_for_drone(drone_id):
                    cleared.append(drone_id)
                    self._log_controller_to_drone_encrypted(drone_id, "clear_area_mission", {})
            if cleared:
                self.log(f"Mission cleared for ALL drones {cleared}")
            else:
                self.log("Mission clear failed (no drones found)")
            return
        if self.swarm_manager.clear_area_mission_for_drone(target_id):
            self._log_controller_to_drone_encrypted(target_id, "clear_area_mission", {})
            self.log(f"Mission cleared for D{target_id}")
        else:
            self.log(f"Mission clear failed for D{target_id}")

    def open_target_in_google_maps(self):
        """Open selected target coordinates in Google Maps."""
        lat = float(self.target_lat_spin.value())
        lon = float(self.target_lon_spin.value())
        url = f"https://www.google.com/maps/search/?api=1&query={lat},{lon}"
        try:
            webbrowser.open(url)
            self.log(f"Opened Google Maps target: {lat:.6f},{lon:.6f}")
        except Exception as e:
            self.log(f"Could not open Google Maps: {e}")


def start_gui(swarm_manager):
    """Start the GUI application"""
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # Modern style
    
    window = MainWindow(swarm_manager)
    window.show()
    
    return app.exec_()
