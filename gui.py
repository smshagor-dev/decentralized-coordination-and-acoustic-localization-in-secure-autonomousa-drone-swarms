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
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QFont, QPalette, QPolygonF
import logging
try:
    from PyQt5.QtSvg import QSvgRenderer
except Exception:
    QSvgRenderer = None

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
        self.animation_phase = 0.0
        self.corners = {}
        self.start_point = None
        self.destination_point = None
        self.destination_slots = {}
        self.start_slots = {}
        self.minimum_gap_m = 40.0
        
        # Colors
        self.colors = {
            'leader': QColor(255, 215, 0),      # Gold
            'follower': QColor(70, 130, 180),   # Steel Blue
            'emergency': QColor(255, 69, 0),    # Red-Orange
            'grounded': QColor(128, 128, 128),  # Gray
            'home': QColor(34, 139, 34),        # Forest Green
            'background': QColor(20, 20, 30),   # Dark Blue
            'grid': QColor(50, 50, 70)          # Grid color
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
        active_ids = set(self.drones.keys())
        for existing_id in list(self.position_history.keys()):
            if existing_id not in active_ids:
                del self.position_history[existing_id]
        for drone_id, drone_data in self.drones.items():
            pos = drone_data.get("position", {})
            point = (pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0))
            trail = self.position_history.setdefault(drone_id, [])
            trail.append(point)
            if len(trail) > 60:
                del trail[0]
        self._update_map_origin()
        self.update()

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
        """Draw static SVG field background with fallback color."""
        if self.fields_renderer:
            self.fields_renderer.render(painter, QRectF(0, 0, self.width(), self.height()))
            return
        painter.fillRect(self.rect(), QColor(37, 76, 39))

    def _draw_trails(self, painter):
        """Draw short movement trails to visualize forward motion."""
        for drone_id, trail in self.position_history.items():
            if len(trail) < 2:
                continue
            for i in range(1, len(trail)):
                x1, y1, z1 = trail[i - 1]
                x2, y2, z2 = trail[i]
                sx1, sy1, _ = self.world_to_screen(x1, y1, z1)
                sx2, sy2, _ = self.world_to_screen(x2, y2, z2)
                alpha = int(180 * (i / len(trail)))
                pen = QPen(QColor(120, 220, 255, alpha))
                pen.setWidth(1)
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
        self._setup_controller_crypto()
        
        self.setWindowTitle("Drone Swarm Management System")
        self.setGeometry(100, 100, 1400, 900)
        self.setFocusPolicy(Qt.StrongFocus)
        
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # Left panel - Visualization + location map tabs
        viz_group = QGroupBox("Swarm Visualization")
        viz_layout = QVBoxLayout()
        self.viz_tabs = QTabWidget()
        self.drone_widget = DroneWidget()
        self.location_map_widget = LocationMapWidget()
        self.viz_tabs.addTab(self.drone_widget, "Drone Visual")
        self.viz_tabs.addTab(self.location_map_widget, "Location Map")
        viz_layout.addWidget(self.viz_tabs)
        viz_group.setLayout(viz_layout)
        main_layout.addWidget(viz_group, 3)
        
        # Right panel - Controls and Info (scrollable controls)
        right_container = QWidget()
        right_panel = QVBoxLayout(right_container)
        right_panel.setContentsMargins(0, 0, 0, 0)
        right_panel.setSpacing(6)

        # Control panel inside scroll area
        control_group = self._create_control_panel()
        control_scroll = QScrollArea()
        control_scroll.setWidgetResizable(True)
        control_scroll.setWidget(control_group)
        control_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        control_scroll.setMinimumHeight(420)
        control_scroll.setMaximumHeight(16777215)
        right_panel.addWidget(control_scroll, 5)

        # Status panel
        status_group = self._create_status_panel()
        right_panel.addWidget(status_group, 2)

        # Logs
        log_group = self._create_log_panel()
        right_panel.addWidget(log_group, 2)

        main_layout.addWidget(right_container, 1)
        
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
    
    def _create_control_panel(self):
        """Create control panel"""
        group = QGroupBox("Controls Panel")
        layout = QVBoxLayout()
        
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
        self.arm_all_btn.clicked.connect(self.arm_all)
        flight_layout.addWidget(self.arm_all_btn, 0, 0)
        
        self.takeoff_all_btn = QPushButton("Takeoff All")
        self.takeoff_all_btn.clicked.connect(self.takeoff_all)
        flight_layout.addWidget(self.takeoff_all_btn, 0, 1)
        
        self.land_all_btn = QPushButton("Land All")
        self.land_all_btn.clicked.connect(self.land_all)
        flight_layout.addWidget(self.land_all_btn, 1, 0)
        
        self.rth_all_btn = QPushButton("RTH All")
        self.rth_all_btn.clicked.connect(self.return_all_home)
        flight_layout.addWidget(self.rth_all_btn, 1, 1)
        
        self.emergency_btn = QPushButton("EMERGENCY LAND")
        self.emergency_btn.setStyleSheet("background-color: red; color: white; font-weight: bold")
        self.emergency_btn.clicked.connect(self.emergency_land_all)
        flight_layout.addWidget(self.emergency_btn, 2, 0, 1, 2)

        self.personal_emergency_btn = QPushButton("EMERGENCY SELECTED")
        self.personal_emergency_btn.setStyleSheet("background-color: #b22222; color: white; font-weight: bold")
        self.personal_emergency_btn.clicked.connect(self.emergency_land_selected)
        flight_layout.addWidget(self.personal_emergency_btn, 3, 0, 1, 2)

        self.command_xy_btn = QPushButton("Leader Command X->Y")
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
        self.auto_fault_btn.setStyleSheet("background-color: #5a5f6b; color: white; font-weight: bold")
        test_layout.addWidget(self.auto_fault_btn)

        self.test_latency_btn = QPushButton("Simulate Latency Spike")
        self.test_latency_btn.clicked.connect(self.simulate_latency_spike)
        test_layout.addWidget(self.test_latency_btn)
        
        layout.addLayout(test_layout)

        obstacle_group = QGroupBox("Dynamic Obstacles")
        obstacle_layout = QGridLayout()

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

        self.use_ml_avoidance_cb = QCheckBox("Use Personal ML Avoidance")
        self.use_ml_avoidance_cb.setChecked(True)
        self.use_ml_avoidance_cb.stateChanged.connect(self._on_use_ml_avoidance_changed)
        obstacle_layout.addWidget(self.use_ml_avoidance_cb, 5, 0, 1, 3)

        self.clear_obstacles_btn = QPushButton("Clear Obstacles")
        self.clear_obstacles_btn.clicked.connect(self.clear_all_obstacles)
        obstacle_layout.addWidget(self.clear_obstacles_btn, 5, 3)

        obstacle_group.setLayout(obstacle_layout)
        layout.addWidget(obstacle_group)

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
            "background-color: #2b2f3a;"
            "color: #f0f3f7;"
            "border: 1px solid #4b5568;"
            "border-radius: 6px;"
            "font-weight: bold;"
            "min-width: 48px;"
            "min-height: 30px;"
            "}"
            "QPushButton:hover { background-color: #3a4151; }"
            "QPushButton:pressed { background-color: #1f2430; }"
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
        gps_group = QGroupBox("Google Map GPS Mission (Selected)")
        gps_layout = QGridLayout()
        gps_layout.setHorizontalSpacing(12)
        gps_layout.setVerticalSpacing(8)
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
        self.ref_lat_spin.setMinimumWidth(130)
        gps_layout.addWidget(self.ref_lat_spin, 0, 1)

        gps_layout.addWidget(QLabel("Ref Lon"), 0, 2)
        self.ref_lon_spin = QDoubleSpinBox()
        self.ref_lon_spin.setRange(-180.0, 180.0)
        self.ref_lon_spin.setDecimals(6)
        self.ref_lon_spin.setValue(39.200269)  # Voronezh
        self.ref_lon_spin.setMinimumWidth(130)
        gps_layout.addWidget(self.ref_lon_spin, 0, 3)

        # Target GPS center for the mission zone.
        gps_layout.addWidget(QLabel("Target Lat"), 1, 0)
        self.target_lat_spin = QDoubleSpinBox()
        self.target_lat_spin.setRange(-90.0, 90.0)
        self.target_lat_spin.setDecimals(6)
        self.target_lat_spin.setValue(51.664500)
        self.target_lat_spin.setMinimumWidth(130)
        # gps_layout.addWidget(self.target_lat_spin, 1, 1)

        gps_layout.addWidget(QLabel("Target Lon"), 1, 2)
        self.target_lon_spin = QDoubleSpinBox()
        self.target_lon_spin.setRange(-180.0, 180.0)
        self.target_lon_spin.setDecimals(6)
        self.target_lon_spin.setValue(39.214300)
        self.target_lon_spin.setMinimumWidth(130)
        gps_layout.addWidget(self.target_lon_spin, 1, 3)

        # Radius of mission search/coverage area in meters.
        gps_layout.addWidget(QLabel("Radius m"), 2, 0)
        self.target_radius_spin = QSpinBox()
        self.target_radius_spin.setRange(10, 5000)
        self.target_radius_spin.setValue(200)
        self.target_radius_spin.setMinimumWidth(100)
        gps_layout.addWidget(self.target_radius_spin, 2, 1)

        # 0 means broadcast to all drones; any other value targets one drone.
        gps_layout.addWidget(QLabel("Drone ID (0=All)"), 2, 2)
        self.mission_drone_id_spin = QSpinBox()
        self.mission_drone_id_spin.setRange(0, 9999)
        self.mission_drone_id_spin.setValue(0)  # default: all drones
        self.mission_drone_id_spin.setMinimumWidth(100)
        gps_layout.addWidget(self.mission_drone_id_spin, 2, 3)

        self.assign_mission_btn = QPushButton("Assign Mission")
        self.assign_mission_btn.clicked.connect(self.assign_selected_drone_mission)
        self.assign_mission_btn.setMinimumHeight(32)
        gps_layout.addWidget(self.assign_mission_btn, 3, 2)

        self.clear_mission_btn = QPushButton("Clear Mission")
        self.clear_mission_btn.clicked.connect(self.clear_selected_drone_mission)
        self.clear_mission_btn.setMinimumHeight(32)
        gps_layout.addWidget(self.clear_mission_btn, 3, 3)

        self.open_map_btn = QPushButton("Open Target in Google Maps")
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
        group.setMinimumHeight(260)
        group.setMaximumHeight(360)
        
        # Status labels
        self.status_labels = {}
        
        labels = [
            ("total_drones", "Total Drones:"),
            ("active_drones", "Active Drones:"),
            ("leader_id", "Leader ID:"),
            ("avg_battery", "Avg Battery:")
        ]
        
        # Compact 2x2 inline metrics:
        # Total Drones: X    Active Drones: Y
        # Leader ID: Z       Avg Battery: N%
        stats_grid = QGridLayout()
        stats_grid.setHorizontalSpacing(10)
        stats_grid.setVerticalSpacing(2)
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
        ]
        for idx, (key, title) in enumerate(latency_keys):
            row = idx // 2
            col = (idx % 2) * 2
            name = QLabel(f"{title}:")
            value = QLabel("0.0 ms")
            value.setStyleSheet("font-size: 11px;")
            latency_grid.addWidget(name, row, col)
            latency_grid.addWidget(value, row, col + 1)
            self.latency_value_labels[key] = value
        layout.addLayout(latency_grid)
        
        # Drone table
        self.drone_table = QTableWidget()
        self.drone_table.setColumnCount(6)
        self.drone_table.setHorizontalHeaderLabels([
            "ID", "Role", "Mode", "Battery", "Altitude", "Status"
        ])
        self.drone_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.drone_table.verticalHeader().setSectionResizeMode(QHeaderView.Fixed)
        self.drone_table.verticalHeader().setDefaultSectionSize(24)
        self.drone_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.drone_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.drone_table.setVerticalScrollMode(QAbstractItemView.ScrollPerItem)
        self.drone_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.drone_table.setMinimumHeight(150)
        self.drone_table.setMaximumHeight(150)
        self.drone_table.itemSelectionChanged.connect(self._on_drone_selection_changed)
        layout.addWidget(self.drone_table)
        
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
        if drones:
            avg_battery = sum(d["battery"] for d in drones.values()) / len(drones)
            self.status_labels["avg_battery"].setText(f"{avg_battery:.1f}%")
        else:
            self.status_labels["avg_battery"].setText("0.0%")

        latency = status.get("latency", {})
        for key, label in self.latency_value_labels.items():
            label.setText(f"{float(latency.get(key, 0.0)):.1f} ms")

        raw_obstacles = status.get("dynamic_obstacles", [])
        filtered_obstacles = []
        for obstacle in raw_obstacles:
            motion_type = obstacle.get("motion_type", "linear")
            dynamic = motion_type != "static" and (
                abs(float(obstacle.get("vx", 0.0))) > 0.001 or abs(float(obstacle.get("vy", 0.0))) > 0.001
                or motion_type in {"circular", "random_walk"}
            )
            if dynamic and not self.show_dynamic_obstacles:
                continue
            if (not dynamic) and not self.show_static_obstacles:
                continue
            view = dict(obstacle)
            view["dynamic"] = dynamic
            filtered_obstacles.append(view)
        # Update drone table
        ordered_rows = sorted(drones.items(), key=lambda kv: int(kv[0]))
        self.drone_table.setRowCount(len(ordered_rows))
        for row, (drone_id, drone_data) in enumerate(ordered_rows):
            self.drone_table.setItem(row, 0, QTableWidgetItem(str(drone_id)))
            self.drone_table.setItem(row, 1, QTableWidgetItem(drone_data["role"]))
            self.drone_table.setItem(row, 2, QTableWidgetItem(drone_data["flight_mode"]))
            self.drone_table.setItem(row, 3, QTableWidgetItem(f"{drone_data['battery']:.1f}%"))
            self.drone_table.setItem(row, 4, QTableWidgetItem(f"{drone_data['position']['z']:.1f}m"))

            swarm_state = drone_data.get("swarm_state", "IDLE")
            status_text = swarm_state if drone_data["is_active"] else "Inactive"
            mission = drone_data.get("mission", {})
            if mission.get("active"):
                status_text = f"{swarm_state} | Mission: {mission.get('status', 'active')}"
            if drone_data.get("motor_failure_warning"):
                status_text = "Warning: Motor fail -> Return X"
            if drone_data.get("emergency_return_active"):
                status_text = "Emergency Return to X"
            self.drone_table.setItem(row, 5, QTableWidgetItem(status_text))

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
            self.auto_fault_btn.setStyleSheet("background-color: #b35b00; color: white; font-weight: bold")
            self.log("Auto fault demo enabled (motor failure + leader crash)")
        else:
            self.auto_fault_btn.setText("Auto Fault Demo: OFF")
            self.auto_fault_btn.setStyleSheet("background-color: #5a5f6b; color: white; font-weight: bold")
            self.log("Auto fault demo disabled")

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
