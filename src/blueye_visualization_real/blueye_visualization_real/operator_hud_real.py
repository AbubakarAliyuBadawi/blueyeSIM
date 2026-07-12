#!/usr/bin/env python3
"""
Blueye Real Drone Operator HUD.

Topics consumed:
  RTSP stream                                         rtsp://<drone_ip>:8554/test
  /blueye/sonar_3d/strength_image   sensor_msgs/Image (mono8)
  /joy                               sensor_msgs/Joy
  /blueye/commands                   geometry_msgs/WrenchStamped
  /blueye/pose                       geometry_msgs/PoseStamped
  /blueye/battery                    geometry_msgs/Pose  (packed)
  /blueye/takeover_request/takeover_requested_prob   std_msgs/Float32
  /blueye/takeover_request/attention_required_prob   std_msgs/Float32
  /blueye/takeover_request/threshold                 std_msgs/Float32

Topics published:
  /blueye/takeover_request/human_decision   std_msgs/Bool
  /blueye/takeover_request/handback         std_msgs/Bool
"""

import math
import os
import sys
import threading

# PyQt5 MUST be imported before cv2.
_SYSTEM_QT_PLUGIN_PATH = "/usr/lib/x86_64-linux-gnu/qt5/plugins"
if os.path.isdir(_SYSTEM_QT_PLUGIN_PATH):
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = _SYSTEM_QT_PLUGIN_PATH
else:
    os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

from PyQt5.QtCore import Qt, QCoreApplication, QObject, QUrl, pyqtSignal
from PyQt5.QtWebEngineWidgets import QWebEnginePage, QWebEngineView
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Pose, PoseStamped, WrenchStamped
from sensor_msgs.msg import Image, Joy
from std_msgs.msg import Bool, Float32, Float64
from std_srvs.srv import SetBool


# ── Thread-safe Qt signals ────────────────────────────────────────────────────

class _Signals(QObject):
    camera_ready     = pyqtSignal(object)        # numpy BGR array  (drone cam)
    rig_camera_ready = pyqtSignal(object)        # numpy BGR array  (rig/docking cam)
    sonar_ready      = pyqtSignal(object)        # numpy BGR array
    joy_ready        = pyqtSignal(object)        # sensor_msgs/Joy
    cmd_ready        = pyqtSignal(list)          # [T0, T1, T2, T3] normalised -1..1
    pose_ready       = pyqtSignal(object)        # geometry_msgs/PoseStamped
    battery_ready    = pyqtSignal(float, float, float, float)  # soc%, runtime_s, current_a, charging_current_a
    takeover_update  = pyqtSignal(float, float)  # urgency, threshold
    show_popup       = pyqtSignal()


# ── Camera panel ──────────────────────────────────────────────────────────────

class CameraPanel(QLabel):
    _NO_SIGNAL_COLOR = "#1a1a2e"
    _NO_SIGNAL_TEXT  = "NO CAMERA SIGNAL"

    def __init__(self):
        super().__init__()
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(640, 360)
        self._apply_no_signal()

    def _apply_no_signal(self):
        self.setStyleSheet(
            f"background:{self._NO_SIGNAL_COLOR}; color:#444466; "
            "font-family:Monospace; font-size:18px; font-weight:bold;"
        )
        self.setText(self._NO_SIGNAL_TEXT)

    def ingest(self, bgr_frame):
        rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg).scaled(
            self.width(), self.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation,
        )
        self.setStyleSheet(f"background:{self._NO_SIGNAL_COLOR};")
        self.setText("")
        self.setPixmap(pix)


# ── Sonar panel ───────────────────────────────────────────────────────────────

class SonarPanel(CameraPanel):
    _NO_SIGNAL_COLOR = "#101018"
    _NO_SIGNAL_TEXT  = "NO SONAR SIGNAL"

    def __init__(self):
        super().__init__()
        self.setMinimumSize(280, 200)
        # Preferred width prevents layout from resizing the panel when sonar
        # data arrives with different dimensions. Expanding height fills strip.
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)


# ── Joystick panel ────────────────────────────────────────────────────────────

class JoystickPanel(QWidget):
    _BG      = QColor("#1e1e2e")
    _RING    = QColor("#585b70")
    _PAD     = QColor("#313244")
    _CROSS   = QColor("#45475a")
    _DOT     = QColor("#89b4fa")
    _LABEL   = QColor("#6c7086")
    _TITLE   = QColor("#cdd6f4")
    _BTN_ON  = QColor("#89b4fa")
    _BTN_OFF = QColor("#45475a")
    _BTN_TXT = QColor("#cdd6f4")

    _BTN_LABELS = ["X", "O", "△", "□", "L1", "R1", "L2", "R2", "SH", "OPT"]

    def __init__(self):
        super().__init__()
        self._axes    = [0.0] * 8
        self._buttons = [0]   * 16
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def ingest(self, msg):
        n_ax  = len(msg.axes)
        n_btn = len(msg.buttons)
        self._axes    = list(msg.axes)    + [0.0] * max(0, 8  - n_ax)
        self._buttons = list(msg.buttons) + [0]   * max(0, 16 - n_btn)
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        W, H = self.width(), self.height()
        p.fillRect(0, 0, W, H, self._BG)

        p.setPen(self._TITLE)
        p.setFont(QFont("Monospace", 9, QFont.Bold))
        p.drawText(0, 2, W, 18, Qt.AlignCenter, "JOYSTICK")

        pad_r = max(30, min(W // 5, (H - 60) // 2))
        cy    = 22 + pad_r + 4
        lx, rx = W // 4, 3 * W // 4

        self._stick(p, lx, cy, pad_r, -self._axes[0], self._axes[1], "YAW / HEAVE")
        self._stick(p, rx, cy, pad_r, -self._axes[3],  self._axes[4], "SWAY / SURGE")
        self._buttons_row(p, W, H)

    def _stick(self, p, cx, cy, r, ax, ay, label):
        p.setPen(QPen(self._RING, 2))
        p.setBrush(self._PAD)
        p.drawEllipse(cx - r, cy - r, 2 * r, 2 * r)
        p.setPen(QPen(self._CROSS, 1))
        p.drawLine(cx - r + 4, cy, cx + r - 4, cy)
        p.drawLine(cx, cy - r + 4, cx, cy + r - 4)
        dot_r = max(6, r // 5)
        dx = int(ax  * (r - dot_r - 2))
        dy = int(-ay * (r - dot_r - 2))
        p.setBrush(self._DOT)
        p.setPen(Qt.NoPen)
        p.drawEllipse(cx + dx - dot_r, cy + dy - dot_r, 2 * dot_r, 2 * dot_r)
        p.setPen(self._LABEL)
        p.setFont(QFont("Monospace", 7))
        p.drawText(cx - r, cy + r + 4, 2 * r, 14, Qt.AlignCenter, label)

    def _buttons_row(self, p, W, H):
        labels  = self._BTN_LABELS
        n       = len(labels)
        btn_r   = max(7, min(10, W // (n * 3)))
        spacing = max(btn_r * 2 + 4, W // (n + 1))
        by      = H - btn_r - 6
        x0      = (W - spacing * (n - 1)) // 2
        for i, lbl in enumerate(labels):
            bx      = x0 + i * spacing
            pressed = bool(self._buttons[i]) if i < len(self._buttons) else False
            color   = self._BTN_ON if pressed else self._BTN_OFF
            p.setBrush(color)
            p.setPen(Qt.NoPen)
            p.drawEllipse(bx - btn_r, by - btn_r, 2 * btn_r, 2 * btn_r)
            p.setPen(self._BTN_TXT if pressed else self._LABEL)
            p.setFont(QFont("Monospace", 6))
            p.drawText(bx - btn_r, by - btn_r, 2 * btn_r, 2 * btn_r, Qt.AlignCenter, lbl)


# ── Thruster panel ────────────────────────────────────────────────────────────

class ThrusterPanel(QWidget):
    """
    Estimated thruster effort from /blueye/commands WrenchStamped.

    Blueye X3 mixing approximation (values normalised -1..1):
        T0  front-right  clamp(surge + yaw)
        T1  front-left   clamp(surge - yaw)
        T2  rear/sway    sway
        T3  vertical     heave
    """

    _BG   = QColor("#1e1e2e")
    _BODY = QColor("#313244")
    _IDLE = QColor("#45475a")
    _FWD  = QColor("#a6e3a1")
    _REV  = QColor("#f38ba8")

    def __init__(self):
        super().__init__()
        self._thrusters = [0.0, 0.0, 0.0, 0.0]
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def ingest(self, thrusters: list):
        self._thrusters = thrusters
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        W, H = self.width(), self.height()
        p.fillRect(0, 0, W, H, self._BG)

        p.setPen(QColor("#cdd6f4"))
        p.setFont(QFont("Monospace", 9, QFont.Bold))
        p.drawText(0, 2, W, 18, Qt.AlignCenter, "THRUSTERS (EST.)")

        bw = int(W * 0.36)
        bh = int((H - 80) * 0.55)
        bx = (W - bw) // 2
        by = 22 + (H - 22 - bh) // 3

        p.setBrush(self._BODY)
        p.setPen(QPen(QColor("#585b70"), 2))
        p.drawRoundedRect(bx, by, bw, bh, 10, 10)

        p.setPen(QColor("#6c7086"))
        p.setFont(QFont("Monospace", 7))
        p.drawText(bx, by - 14, bw, 12, Qt.AlignCenter, "▲ FRONT")

        cx_body = bx + bw // 2
        positions = [
            (bx + bw + 8,  by + bh // 4, "T0"),   # front-right
            (bx - 8,       by + bh // 4, "T1"),   # front-left
            (cx_body,      by + bh + 8,  "T2"),   # rear/sway
            (cx_body + 26, by + bh // 2, "T3"),   # vertical
        ]

        for i, (tx, ty, lbl) in enumerate(positions):
            norm      = max(-1.0, min(1.0, self._thrusters[i] if i < len(self._thrusters) else 0.0))
            dot_color = self._thrust_color(norm)
            tr = 9
            p.setBrush(dot_color)
            p.setPen(Qt.NoPen)
            p.drawEllipse(tx - tr, ty - tr, 2 * tr, 2 * tr)
            p.setPen(QColor("#cdd6f4"))
            p.setFont(QFont("Monospace", 6, QFont.Bold))
            p.drawText(tx - tr, ty - tr, 2 * tr, 2 * tr, Qt.AlignCenter, lbl)

            bar_w, bar_h = 38, 6
            bbx = tx - bar_w // 2
            bby = ty + tr + 3
            p.setBrush(QColor("#1e1e2e"))
            p.setPen(QPen(QColor("#45475a"), 1))
            p.drawRect(bbx, bby, bar_w, bar_h)
            if abs(norm) > 0.02:
                fill = int(abs(norm) * bar_w // 2)
                mid  = bbx + bar_w // 2
                if norm > 0:
                    p.fillRect(mid, bby + 1, fill, bar_h - 2, dot_color)
                else:
                    p.fillRect(mid - fill, bby + 1, fill, bar_h - 2, dot_color)
            p.setPen(QColor("#6c7086"))
            p.setFont(QFont("Monospace", 6))
            p.drawText(bbx, bby + bar_h + 1, bar_w, 10, Qt.AlignCenter,
                       f"{abs(norm) * 100:.0f}%")

        p.setFont(QFont("Monospace", 7))
        p.setPen(self._FWD)
        p.drawText(4, H - 18, W // 2, 14, Qt.AlignLeft, "■ FWD")
        p.setPen(self._REV)
        p.drawText(W // 2, H - 18, W // 2, 14, Qt.AlignLeft, "■ REV")

    def _thrust_color(self, norm: float) -> QColor:
        if abs(norm) < 0.03:
            return self._IDLE
        return self._FWD if norm > 0 else self._REV


# ── Pose panel ────────────────────────────────────────────────────────────────

class PosePanel(QWidget):
    _BG      = QColor("#1e1e2e")
    _RING    = QColor("#585b70")
    _COMPASS = QColor("#313244")
    _ARROW   = QColor("#89b4fa")
    _NORTH   = QColor("#f5c2e7")
    _CARD    = QColor("#6c7086")
    _DEPTH   = QColor("#89dceb")
    _LBL     = QColor("#6c7086")
    _VAL     = QColor("#cdd6f4")

    def __init__(self):
        super().__init__()
        self.x = self.y = self.z = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def ingest(self, msg: PoseStamped):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        q = msg.pose.orientation
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.roll  = math.atan2(sinr, cosr)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        self.pitch = math.asin(max(-1.0, min(1.0, sinp)))
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw   = math.atan2(siny, cosy)
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        W, H = self.width(), self.height()
        p.fillRect(0, 0, W, H, self._BG)

        p.setPen(QColor("#cdd6f4"))
        p.setFont(QFont("Monospace", 9, QFont.Bold))
        p.drawText(0, 2, W, 18, Qt.AlignCenter, "POSE")

        comp_r  = max(28, min(W // 3 - 10, (H - 90) // 2))
        comp_cx = W // 2 - 10
        comp_cy = 22 + comp_r + 8

        p.setPen(QPen(self._RING, 2))
        p.setBrush(self._COMPASS)
        p.drawEllipse(comp_cx - comp_r, comp_cy - comp_r, 2 * comp_r, 2 * comp_r)

        p.setPen(QPen(QColor("#45475a"), 1))
        for deg in range(0, 360, 30):
            rad = math.radians(deg)
            r0  = comp_r - 5
            r1  = comp_r - 1
            p.drawLine(
                comp_cx + int(r0 * math.sin(rad)), comp_cy - int(r0 * math.cos(rad)),
                comp_cx + int(r1 * math.sin(rad)), comp_cy - int(r1 * math.cos(rad)),
            )

        for lbl, deg in [("N", 0), ("E", 90), ("S", 180), ("W", 270)]:
            rad = math.radians(deg)
            off = comp_r + 11
            ax  = comp_cx + int(off * math.sin(rad))
            ay  = comp_cy - int(off * math.cos(rad))
            p.setPen(self._NORTH if lbl == "N" else self._CARD)
            p.setFont(QFont("Monospace", 7, QFont.Bold))
            p.drawText(ax - 7, ay - 7, 14, 14, Qt.AlignCenter, lbl)

        yaw_deg = math.degrees(self.yaw)
        alen    = comp_r - 5
        ax2 = comp_cx + int(alen * math.sin(math.radians(yaw_deg)))
        ay2 = comp_cy - int(alen * math.cos(math.radians(yaw_deg)))
        p.setPen(QPen(self._ARROW, 3))
        p.drawLine(comp_cx, comp_cy, ax2, ay2)
        p.setBrush(self._ARROW)
        p.setPen(Qt.NoPen)
        p.drawEllipse(comp_cx - 4, comp_cy - 4, 8, 8)

        db_x  = comp_cx + comp_r + 14
        db_y  = 22
        db_h  = comp_r * 2 + 16
        db_w  = 14
        dnorm = max(0.0, min(1.0, abs(self.z) / 30.0))
        p.setPen(QPen(QColor("#45475a"), 1))
        p.setBrush(QColor("#313244"))
        p.drawRect(db_x, db_y, db_w, db_h)
        fill_h = int(dnorm * db_h)
        if fill_h:
            p.fillRect(db_x + 1, db_y + db_h - fill_h, db_w - 2, fill_h, self._DEPTH)
        p.setPen(self._LBL)
        p.setFont(QFont("Monospace", 7))
        p.drawText(db_x - 4, db_y - 12, db_w + 8, 11, Qt.AlignCenter, "DEP")
        p.drawText(db_x - 8, db_y + db_h + 2, db_w + 16, 11,
                   Qt.AlignCenter, f"{abs(self.z):.1f}m")

        ty   = comp_cy + comp_r + 14
        lh   = 15
        half = W // 2
        rows = [
            ("HDG", f"{math.degrees(self.yaw):+.1f}°"),
            ("RLL", f"{math.degrees(self.roll):+.1f}°"),
            ("PCH", f"{math.degrees(self.pitch):+.1f}°"),
            ("  X", f"{self.x:.2f} m"),
            ("  Y", f"{self.y:.2f} m"),
            ("  Z", f"{abs(self.z):.2f} m"),
        ]
        for i, (lbl, val) in enumerate(rows):
            p.setPen(self._LBL)
            p.setFont(QFont("Monospace", 8))
            p.drawText(4, ty + i * lh, half - 4, lh, Qt.AlignLeft, lbl)
            p.setPen(self._VAL)
            p.drawText(half, ty + i * lh, W - half - 4, lh, Qt.AlignRight, val)


# ── Battery panel ─────────────────────────────────────────────────────────────

class BatteryPanel(QWidget):
    _BG   = QColor("#1e1e2e")
    _FULL = QColor("#a6e3a1")
    _MID  = QColor("#f9e2af")
    _LOW  = QColor("#f38ba8")
    _LBL  = QColor("#6c7086")
    _VAL  = QColor("#cdd6f4")

    def __init__(self):
        super().__init__()
        self._soc      = 0.0
        self._runtime  = 0.0
        self._current  = 0.0
        self._charging = False
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def ingest(self, soc_pct: float, runtime_s: float, current_a: float,
               charging_current_a: float):
        self._soc      = max(0.0, min(100.0, soc_pct))
        self._runtime  = runtime_s
        self._current  = current_a
        self._charging = current_a > 0.1
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        W, H = self.width(), self.height()
        p.fillRect(0, 0, W, H, self._BG)

        p.setPen(QColor("#cdd6f4"))
        p.setFont(QFont("Monospace", 9, QFont.Bold))
        p.drawText(0, 2, W, 18, Qt.AlignCenter, "BATTERY")

        margin  = 12
        bar_x   = margin
        bar_y   = 24
        bar_w   = W - 2 * margin
        bar_h   = 22
        fill_w  = int(self._soc / 100.0 * bar_w)
        bar_color = (
            self._FULL if self._soc > 50.0 else
            self._MID  if self._soc > 25.0 else
            self._LOW
        )

        p.setBrush(QColor("#313244"))
        p.setPen(QPen(QColor("#45475a"), 1))
        p.drawRoundedRect(bar_x, bar_y, bar_w, bar_h, 4, 4)
        if fill_w > 2:
            p.setBrush(bar_color)
            p.setPen(Qt.NoPen)
            p.drawRoundedRect(bar_x + 1, bar_y + 1, fill_w - 2, bar_h - 2, 3, 3)

        p.setPen(QColor("#cdd6f4"))
        p.setFont(QFont("Monospace", 10, QFont.Bold))
        p.drawText(bar_x, bar_y, bar_w, bar_h, Qt.AlignCenter,
                   f"{self._soc:.1f}%")

        state_color = self._FULL if self._charging else self._MID
        state_text  = "⚡ CHARGING" if self._charging else "● DISCHARGING"
        p.setPen(state_color)
        p.setFont(QFont("Monospace", 8, QFont.Bold))
        p.drawText(0, bar_y + bar_h + 4, W, 16, Qt.AlignCenter, state_text)

        if self._runtime > 0:
            hours   = int(self._runtime // 3600)
            minutes = int((self._runtime % 3600) // 60)
            runtime_str = f"{hours}h {minutes:02d}m"
        else:
            runtime_str = "--h --m"

        ty   = bar_y + bar_h + 24
        lh   = 17
        half = W // 2
        rows = [
            ("RUNTIME", runtime_str),
            ("CURRENT", f"{abs(self._current):.2f} A"),
        ]
        for i, (lbl, val) in enumerate(rows):
            p.setPen(self._LBL)
            p.setFont(QFont("Monospace", 8))
            p.drawText(margin, ty + i * lh, half - margin, lh, Qt.AlignLeft, lbl)
            p.setPen(self._VAL)
            p.drawText(half, ty + i * lh, half - margin, lh, Qt.AlignRight, val)


# ── Auto-modes panel ─────────────────────────────────────────────────────────

class AutoModesPanel(QWidget):
    """Toggle buttons for depth hold and heading hold."""

    _BG      = QColor("#1e1e2e")
    _ON      = QColor("#a6e3a1")   # green  — active
    _OFF     = QColor("#313244")   # dark   — inactive
    _ON_TXT  = QColor("#1e1e2e")
    _OFF_TXT = QColor("#cdd6f4")

    depth_toggled   = None   # set after construction to avoid signal on class body
    heading_toggled = None

    def __init__(self):
        super().__init__()
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setFixedHeight(58)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        outer.setSpacing(3)

        lbl = QLabel("AUTO MODES")
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet(
            "color:#6c7086; font-family:Monospace; font-size:8px; font-weight:bold; "
            "border:none; background:transparent;"
        )
        outer.addWidget(lbl)

        row = QHBoxLayout()
        row.setSpacing(4)

        self.depth_btn   = self._make_btn("DEPTH HOLD")
        self.heading_btn = self._make_btn("HEADING HOLD")
        row.addWidget(self.depth_btn)
        row.addWidget(self.heading_btn)
        outer.addLayout(row)

    def _make_btn(self, text: str) -> QPushButton:
        btn = QPushButton(text)
        btn.setCheckable(True)
        btn.setFixedHeight(24)
        btn.setFont(QFont("Monospace", 8, QFont.Bold))
        self._apply_style(btn)
        btn.toggled.connect(lambda _: self._apply_style(btn))
        return btn

    def _apply_style(self, btn: QPushButton):
        on = btn.isChecked()
        bg  = self._ON.name()  if on else self._OFF.name()
        txt = self._ON_TXT.name() if on else self._OFF_TXT.name()
        btn.setStyleSheet(
            f"QPushButton {{ background:{bg}; color:{txt}; "
            f"font-family:Monospace; font-size:8px; font-weight:bold; "
            f"border:1px solid #45475a; border-radius:3px; padding:2px 4px; }}"
        )


# ── Takeover banner ───────────────────────────────────────────────────────────

class TakeoverBanner(QWidget):
    _BG_SAFE  = QColor("#1e1e2e")
    _BG_ALERT = QColor("#2a1a1a")
    _SAFE     = QColor("#a6e3a1")
    _WARN     = QColor("#f9e2af")
    _DANGER   = QColor("#f38ba8")
    _TEXT     = QColor("#cdd6f4")
    _MUTED    = QColor("#6c7086")

    def __init__(self):
        super().__init__()
        self._prob      = 0.0
        self._threshold = 0.65
        self.setFixedHeight(32)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def update_values(self, prob: float, threshold: float):
        self._prob      = max(0.0, min(1.0, prob))
        self._threshold = max(0.01, threshold)
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        W, H = self.width(), self.height()

        triggered = self._prob >= self._threshold
        p.fillRect(0, 0, W, H, self._BG_ALERT if triggered else self._BG_SAFE)

        bar_w = int(self._prob * (W - 140))
        color = (
            self._DANGER if triggered else
            self._WARN   if self._prob > self._threshold * 0.6 else
            self._SAFE
        )
        p.fillRect(100, 6, bar_w, H - 12, color)
        p.setPen(QPen(QColor("#45475a"), 1))
        p.drawRect(100, 6, W - 140, H - 12)

        thr_x = 100 + int(self._threshold * (W - 140))
        p.setPen(QPen(QColor("#f38ba8"), 2))
        p.drawLine(thr_x, 4, thr_x, H - 4)

        p.setPen(self._TEXT if triggered else self._MUTED)
        p.setFont(QFont("Monospace", 8, QFont.Bold))
        label = "⚠ TAKEOVER REQUESTED" if triggered else "TAKEOVER RISK"
        p.drawText(4, 0, 96, H, Qt.AlignVCenter | Qt.AlignLeft, label)

        p.setPen(self._TEXT)
        p.setFont(QFont("Monospace", 8))
        p.drawText(W - 36, 0, 36, H, Qt.AlignVCenter | Qt.AlignRight,
                   f"{self._prob:.2f}")


# ── Takeover dialog ───────────────────────────────────────────────────────────

class TakeoverDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Takeover Request")
        self.setModal(True)
        self.setFixedSize(420, 200)
        self.setStyleSheet(
            "background:#1e1e2e; color:#cdd6f4; font-family:Monospace;"
        )

        layout = QVBoxLayout(self)
        layout.setSpacing(16)
        layout.setContentsMargins(20, 20, 20, 20)

        title = QLabel("⚠  TAKEOVER REQUESTED")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size:16px; font-weight:bold; color:#f38ba8;")
        layout.addWidget(title)

        msg = QLabel(
            "The Bayesian Network suggests human intervention is needed.\n"
            "Accept to take manual joystick control.\n"
            "Reject to trigger the emergency return-to-dock sequence."
        )
        msg.setAlignment(Qt.AlignCenter)
        msg.setWordWrap(True)
        msg.setStyleSheet("font-size:11px; color:#cdd6f4;")
        layout.addWidget(msg)

        btn_row = QHBoxLayout()
        self.accept_btn = QPushButton("ACCEPT TAKEOVER")
        self.accept_btn.setStyleSheet(
            "background:#a6e3a1; color:#1e1e2e; font-weight:bold; "
            "padding:8px; border-radius:4px;"
        )
        self.reject_btn = QPushButton("REJECT  (Emergency Dock)")
        self.reject_btn.setStyleSheet(
            "background:#f38ba8; color:#1e1e2e; font-weight:bold; "
            "padding:8px; border-radius:4px;"
        )
        btn_row.addWidget(self.accept_btn)
        btn_row.addWidget(self.reject_btn)
        layout.addLayout(btn_row)

        self.accept_btn.clicked.connect(self.accept)
        self.reject_btn.clicked.connect(self.reject)

    def result_accepted(self) -> bool:
        return self.result() == QDialog.Accepted


# ── Silent WebEngine page (suppresses JS console spam) ────────────────────────

class _SilentPage(QWebEnginePage):
    def javaScriptConsoleMessage(self, level, message, line, source):
        pass  # discard all JS console output from embedded web views


# ── Main window ───────────────────────────────────────────────────────────────

class OperatorWindow(QMainWindow):

    def __init__(
        self,
        signals: _Signals,
        ros_node,
        sonar_url: str = "http://192.168.1.96",
        usbl_url: str = "http://localhost:10000",
        enable_sonar_3d: bool = True,
        enable_usbl: bool = True,
    ):
        super().__init__()
        self._ros_node    = ros_node
        self._in_takeover = False
        self.setWindowTitle("Blueye — Real Drone Operator Interface")
        self.setStyleSheet("background:#181825;")
        self.resize(1280, 820)

        root_w = QWidget()
        self.setCentralWidget(root_w)
        root = QVBoxLayout(root_w)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(4)

        self.takeover_banner = TakeoverBanner()
        root.addWidget(self.takeover_banner)

        # ── tab widget ────────────────────────────────────────────────────────
        self._tabs = QTabWidget()
        self._tabs.setStyleSheet(
            "QTabWidget::pane { border:1px solid #313244; background:#181825; }"
            "QTabBar::tab { background:#1e1e2e; color:#6c7086; font-family:Monospace; "
            "  font-size:11px; padding:6px 18px; border:1px solid #313244; "
            "  border-bottom:none; border-top-left-radius:4px; border-top-right-radius:4px; }"
            "QTabBar::tab:selected { background:#313244; color:#cdd6f4; }"
            "QTabBar::tab:hover    { background:#2a2a3e; color:#cdd6f4; }"
        )
        root.addWidget(self._tabs)

        # ── tab 1: Drone View ─────────────────────────────────────────────────
        overview = QWidget()
        ov = QVBoxLayout(overview)
        ov.setContentsMargins(4, 4, 4, 4)
        ov.setSpacing(4)

        top = QWidget()
        tl  = QHBoxLayout(top)
        tl.setContentsMargins(0, 0, 0, 0)
        tl.setSpacing(4)

        self.cam = CameraPanel()
        tl.addWidget(self.cam, stretch=3)

        right_col  = QWidget()
        rc         = QVBoxLayout(right_col)
        rc.setContentsMargins(0, 0, 0, 0)
        rc.setSpacing(4)
        self.battery_panel    = BatteryPanel()
        self.auto_modes_panel = AutoModesPanel()
        self.pose_panel       = PosePanel()
        rc.addWidget(self._frame(self.battery_panel),    stretch=2)
        rc.addWidget(self._frame(self.auto_modes_panel), stretch=0)
        rc.addWidget(self._frame(self.pose_panel),       stretch=3)

        self.auto_modes_panel.depth_btn.toggled.connect(
            lambda on: self._ros_node.call_depth_hold(on))
        self.auto_modes_panel.heading_btn.toggled.connect(
            lambda on: self._ros_node.call_heading_hold(on))
        tl.addWidget(right_col, stretch=2)

        ov.addWidget(top, stretch=7)

        bottom = QWidget()
        bl     = QHBoxLayout(bottom)
        bl.setContentsMargins(0, 0, 0, 0)
        bl.setSpacing(4)
        self.sonar_panel    = SonarPanel()
        self.joy_panel      = JoystickPanel()
        self.thruster_panel = ThrusterPanel()
        bl.addWidget(self._titled_frame(self.sonar_panel, "SONAR STRENGTH IMAGE"))
        for w in (self.joy_panel, self.thruster_panel):
            bl.addWidget(self._frame(w))
        ov.addWidget(bottom, stretch=3)

        self.handback_btn = QPushButton("HAND BACK CONTROL TO MISSION")
        self.handback_btn.setFixedHeight(40)
        self.handback_btn.setStyleSheet(
            "background:#89b4fa; color:#1e1e2e; font-family:Monospace; "
            "font-size:13px; font-weight:bold; border-radius:4px;"
        )
        self.handback_btn.hide()
        self.handback_btn.clicked.connect(self._on_handback)
        ov.addWidget(self.handback_btn)

        self._tabs.addTab(overview, "Drone View")

        # ── tab 2: Docking Camera ─────────────────────────────────────────────
        rig_tab = QWidget()
        rig_layout = QVBoxLayout(rig_tab)
        rig_layout.setContentsMargins(4, 4, 4, 4)
        self.rig_cam = CameraPanel()
        self.rig_cam._NO_SIGNAL_TEXT = "NO DOCKING CAMERA SIGNAL"
        rig_layout.addWidget(self.rig_cam)
        self._tabs.addTab(rig_tab, "Docking Camera")

        # ── tab 3: Sonar 3D (optional — manufacturer web UI) ─────────────────
        if enable_sonar_3d:
            sonar_tab = QWidget()
            sonar_tab_layout = QVBoxLayout(sonar_tab)
            sonar_tab_layout.setContentsMargins(0, 0, 0, 0)
            self.sonar_web = QWebEngineView()
            self.sonar_web.setPage(_SilentPage(self.sonar_web))
            self.sonar_web.load(QUrl(sonar_url))
            sonar_tab_layout.addWidget(self.sonar_web)
            self._tabs.addTab(sonar_tab, "Sonar 3D")

        # ── tab 4: USBL — Sinaps web UI (optional, full-screen) ──────────────
        if enable_usbl:
            usbl_tab = QWidget()
            usbl_layout = QVBoxLayout(usbl_tab)
            usbl_layout.setContentsMargins(0, 0, 0, 0)
            self.usbl_web = QWebEngineView()
            self.usbl_web.setPage(_SilentPage(self.usbl_web))
            self.usbl_web.load(QUrl(usbl_url))
            usbl_layout.addWidget(self.usbl_web)
            self._tabs.addTab(usbl_tab, "USBL")

        # ── wire signals ─────────────────────────────────────────────────────
        signals.camera_ready.connect(self.cam.ingest)
        signals.rig_camera_ready.connect(self.rig_cam.ingest)
        signals.sonar_ready.connect(self.sonar_panel.ingest)
        signals.joy_ready.connect(self.joy_panel.ingest)
        signals.cmd_ready.connect(self.thruster_panel.ingest)
        signals.pose_ready.connect(self.pose_panel.ingest)
        signals.battery_ready.connect(self.battery_panel.ingest)
        signals.takeover_update.connect(self._on_takeover_update)
        signals.show_popup.connect(self._on_show_popup)

    @staticmethod
    def _frame(widget: QWidget) -> QFrame:
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel)
        frame.setStyleSheet(
            "QFrame { border:1px solid #313244; border-radius:6px; background:#1e1e2e; }"
        )
        fl = QVBoxLayout(frame)
        fl.setContentsMargins(2, 2, 2, 2)
        fl.addWidget(widget)
        return frame

    @staticmethod
    def _titled_frame(widget: QWidget, title: str) -> QFrame:
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel)
        frame.setStyleSheet(
            "QFrame { border:1px solid #313244; border-radius:6px; background:#1e1e2e; }"
        )
        fl = QVBoxLayout(frame)
        fl.setContentsMargins(2, 2, 2, 2)
        fl.setSpacing(2)
        lbl = QLabel(title)
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet(
            "border:none; color:#6c7086; font-family:Monospace; "
            "font-size:9px; font-weight:bold; background:transparent;"
        )
        lbl.setFixedHeight(14)
        fl.addWidget(lbl)
        fl.addWidget(widget)
        return frame

    def _on_takeover_update(self, prob: float, threshold: float):
        self.takeover_banner.update_values(prob, threshold)

    def _on_show_popup(self):
        if self._in_takeover:
            return
        dlg = TakeoverDialog(self)
        dlg.exec_()
        decision = dlg.result_accepted()
        self._ros_node.publish_human_decision(decision)
        if decision:
            self._in_takeover = True
            self.handback_btn.show()

    def _on_handback(self):
        self._ros_node.publish_handback()
        self._in_takeover = False
        self.handback_btn.hide()


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class OperatorHUDNode(Node):

    def __init__(self, signals: _Signals):
        super().__init__("blueye_real_operator_hud")
        self._signals    = signals
        self._bridge     = CvBridge()
        self._urgency    = 0.0
        self._threshold  = 0.65
        self._popup_sent = False
        self._shutdown   = False

        self.declare_parameter("drone_ip", "192.168.1.101")
        self.declare_parameter("rig_camera_url", "rtsp://192.168.1.10:554/stream1")
        self.declare_parameter("sonar_url", "http://192.168.1.96")
        self.declare_parameter("usbl_url", "http://localhost:10000")
        self.declare_parameter("enable_sonar_3d", True)
        self.declare_parameter("enable_usbl", True)
        drone_ip = str(self.get_parameter("drone_ip").value)
        self._rtsp_url      = f"rtsp://{drone_ip}:8554/test"
        self._rig_url       = str(self.get_parameter("rig_camera_url").value)
        self.sonar_url      = str(self.get_parameter("sonar_url").value)
        self.usbl_url       = str(self.get_parameter("usbl_url").value)
        self.enable_sonar_3d = bool(self.get_parameter("enable_sonar_3d").value)
        self.enable_usbl    = bool(self.get_parameter("enable_usbl").value)

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            Image, "/blueye/sonar_3d/strength_image", self._on_sonar, best_effort)
        self.create_subscription(Joy, "/joy", self._on_joy, 10)
        self.create_subscription(
            WrenchStamped, "/blueye/commands", self._on_commands, 10)
        self.create_subscription(
            PoseStamped, "/blueye/pose", self._on_pose, best_effort)
        self.create_subscription(Pose, "/blueye/battery", self._on_battery, 10)
        self.create_subscription(
            Float64, "/blueye/takeover_request/urgency", self._on_urgency, 10)
        self.create_subscription(
            Float32, "/blueye/takeover_request/threshold", self._on_threshold, 10)

        self._depth_client   = self.create_client(SetBool, "/blueye/depth_hold")
        self._heading_client = self.create_client(SetBool, "/blueye/heading_hold")

        self._decision_pub = self.create_publisher(
            Bool, "/blueye/takeover_request/human_decision", 10)
        self._handback_pub = self.create_publisher(
            Bool, "/blueye/takeover_request/handback", 10)

        self._cam_thread = threading.Thread(
            target=self._stream_loop,
            args=(self._rtsp_url, self._signals.camera_ready, "drone cam"),
            daemon=True,
        )
        self._rig_thread = threading.Thread(
            target=self._stream_loop,
            args=(self._rig_url, self._signals.rig_camera_ready, "rig cam"),
            daemon=True,
        )
        self._cam_thread.start()
        self._rig_thread.start()
        self.get_logger().info(f"Operator HUD started — drone: {self._rtsp_url}")
        self.get_logger().info(f"Rig/docking camera: {self._rig_url}")

    def _stream_loop(self, url: str, signal, label: str):
        gst_pipeline = (
            f"rtspsrc location={url} latency=50 drop-on-latency=true ! "
            "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
            "video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
        )
        cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            self.get_logger().warning(f"GStreamer failed for {label}; trying FFMPEG")
            cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            self.get_logger().error(f"Cannot open {label}: {url}")
            return
        try:
            while rclpy.ok() and not self._shutdown:
                ret, frame = cap.read()
                if ret:
                    signal.emit(frame)
        finally:
            cap.release()

    def _on_sonar(self, msg: Image):
        try:
            gray  = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            frame = cv2.applyColorMap(gray, cv2.COLORMAP_OCEAN)
            self._signals.sonar_ready.emit(frame)
        except Exception as exc:
            self.get_logger().warn(f"Sonar image error: {exc}", throttle_duration_sec=5)

    def _on_joy(self, msg: Joy):
        self._signals.joy_ready.emit(msg)

    def _on_commands(self, msg: WrenchStamped):
        surge = max(-1.0, min(1.0,  msg.wrench.force.x))
        sway  = max(-1.0, min(1.0,  msg.wrench.force.y))
        heave = max(-1.0, min(1.0,  msg.wrench.force.z))
        yaw   = max(-1.0, min(1.0,  msg.wrench.torque.z))
        thrusters = [
            max(-1.0, min(1.0, surge + yaw)),   # T0 front-right
            max(-1.0, min(1.0, surge - yaw)),   # T1 front-left
            sway,                                # T2 rear/sway
            heave,                               # T3 vertical
        ]
        self._signals.cmd_ready.emit(thrusters)

    def _on_pose(self, msg: PoseStamped):
        self._signals.pose_ready.emit(msg)

    def _on_battery(self, msg: Pose):
        soc = float(msg.position.y)
        if 0.0 < soc <= 1.5:
            soc *= 100.0
        soc = max(0.0, min(100.0, soc))
        self._signals.battery_ready.emit(
            soc,
            float(msg.orientation.x),   # runtime_to_empty
            float(msg.position.z),       # current
            float(msg.position.x),       # charging_current
        )

    def _on_urgency(self, msg: Float64):
        self._urgency = float(msg.data)
        self._signals.takeover_update.emit(self._urgency, self._threshold)
        if self._urgency >= self._threshold and not self._popup_sent:
            self._popup_sent = True
            self._signals.show_popup.emit()
        elif self._urgency < self._threshold * 0.8:
            self._popup_sent = False

    def _on_threshold(self, msg: Float32):
        self._threshold = float(msg.data)

    def publish_human_decision(self, accepted: bool):
        out = Bool()
        out.data = accepted
        self._decision_pub.publish(out)

    def publish_handback(self):
        out = Bool()
        out.data = True
        self._handback_pub.publish(out)

    def call_depth_hold(self, enable: bool):
        if not self._depth_client.wait_for_service(timeout_sec=0.3):
            self.get_logger().warning("depth_hold service not available")
            return
        req = SetBool.Request()
        req.data = enable
        self._depth_client.call_async(req)

    def call_heading_hold(self, enable: bool):
        if not self._heading_client.wait_for_service(timeout_sec=0.3):
            self.get_logger().warning("heading_hold service not available")
            return
        req = SetBool.Request()
        req.data = enable
        self._heading_client.call_async(req)

    def shutdown(self):
        self._shutdown = True


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)

    if os.path.isdir(_SYSTEM_QT_PLUGIN_PATH):
        os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = _SYSTEM_QT_PLUGIN_PATH
        QCoreApplication.setLibraryPaths([_SYSTEM_QT_PLUGIN_PATH])

    app     = QApplication(sys.argv)
    app.setStyle("Fusion")

    signals = _Signals()
    node    = OperatorHUDNode(signals)
    window  = OperatorWindow(
        signals, node,
        sonar_url=node.sonar_url,
        usbl_url=node.usbl_url,
        enable_sonar_3d=node.enable_sonar_3d,
        enable_usbl=node.enable_usbl,
    )
    window.show()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    exit_code = app.exec_()

    node.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
