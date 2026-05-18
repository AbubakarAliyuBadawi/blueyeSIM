#!/usr/bin/env python3
"""
Blueye X3 Operator HUD
======================
Live camera feed + joystick overlay + thruster status + pose display.

Topics consumed:
  /blueye/cam/image_color   sensor_msgs/Image
  /joy                      sensor_msgs/Joy
  /blueye/thrusters         std_msgs/Float64MultiArray
  /blueye/odom              nav_msgs/Odometry
"""

import os
import sys
import math
import threading

# PyQt5 MUST be imported before cv2.
# cv2 ships its own bundled Qt libs; if cv2 loads first it registers conflicting
# xcb platform plugins and PyQt5 crashes.  Loading PyQt5 first anchors the
# system Qt5 in the process, after which cv2 reuses those libraries harmlessly.
_SYSTEM_QT_PLUGIN_PATH = "/usr/lib/x86_64-linux-gnu/qt5/plugins"
if os.path.isdir(_SYSTEM_QT_PLUGIN_PATH):
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = _SYSTEM_QT_PLUGIN_PATH
else:
    os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QFrame, QSizePolicy, QLabel,
)
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QCoreApplication
from PyQt5.QtGui import (
    QImage, QPixmap, QPainter, QColor, QPen, QFont,
)

# cv2 and ROS imports come AFTER PyQt5 is loaded
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, Joy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


# ── Thread-safe Qt signals ────────────────────────────────────────────────────

class _Signals(QObject):
    camera_ready   = pyqtSignal(object)   # numpy BGR array
    joy_ready      = pyqtSignal(object)   # sensor_msgs/Joy
    thrust_ready   = pyqtSignal(object)   # list[float]
    odom_ready     = pyqtSignal(object)   # nav_msgs/Odometry


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
            f"font-family:Monospace; font-size:18px; font-weight:bold;"
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


# ── Joystick panel ────────────────────────────────────────────────────────────

class JoystickPanel(QWidget):
    """
    Two thumbstick visualisations + button row.

    Axis mapping (matches stonefish_blueye_control.launch.py):
      axis 0 → left  X → yaw
      axis 1 → left  Y → heave
      axis 3 → right X → sway
      axis 4 → right Y → surge
    """

    _BG       = QColor("#1e1e2e")
    _RING     = QColor("#585b70")
    _PAD      = QColor("#313244")
    _CROSS    = QColor("#45475a")
    _DOT      = QColor("#89b4fa")
    _LABEL    = QColor("#6c7086")
    _TITLE    = QColor("#cdd6f4")
    _BTN_ON   = QColor("#89b4fa")
    _BTN_OFF  = QColor("#45475a")
    _BTN_TXT  = QColor("#cdd6f4")

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

    # ── painting ──────────────────────────────────────────────────────────────

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

        self._stick(p, lx, cy, pad_r,
                    self._axes[0], self._axes[1],
                    "YAW / HEAVE")
        self._stick(p, rx, cy, pad_r,
                    self._axes[3], self._axes[4],
                    "SWAY / SURGE")
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
    Top-down ROV silhouette with 4 thruster indicators.

    Blueye X3 approximate layout (4 thrusters from Float64MultiArray):
        T0 — front-right  (horizontal, surge/yaw)
        T1 — front-left   (horizontal, surge/yaw)
        T2 — rear         (horizontal, yaw)
        T3 — vertical     (heave)

    Thrust bars show forward (green) / reverse (red) intensity.
    Max setpoint from launch file: 35.0
    """

    _MAX_SETPOINT = 35.0
    _BG   = QColor("#1e1e2e")
    _BODY = QColor("#313244")
    _IDLE = QColor("#45475a")
    _FWD  = QColor("#a6e3a1")
    _REV  = QColor("#f38ba8")

    def __init__(self):
        super().__init__()
        self._thrusters = [0.0, 0.0, 0.0, 0.0]
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def ingest(self, data):
        lst = list(data)
        self._thrusters = lst + [0.0] * max(0, 4 - len(lst))
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        W, H = self.width(), self.height()

        p.fillRect(0, 0, W, H, self._BG)

        p.setPen(QColor("#cdd6f4"))
        p.setFont(QFont("Monospace", 9, QFont.Bold))
        p.drawText(0, 2, W, 18, Qt.AlignCenter, "THRUSTERS")

        # ROV body
        bw = int(W * 0.36)
        bh = int((H - 80) * 0.55)
        bx = (W - bw) // 2
        by = 22 + (H - 22 - bh) // 3

        p.setBrush(self._BODY)
        p.setPen(QPen(QColor("#585b70"), 2))
        p.drawRoundedRect(bx, by, bw, bh, 10, 10)

        # Front marker
        p.setPen(QColor("#6c7086"))
        p.setFont(QFont("Monospace", 7))
        p.drawText(bx, by - 14, bw, 12, Qt.AlignCenter, "▲ FRONT")

        # Thruster positions  [x, y, label, axis_hint]
        cx_body = bx + bw // 2
        positions = [
            (bx + bw + 8,  by + bh // 4,      "T0", "H"),   # front-right
            (bx - 8,       by + bh // 4,      "T1", "H"),   # front-left
            (cx_body,      by - 8,            "T2", "H"),   # front-center
            (cx_body,      by + bh + 8,       "T3", "V"),   # vertical
        ]

        for i, (tx, ty, lbl, axis) in enumerate(positions):
            val  = self._thrusters[i] if i < len(self._thrusters) else 0.0
            norm = max(-1.0, min(1.0, val / self._MAX_SETPOINT))
            dot_color = self._thrust_color(norm)

            tr = 9
            p.setBrush(dot_color)
            p.setPen(Qt.NoPen)
            p.drawEllipse(tx - tr, ty - tr, 2 * tr, 2 * tr)

            p.setPen(QColor("#cdd6f4"))
            p.setFont(QFont("Monospace", 6, QFont.Bold))
            p.drawText(tx - tr, ty - tr, 2 * tr, 2 * tr, Qt.AlignCenter, lbl)

            # Thrust bar
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

            # Percentage label
            p.setPen(QColor("#6c7086"))
            p.setFont(QFont("Monospace", 6))
            pct = f"{abs(norm)*100:.0f}%"
            p.drawText(bbx, bby + bar_h + 1, bar_w, 10, Qt.AlignCenter, pct)

        # Legend
        p.setFont(QFont("Monospace", 7))
        p.setPen(self._FWD)
        p.drawText(4, H - 18, W // 2, 14, Qt.AlignLeft, "■ FWD")
        p.setPen(self._REV)
        p.drawText(W // 2, H - 18, W // 2, 14, Qt.AlignLeft, "■ REV")

    def _thrust_color(self, norm):
        if abs(norm) < 0.03:
            return self._IDLE
        return self._FWD if norm > 0 else self._REV


# ── Pose panel ────────────────────────────────────────────────────────────────

class PosePanel(QWidget):
    """
    Compass (top-down heading arrow) + depth bar + numeric RPY / XYZ readouts.
    """

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

    def ingest(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
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

        # Compass
        comp_r  = max(28, min(W // 3 - 10, (H - 90) // 2))
        comp_cx = W // 2 - 10
        comp_cy = 22 + comp_r + 8

        p.setPen(QPen(self._RING, 2))
        p.setBrush(self._COMPASS)
        p.drawEllipse(comp_cx - comp_r, comp_cy - comp_r, 2 * comp_r, 2 * comp_r)

        # Tick marks
        p.setPen(QPen(QColor("#45475a"), 1))
        for deg in range(0, 360, 30):
            rad = math.radians(deg)
            r0  = comp_r - 5
            r1  = comp_r - 1
            x0  = comp_cx + int(r0 * math.sin(rad))
            y0  = comp_cy - int(r0 * math.cos(rad))
            x1  = comp_cx + int(r1 * math.sin(rad))
            y1  = comp_cy - int(r1 * math.cos(rad))
            p.drawLine(x0, y0, x1, y1)

        # Cardinals
        for lbl, deg in [("N", 0), ("E", 90), ("S", 180), ("W", 270)]:
            rad = math.radians(deg)
            off = comp_r + 11
            ax  = comp_cx + int(off * math.sin(rad))
            ay  = comp_cy - int(off * math.cos(rad))
            p.setPen(self._NORTH if lbl == "N" else self._CARD)
            p.setFont(QFont("Monospace", 7, QFont.Bold))
            p.drawText(ax - 7, ay - 7, 14, 14, Qt.AlignCenter, lbl)

        # Heading arrow (yaw: 0=North, CW positive in NED)
        yaw_deg  = math.degrees(self.yaw)
        alen     = comp_r - 5
        ax2 = comp_cx + int(alen * math.sin(math.radians(yaw_deg)))
        ay2 = comp_cy - int(alen * math.cos(math.radians(yaw_deg)))
        p.setPen(QPen(self._ARROW, 3))
        p.drawLine(comp_cx, comp_cy, ax2, ay2)
        p.setBrush(self._ARROW)
        p.setPen(Qt.NoPen)
        p.drawEllipse(comp_cx - 4, comp_cy - 4, 8, 8)

        # Depth bar (right of compass)
        db_x  = comp_cx + comp_r + 14
        db_y  = 22
        db_h  = comp_r * 2 + 16
        db_w  = 14
        max_d = 30.0
        dnorm = max(0.0, min(1.0, abs(self.z) / max_d))

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

        # Numeric readouts
        ty   = comp_cy + comp_r + 14
        lh   = 15
        rows = [
            ("HDG", f"{math.degrees(self.yaw):+.1f}°"),
            ("RLL", f"{math.degrees(self.roll):+.1f}°"),
            ("PCH", f"{math.degrees(self.pitch):+.1f}°"),
            ("  X", f"{self.x:.2f} m"),
            ("  Y", f"{self.y:.2f} m"),
            ("  Z", f"{abs(self.z):.2f} m"),
        ]
        half = W // 2
        for i, (lbl, val) in enumerate(rows):
            p.setPen(self._LBL)
            p.setFont(QFont("Monospace", 8))
            p.drawText(4, ty + i * lh, half - 4, lh, Qt.AlignLeft, lbl)
            p.setPen(self._VAL)
            p.drawText(half, ty + i * lh, W - half - 4, lh, Qt.AlignRight, val)


# ── Main window ───────────────────────────────────────────────────────────────

class OperatorWindow(QMainWindow):

    def __init__(self, signals: _Signals):
        super().__init__()
        self.setWindowTitle("Blueye X3  —  Operator Interface")
        self.setStyleSheet("background:#181825;")
        self.resize(1280, 820)

        root_w = QWidget()
        self.setCentralWidget(root_w)
        root = QVBoxLayout(root_w)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(4)

        # ── camera (top) ────────────────────────────────────────────────────
        self.cam = CameraPanel()
        root.addWidget(self.cam, stretch=7)

        # ── bottom three panels ─────────────────────────────────────────────
        bottom = QWidget()
        bl = QHBoxLayout(bottom)
        bl.setContentsMargins(0, 0, 0, 0)
        bl.setSpacing(4)

        self.joy_panel      = JoystickPanel()
        self.thruster_panel = ThrusterPanel()
        self.pose_panel     = PosePanel()

        for widget in (self.joy_panel, self.thruster_panel, self.pose_panel):
            frame = QFrame()
            frame.setFrameShape(QFrame.StyledPanel)
            frame.setStyleSheet(
                "QFrame { border:1px solid #313244; border-radius:6px; "
                "background:#1e1e2e; }"
            )
            fl = QVBoxLayout(frame)
            fl.setContentsMargins(2, 2, 2, 2)
            fl.addWidget(widget)
            bl.addWidget(frame)

        root.addWidget(bottom, stretch=3)

        # ── wire signals ────────────────────────────────────────────────────
        signals.camera_ready.connect(self.cam.ingest)
        signals.joy_ready.connect(self.joy_panel.ingest)
        signals.thrust_ready.connect(self.thruster_panel.ingest)
        signals.odom_ready.connect(self.pose_panel.ingest)


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class OperatorHUDNode(Node):

    def __init__(self, signals: _Signals):
        super().__init__("operator_hud")
        self._signals = signals
        self._bridge  = CvBridge()

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            Image, "/blueye/cam/image_color", self._on_image, best_effort)
        self.create_subscription(
            Joy, "/joy", self._on_joy, 10)
        self.create_subscription(
            Float64MultiArray, "/blueye/thrusters", self._on_thrusters, 10)
        self.create_subscription(
            Odometry, "/blueye/odom", self._on_odom, best_effort)

    def _on_image(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            self._signals.camera_ready.emit(frame)
        except Exception as exc:
            self.get_logger().warn(f"Image error: {exc}", throttle_duration_sec=5)

    def _on_joy(self, msg):
        self._signals.joy_ready.emit(msg)

    def _on_thrusters(self, msg):
        self._signals.thrust_ready.emit(msg.data)

    def _on_odom(self, msg):
        self._signals.odom_ready.emit(msg)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)

    if os.path.isdir(_SYSTEM_QT_PLUGIN_PATH):
        os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = _SYSTEM_QT_PLUGIN_PATH
        QCoreApplication.setLibraryPaths([_SYSTEM_QT_PLUGIN_PATH])

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    signals = _Signals()
    node    = OperatorHUDNode(signals)
    window  = OperatorWindow(signals)
    window.show()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    exit_code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
