#!/usr/bin/env python3
import sys
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Twist

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QSpacerItem, QSizePolicy
)
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QTimer, Qt

# Try to resolve share directory when installed
try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None


def _find_logo_path() -> str:
    """Find VerdantEye logo both in dev tree and in installed share dir."""
    here = Path(__file__).resolve().parent
    # dev-time candidates (run from source)
    candidates = [
        here / "VerdantEye.png",
        here.parent / "assets" / "VerdantEye.png",
        here.parent / "ve_behaviours" / "assets" / "VerdantEye.png",
    ]

    # installed location via ament index
    if get_package_share_directory:
        try:
            share = Path(get_package_share_directory("ve_behaviours"))
            candidates += [
                share / "assets" / "VerdantEye.png",
                share / "ve_behaviours" / "assets" / "VerdantEye.png",
            ]
        except Exception:
            pass

    for p in candidates:
        if p.exists():
            return str(p)
    return ""

class RobotUI(Node):
    def __init__(self):
        super().__init__('ui')

        # ROS pubs / subs
        self.run_pub = self.create_publisher(Bool, '/ui/run_enabled', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.restart_cli = self.create_client(Trigger, '/ui/restart')
        self.pose_sub = self.create_subscription(PoseStamped, '/robot_pose', self.pose_callback, 10)

        self.zero_twist = Twist()
        self.x = self.y = self.theta = 0.0

        # Qt app / window
        self.app = QApplication(sys.argv)
        self.win = QWidget()
        self.win.setWindowTitle("VerdantEye Robotics Control")
        self.win.setStyleSheet("background-color: #000000; color: white;")
        # wider stays similar, much taller for “length”
        self.win.setFixedSize(900, 1000)

        root = QVBoxLayout(self.win)
        root.setContentsMargins(28, 20, 28, 20)
        root.setSpacing(14)

        # ---- Title ----
        title = QLabel("VerdantEye Robotics Control")
        title.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        title.setStyleSheet("font-size: 34px; font-weight: 800; color: #00e0c6; letter-spacing: 0.5px;")
        root.addWidget(title)

        # ---- Logo ----
        logo = QLabel()
        logo.setAlignment(Qt.AlignHCenter)
        logo_path = _find_logo_path()
        if logo_path:
            pm = QPixmap(logo_path)
            if not pm.isNull():
                # bigger image than before
                pm = pm.scaled(520, 400, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                logo.setPixmap(pm)
                self.get_logger().info(f"[UI] Loaded logo: {logo_path}")
            else:
                logo.setText("VerdantEye Logo (unsupported format)")
                logo.setStyleSheet("font-size: 16px; color: #AAAAAA;")
        else:
            logo.setText("VerdantEye Logo Missing")
            logo.setStyleSheet("font-size: 16px; color: #AAAAAA;")
        root.addWidget(logo)

        # ---- Status ----
        self.label = QLabel("Status: Stopped")
        self.label.setAlignment(Qt.AlignHCenter)
        self.label.setStyleSheet("font-size: 26px; color: #ff5555; font-weight: 700; padding-top: 4px;")
        root.addWidget(self.label)

        # ---- Coordinates ----
        self.coords = QLabel("Position: (x=0.00, y=0.00, θ=0.00)")
        self.coords.setAlignment(Qt.AlignHCenter)
        self.coords.setStyleSheet("font-size: 20px; color: #9aa0a6; padding-bottom: 12px;")
        root.addWidget(self.coords)

        # thin separator line
        sep = QLabel("")
        sep.setFixedHeight(2)
        sep.setStyleSheet("background-color: #00a092;")
        root.addWidget(sep)

        # ---- Buttons row (left column style on the SAME page) ----
        # We put buttons in a small left-side column and add stretch on the right
        row = QHBoxLayout()
        row.setSpacing(18)

        buttons_col = QVBoxLayout()
        buttons_col.setSpacing(12)

        btn_style = (
            "font-size: 18px; font-weight: 700; color: white; "
            "padding: 10px 18px; border-radius: 10px;"
        )
        # Smaller, squarer buttons
        btn_w, btn_h = 190, 54

        self.btn_start = QPushButton("▶ START")
        self.btn_start.setFixedSize(btn_w, btn_h)
        self.btn_start.setStyleSheet(btn_style + "background-color: #10b981;")
        self.btn_start.clicked.connect(self.on_start)
        buttons_col.addWidget(self.btn_start, alignment=Qt.AlignLeft)

        self.btn_stop = QPushButton("⏹ STOP")
        self.btn_stop.setFixedSize(btn_w, btn_h)
        self.btn_stop.setStyleSheet(btn_style + "background-color: #ef4444;")
        self.btn_stop.clicked.connect(self.on_stop)
        buttons_col.addWidget(self.btn_stop, alignment=Qt.AlignLeft)

        self.btn_restart = QPushButton("🔁 RESTART")
        self.btn_restart.setFixedSize(btn_w, btn_h)
        self.btn_restart.setStyleSheet(btn_style + "background-color: #3b82f6;")
        self.btn_restart.clicked.connect(self.on_restart)
        buttons_col.addWidget(self.btn_restart, alignment=Qt.AlignLeft)

        self.btn_close = QPushButton("❌ CLOSE")
        self.btn_close.setFixedSize(btn_w, btn_h)
        self.btn_close.setStyleSheet(btn_style + "background-color: #6b7280;")
        self.btn_close.clicked.connect(self.on_close)
        buttons_col.addWidget(self.btn_close, alignment=Qt.AlignLeft)

        # left column goes in the row, right side left empty for future widgets
        row.addLayout(buttons_col)
        row.addStretch(1)  # keeps buttons “hanging” on the left wall

        root.addLayout(row)

        # bottom stretch so there’s breathing room when tall
        root.addStretch(1)

        self.win.show()

        # ROS spin timer
        self.timer = QTimer()
        self.timer.timeout.connect(self._safe_spin)
        self.timer.start(10)

    # ---- ROS Callbacks / actions ----
    def pose_callback(self, msg: PoseStamped):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)
        self.coords.setText(f"Position: (x={self.x:.2f}, y={self.y:.2f}, θ={self.theta:.2f})")

    def on_start(self):
        self.run_pub.publish(Bool(data=True))
        self.label.setText("Status: Running")
        self.label.setStyleSheet("font-size: 26px; color: #22ff88; font-weight: 700;")

    def on_stop(self):
        self.run_pub.publish(Bool(data=False))
        self.cmd_pub.publish(self.zero_twist)
        self.label.setText("Status: Stopped")
        self.label.setStyleSheet("font-size: 26px; color: #ff5555; font-weight: 700;")

    def on_restart(self):
        self.label.setText("Status: Restarting…")
        self.label.setStyleSheet("font-size: 26px; color: #66a3ff; font-weight: 700;")
        if not self.restart_cli.service_is_ready():
            self.restart_cli.wait_for_service(timeout_sec=3.0)
        fut = self.restart_cli.call_async(Trigger.Request())
        fut.add_done_callback(self._after_restart)

    def _after_restart(self, fut):
        try:
            result = fut.result()
            self.get_logger().info(f"Restart: {result.message}")
        except Exception as e:
            self.get_logger().warn(f"Restart service error: {e}")
        self.label.setText("Status: Stopped")
        self.label.setStyleSheet("font-size: 26px; color: #ff5555; font-weight: 700;")

    def _safe_spin(self):
        if rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    def on_close(self):
        self.get_logger().info("🛑 Shutting down VerdantEye UI...")
        self.app.quit()
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


def main():
    rclpy.init()
    ui = RobotUI()

    def on_exit():
        ui.destroy_node()
        rclpy.shutdown()

    ui.app.aboutToQuit.connect(on_exit)
    try:
        sys.exit(ui.app.exec_())
    except KeyboardInterrupt:
        on_exit()

if __name__ == '__main__':
    main()
