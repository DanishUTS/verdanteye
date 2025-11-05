#!/usr/bin/env python3
import sys, math, json
from pathlib import Path
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QGridLayout, QFrame
)
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt, QTimer

class GalleryCell(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(6, 6, 6, 6)
        self.pic = QLabel(); self.pic.setFixedSize(260, 160); self.pic.setAlignment(Qt.AlignCenter)
        self.caption = QLabel("—"); self.caption.setAlignment(Qt.AlignCenter)
        self.caption.setStyleSheet("font-size: 14px; color: #cbd5e1;")
        self.pic.setStyleSheet("background:#111;border:1px solid #333;")
        lay.addWidget(self.pic); lay.addWidget(self.caption)

    def set_item(self, img_path: str, text: str):
        if img_path and Path(img_path).exists():
            pm = QPixmap(img_path).scaled(self.pic.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.pic.setPixmap(pm)
        else:
            self.pic.setPixmap(QPixmap())
        self.caption.setText(text or "—")

    def clear(self):
        self.set_item("", "—")

class RobotUI(Node):
    def __init__(self):
        super().__init__('ui')

        # Publishers / services
        self.run_pub = self.create_publisher(Bool, '/ui/run_enabled', 10)
        # Publish directly to /cmd_vel (no mux required since Nav2 is disabled)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.restart_cli = self.create_client(Trigger, '/ui/restart')

        # Pose from EKF odometry (works even when Nav2 is off)
        self.pose_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Checklist (JSON with image paths + labels) from AutoTargetScan
        self.create_subscription(String, '/plant_scan/checklist', self.on_checklist, 10)

        # --- Qt setup ---
        self.app = QApplication(sys.argv)
        self.window = QWidget(); self.window.setWindowTitle("VerdantEye Robotics Control")
        self.window.setStyleSheet("background-color:#0b0f12; color:#e5e7eb; font-family:Inter,Arial;")
        wr = QVBoxLayout(self.window); wr.setContentsMargins(16, 16, 16, 16); wr.setSpacing(18)

        title = QLabel("VerdantEye Robotics Control"); title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 28px; font-weight: 800; color: #22d3ee;")
        wr.addWidget(title)

        self.label = QLabel("Status: Stopped"); self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("font-size: 22px; color: #ff5555; font-weight: 700;")
        wr.addWidget(self.label)

        self.coords = QLabel("Position: (x=0.00, y=0.00, θ=0.00)")
        self.coords.setStyleSheet("font-size: 14px; color:#a1a1aa;")
        self.coords.setAlignment(Qt.AlignCenter)
        wr.addWidget(self.coords)

        row = QHBoxLayout(); wr.addLayout(row)

        buttons_col = QVBoxLayout(); buttons_col.setSpacing(12)
        btn_style = ("font-size: 18px; font-weight: 700; color: white; padding: 10px 18px; border-radius: 10px;")
        btn_w, btn_h = 190, 54

        self.btn_start = QPushButton("▶ START"); self.btn_start.setFixedSize(btn_w, btn_h)
        self.btn_start.setStyleSheet(btn_style + "background-color: #10b981;")
        self.btn_start.clicked.connect(self.on_start); buttons_col.addWidget(self.btn_start, alignment=Qt.AlignLeft)

        self.btn_stop = QPushButton("⏹ STOP"); self.btn_stop.setFixedSize(btn_w, btn_h)
        self.btn_stop.setStyleSheet(btn_style + "background-color: #ef4444;")
        self.btn_stop.clicked.connect(self.on_stop); buttons_col.addWidget(self.btn_stop, alignment=Qt.AlignLeft)

        self.btn_restart = QPushButton("🔁 RESTART"); self.btn_restart.setFixedSize(btn_w, btn_h)
        self.btn_restart.setStyleSheet(btn_style + "background-color: #3b82f6;")
        self.btn_restart.clicked.connect(self.on_restart); buttons_col.addWidget(self.btn_restart, alignment=Qt.AlignLeft)

        self.btn_close = QPushButton("❌ CLOSE"); self.btn_close.setFixedSize(btn_w, btn_h)
        self.btn_close.setStyleSheet(btn_style + "background-color: #6b7280;")
        self.btn_close.clicked.connect(self.on_close); buttons_col.addWidget(self.btn_close, alignment=Qt.AlignLeft)

        row.addLayout(buttons_col)

        gallery_wrap = QVBoxLayout()
        glabel = QLabel("Plant Checklist"); glabel.setAlignment(Qt.AlignLeft)
        glabel.setStyleSheet("font-size:20px;color:#a7f3d0;")
        gallery_wrap.addWidget(glabel)

        self.grid = QGridLayout(); self.grid.setHorizontalSpacing(12); self.grid.setVerticalSpacing(12)
        self.cells: List[GalleryCell] = []
        for i in range(2):
            for j in range(3):
                cell = GalleryCell(); self.grid.addWidget(cell, i, j); self.cells.append(cell)
        gallery_wrap.addLayout(self.grid)
        row.addLayout(gallery_wrap, stretch=1)

        self.window.resize(1100, 720)
        self.window.show()

        self.items = []
        self.x = self.y = self.theta = 0.0
        self._stop_burst_remaining = 0
        self._stop_timer = QTimer()
        self._stop_timer.setInterval(60)
        self._stop_timer.timeout.connect(self._stop_burst_tick)

        # Redraw timer to keep Qt responsive alongside rclpy
        self.timer = QTimer(); self.timer.timeout.connect(self._safe_spin); self.timer.start(10)

    # ------------- ROS callbacks -------------
    def on_checklist(self, msg: String):
        try:
            data = json.loads(msg.data or "{}")
        except Exception:
            return
        self.items = data.get("items", [])
        for i in range(6):
            if i < len(self.items):
                it = self.items[i]
                label = f"#{i+1}  {it.get('color','?').upper()}  {it.get('condition','?')}\n" \
                        f"({it.get('pose',{}).get('x','?')},{it.get('pose',{}).get('y','?')})  conf={it.get('confidence',0)}"
                self.cells[i].set_item(it.get('image_path',"") or "", label)
            else:
                self.cells[i].clear()

    def on_start(self):
        if self._stop_timer.isActive():
            self._stop_timer.stop()
        self.run_pub.publish(Bool(data=True))
        self.label.setText("Status: Running"); self.label.setStyleSheet("font-size: 22px; color: #10b981; font-weight: 700;")

    def on_stop(self):
        self.run_pub.publish(Bool(data=False))
        self._queue_stop_twists()
        self.label.setText("Status: Stopped"); self.label.setStyleSheet("font-size: 22px; color: #ff5555; font-weight: 700;")

    def on_restart(self):
        if self.restart_cli.wait_for_service(timeout_sec=1.0):
            try:
                from std_srvs.srv import Trigger
                fut = self.restart_cli.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
            except Exception:
                pass

    def _queue_stop_twists(self):
        # send a short burst of zero twists so controllers actually halt
        self._stop_burst_remaining = max(self._stop_burst_remaining, 10)
        self._emit_stop_twist()
        if not self._stop_timer.isActive():
            self._stop_timer.start()

    def _emit_stop_twist(self):
        self.cmd_pub.publish(Twist())

    def _stop_burst_tick(self):
        if self._stop_burst_remaining <= 0:
            self._stop_timer.stop()
            return
        self._stop_burst_remaining -= 1
        self._emit_stop_twist()

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        self.x, self.y, self.theta = float(p.x), float(p.y), float(theta)
        self.coords.setText(f"Position: (x={self.x:.2f}, y={self.y:.2f}, θ={self.theta:.2f})")

    def _safe_spin(self):
        if rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    def on_close(self):
        self.get_logger().info("🛑 Shutting down VerdantEye UI...")
        self.app.quit(); self.destroy_node(); rclpy.shutdown(); sys.exit(0)

def main():
    rclpy.init()
    ui = RobotUI()
    def on_exit():
        ui.destroy_node(); rclpy.shutdown()
    ui.app.aboutToQuit.connect(on_exit)
    try:
        sys.exit(ui.app.exec_())
    except KeyboardInterrupt:
        on_exit()

if __name__ == '__main__':
    main()
