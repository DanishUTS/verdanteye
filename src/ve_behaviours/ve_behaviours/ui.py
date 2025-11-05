#!/usr/bin/env python3
import sys, math, json, os, random, signal, subprocess
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
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

        # Publishers
        self.run_pub = self.create_publisher(Bool, '/ui/run_enabled', 10)
        # Publish directly to /cmd_vel (no mux required since Nav2 is disabled)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

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

        self.sim_label = QLabel("Simulation: offline")
        self.sim_label.setAlignment(Qt.AlignCenter)
        self.sim_label.setStyleSheet("font-size: 16px; color:#fca5a5; font-weight:600;")
        wr.addWidget(self.sim_label)

        self.coords = QLabel("Position: (x=0.00, y=0.00, θ=0.00)")
        self.coords.setStyleSheet("font-size: 14px; color:#a1a1aa;")
        self.coords.setAlignment(Qt.AlignCenter)
        wr.addWidget(self.coords)

        row = QHBoxLayout(); wr.addLayout(row)

        buttons_col = QVBoxLayout(); buttons_col.setSpacing(12)
        btn_style = ("font-size: 18px; font-weight: 700; color: white; padding: 10px 18px; border-radius: 10px;")
        btn_w, btn_h = 190, 54

        self.btn_sim_start = QPushButton("🚀 LAUNCH SIM"); self.btn_sim_start.setFixedSize(btn_w, btn_h)
        self.btn_sim_start.setStyleSheet(btn_style + "background-color: #2563eb;")
        self.btn_sim_start.clicked.connect(self.on_sim_start); buttons_col.addWidget(self.btn_sim_start, alignment=Qt.AlignLeft)

        self.btn_sim_stop = QPushButton("🛑 STOP SIM"); self.btn_sim_stop.setFixedSize(btn_w, btn_h)
        self.btn_sim_stop.setStyleSheet(btn_style + "background-color: #ef4444;")
        self.btn_sim_stop.clicked.connect(self.on_sim_stop); buttons_col.addWidget(self.btn_sim_stop, alignment=Qt.AlignLeft)

        self.btn_start = QPushButton("▶ START"); self.btn_start.setFixedSize(btn_w, btn_h)
        self.btn_start.setStyleSheet(btn_style + "background-color: #10b981;")
        self.btn_start.clicked.connect(self.on_start); buttons_col.addWidget(self.btn_start, alignment=Qt.AlignLeft)

        self.btn_stop = QPushButton("⏹ STOP"); self.btn_stop.setFixedSize(btn_w, btn_h)
        self.btn_stop.setStyleSheet(btn_style + "background-color: #ef4444;")
        self.btn_stop.clicked.connect(self.on_stop); buttons_col.addWidget(self.btn_stop, alignment=Qt.AlignLeft)

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
        self.sim_proc: Optional[subprocess.Popen] = None  # type: ignore[name-defined]
        self._sim_log = None
        self._sim_expect_shutdown = False
        self._sim_marked_running = False
        self.sim_launch_base = [
            "ros2", "launch", "41068_ignition_bringup", "41068_ignition.launch.py",
            "ui:=False", "rviz:=True"
        ]
        self._sim_check_timer = QTimer()
        self._sim_check_timer.setInterval(1000)
        self._sim_check_timer.timeout.connect(self._monitor_sim_process)
        self._sim_check_timer.start()

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

    def on_sim_start(self):
        if self._sim_running():
            self.get_logger().info("Simulation already running.")
            self.sim_label.setText("Simulation: running")
            self.sim_label.setStyleSheet("font-size: 16px; color:#4ade80; font-weight:600;")
            return
        if self.sim_proc is not None and self.sim_proc.poll() is not None:
            self._cleanup_sim_process(self.sim_proc.poll())

        log_path = Path.home() / ".ros" / "verdant_sim_launch.log"
        try:
            log_path.parent.mkdir(parents=True, exist_ok=True)
        except Exception:
            pass
        try:
            self._sim_log = open(log_path, "a", buffering=1, encoding="utf-8")
            self._sim_log.write(f"[{datetime.now().isoformat(timespec='seconds')}] Launching simulation...\n")
        except Exception:
            self._sim_log = None

        seed_value = random.randint(1, 2**31 - 1)
        launch_cmd = self.sim_launch_base + [f"bamboo_seed:={seed_value}"]

        try:
            self.sim_proc = subprocess.Popen(
                launch_cmd,
                stdout=self._sim_log or subprocess.DEVNULL,
                stderr=subprocess.STDOUT if self._sim_log else subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            self._sim_expect_shutdown = False
            self._sim_marked_running = False
            self.sim_label.setText("Simulation: launching…")
            self.sim_label.setStyleSheet("font-size: 16px; color:#fde68a; font-weight:600;")
        except Exception as exc:
            self.get_logger().error(f"Failed to start simulation: {exc}")
            if self._sim_log:
                try:
                    self._sim_log.write(f"[{datetime.now().isoformat(timespec='seconds')}] Launch failed: {exc}\n")
                except Exception:
                    pass
            self._cleanup_sim_process(exit_code=None, failed=True)

    def on_sim_stop(self):
        self._request_sim_stop(force=False)

    def _request_sim_stop(self, force: bool) -> None:
        if self.sim_proc is None:
            self.sim_label.setText("Simulation: offline")
            self.sim_label.setStyleSheet("font-size: 16px; color:#fca5a5; font-weight:600;")
            return
        if not self._sim_running():
            self._cleanup_sim_process(self.sim_proc.poll())
            return

        sig = signal.SIGINT if not force else signal.SIGKILL
        self._sim_expect_shutdown = True
        self.sim_label.setText("Simulation: stopping…" if not force else "Simulation: terminating…")
        self.sim_label.setStyleSheet("font-size: 16px; color:#fca5a5; font-weight:600;")
        try:
            os.killpg(os.getpgid(self.sim_proc.pid), sig)
        except ProcessLookupError:
            self._cleanup_sim_process(self.sim_proc.poll())
        except Exception as exc:
            self.get_logger().warn(f"Failed to send signal {sig}: {exc}")
            if not force:
                self._request_sim_stop(force=True)

    def _sim_running(self) -> bool:
        return self.sim_proc is not None and self.sim_proc.poll() is None

    def _monitor_sim_process(self):
        if self.sim_proc is None:
            return
        exit_code = self.sim_proc.poll()
        if exit_code is None:
            if not self._sim_marked_running:
                self.sim_label.setText("Simulation: running")
                self.sim_label.setStyleSheet("font-size: 16px; color:#4ade80; font-weight:600;")
                self._sim_marked_running = True
            return
        self._cleanup_sim_process(exit_code)

    def _cleanup_sim_process(self, exit_code: Optional[int], failed: bool = False) -> None:
        expected = self._sim_expect_shutdown
        self._sim_expect_shutdown = False
        self._sim_marked_running = False
        if self.sim_proc is not None:
            self.sim_proc = None
        self._close_sim_log()

        if failed:
            self.sim_label.setText("Simulation: launch failed")
            self.sim_label.setStyleSheet("font-size: 16px; color:#f87171; font-weight:600;")
            return

        if exit_code is None:
            exit_code = 0
        if exit_code == 0:
            text = "Simulation: stopped" if expected else "Simulation: exited"
            color = "#fca5a5" if expected else "#facc15"
        else:
            text = f"Simulation: crashed (code {exit_code})"
            color = "#f87171"
        self.sim_label.setText(text)
        self.sim_label.setStyleSheet(f"font-size: 16px; color:{color}; font-weight:600;")

    def _close_sim_log(self) -> None:
        if self._sim_log:
            try:
                self._sim_log.write(f"[{datetime.now().isoformat(timespec='seconds')}] Simulation log closed.\n")
            except Exception:
                pass
            try:
                self._sim_log.close()
            except Exception:
                pass
            self._sim_log = None

    def on_start(self):
        if not self._sim_running():
            self.get_logger().warn("Simulation is not running; launch it before starting automation.")
            self.label.setText("Status: Sim offline")
            self.label.setStyleSheet("font-size: 22px; color: #f97316; font-weight: 700;")
            return
        if self._stop_timer.isActive():
            self._stop_timer.stop()
        self.run_pub.publish(Bool(data=True))
        self.label.setText("Status: Running"); self.label.setStyleSheet("font-size: 22px; color: #10b981; font-weight: 700;")

    def on_stop(self):
        self.run_pub.publish(Bool(data=False))
        self._queue_stop_twists()
        self.label.setText("Status: Stopped"); self.label.setStyleSheet("font-size: 22px; color: #ff5555; font-weight: 700;")

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
        self._monitor_sim_process()

    def on_close(self):
        self.get_logger().info("🛑 Shutting down VerdantEye UI...")
        try:
            self._stop_timer.stop()
        except Exception:
            pass
        try:
            self._sim_check_timer.stop()
        except Exception:
            pass
        try:
            self.timer.stop()
        except Exception:
            pass
        self._request_sim_stop(force=True)
        self._close_sim_log()
        self.app.quit()
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

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
