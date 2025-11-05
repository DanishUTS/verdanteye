#!/usr/bin/env python3
import os, sys, json, csv, subprocess
from dataclasses import dataclass
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QGridLayout, QHBoxLayout,
    QVBoxLayout, QScrollArea, QMessageBox
)
    # (imports trimmed to essentials)
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt, QTimer

HEALTH_DESCRIPTORS = [
    (20, "Critical failure — immediate intervention required."),
    (40, "Severely stressed — remediate as soon as possible."),
    (60, "Moderately stressed — monitor closely and treat."),
    (80, "Mild stress — plant should recover with care."),
    (101, "Healthy — keep routine checks.")
]


def describe_health(percent: float) -> str:
    for threshold, text in HEALTH_DESCRIPTORS:
        if percent < threshold:
            return text
    return HEALTH_DESCRIPTORS[-1][1]


@dataclass
class Item:
    id: int
    image_path: str
    color: str
    condition: str
    conf: float
    x: float
    y: float
    name: str
    health: int = 0

class ChecklistUI(Node):
    def __init__(self):
        super().__init__("ui_plants")
        self.items: List[Item] = []
        self.scans_dir = os.path.expanduser("~/.ros/plant_scans")
        self.sub = self.create_subscription(String, "/plant_scan/checklist", self.on_checklist, 10)

        self.app = QApplication(sys.argv)
        self.win = QWidget(); self.win.setWindowTitle("VerdantEye — Plant Checklist")
        self.win.setStyleSheet("background:#101010; color:white;"); self.win.resize(1280, 900)

        root = QVBoxLayout(self.win); root.setContentsMargins(12,12,12,12); root.setSpacing(8)
        bar = QHBoxLayout()
        btn_csv = QPushButton("Export CSV"); btn_open = QPushButton("Open Folder"); btn_clear = QPushButton("Clear")
        for b in (btn_csv, btn_open, btn_clear): b.setStyleSheet("background:#303030; padding:8px 14px; border-radius:8px;")
        btn_csv.clicked.connect(self.export_csv); btn_open.clicked.connect(self.open_folder); btn_clear.clicked.connect(self.clear_ui)
        bar.addWidget(btn_csv); bar.addWidget(btn_open); bar.addStretch(1); bar.addWidget(btn_clear); root.addLayout(bar)

        self.scroll = QScrollArea(); self.scroll.setWidgetResizable(True)
        self.gallery = QWidget(); self.grid = QGridLayout(self.gallery); self.grid.setSpacing(12)
        self.scroll.setWidget(self.gallery); root.addWidget(self.scroll, 1)
        self.win.show()

        self.timer = QTimer(); self.timer.timeout.connect(lambda: rclpy.spin_once(self, timeout_sec=0.01))
        self.timer.start(10)

    def on_checklist(self, msg: String):
        try: data = json.loads(msg.data)
        except Exception as e: self.get_logger().warn(f"Bad JSON: {e}"); return
        self.items.clear()
        for it in data.get("items", []):
            pose = it.get("pose", {})
            x_val = pose.get("x", it.get("x", 0.0))
            y_val = pose.get("y", it.get("y", 0.0))
            health_raw = it.get("health_percent", it.get("health", 0))
            try:
                health_num = float(health_raw)
            except Exception:
                health_num = 0.0
            health_val = int(max(0, min(100, round(health_num))))

            self.items.append(Item(
                id=int(it.get("id", 0)),
                image_path=str(it.get("image_path", "")),
                color=str(it.get("color", "green")),
                condition=str(it.get("condition", "healthy")),
                conf=float(it.get("confidence", 0.0)),
                x=float(x_val),
                y=float(y_val),
                name=str(it.get("bamboo_name", "")),
                health=health_val
            ))
        self.refresh_grid()

    def refresh_grid(self):
        while self.grid.count():
            w = self.grid.takeAt(0).widget()
            if w: w.deleteLater()
        cols = 3
        for i, it in enumerate(self.items):
            r, c = divmod(i, cols)
            self.grid.addWidget(self._make_card(it), r, c)

    def _make_card(self, it: Item):
        w = QWidget(); v = QVBoxLayout(w); v.setSpacing(6)
        img = QLabel(); img.setAlignment(Qt.AlignCenter); img.setFixedSize(370, 240)
        if it.image_path and os.path.exists(it.image_path):
            pm = QPixmap(it.image_path).scaled(370, 240, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            img.setPixmap(pm)
        else:
            img.setStyleSheet("background:#202020; border:1px solid #333;"); img.setText("(no image)"); img.setAlignment(Qt.AlignCenter)
        v.addWidget(img)
        if it.color == "red":
            status = "🔴 Toxic"
        elif it.color == "yellow":
            status = "🟡 Stressed"
        else:
            status = "🟢 Healthy"

        title = QLabel(
            f"Plant #{it.id}  {status}  "
            f"confidence {round(it.conf * 100)}%  at (x={it.x:.2f}, y={it.y:.2f})"
        )
        title.setStyleSheet("font-size:16px; color:#d0d0d0;")
        v.addWidget(title)

        desc = QLabel(f"Health {it.health}% — {describe_health(it.health)}")
        desc.setStyleSheet("font-size:14px; color:#a0a0a0;")
        v.addWidget(desc)
        return w

    def export_csv(self):
        os.makedirs(self.scans_dir, exist_ok=True)
        path = os.path.join(self.scans_dir, "summary.csv")
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["id","name","x","y","color","condition","confidence","image_path"])
            for it in self.items:
                w.writerow([it.id, it.name, it.x, it.y, it.color, it.condition, it.conf, it.image_path])
        QMessageBox.information(self.win, "Export CSV", f"Saved: {path}")

    def open_folder(self):
        try: subprocess.Popen(["xdg-open", self.scans_dir])
        except Exception: QMessageBox.information(self.win, "Open Folder", self.scans_dir)

    def clear_ui(self):
        self.items.clear(); self.refresh_grid()

def main():
    rclpy.init(); ui = ChecklistUI()
    def on_exit(): ui.destroy_node(); rclpy.shutdown()
    ui.app.aboutToQuit.connect(on_exit)
    try: sys.exit(ui.app.exec_())
    except KeyboardInterrupt: on_exit()

if __name__ == "__main__":
    main()
