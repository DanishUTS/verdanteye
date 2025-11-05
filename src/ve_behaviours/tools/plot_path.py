#!/usr/bin/env python3
import sys, yaml, cv2, numpy as np, argparse
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Path as PathMsg

class Saver(Node):
    def __init__(self, map_yaml, out_png, dx_m, dy_m):
        super().__init__('plot_path')
        self.out_png = out_png
        self.dx_m, self.dy_m = float(dx_m), float(dy_m)

        meta = yaml.safe_load(open(map_yaml, 'r'))
        img_path = (Path(map_yaml).parent / meta['image']).resolve()
        img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise RuntimeError(f"Failed to read map image: {img_path}")
        if int(meta.get('negate',0)) == 1:
            img = 255 - img
        self.img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        self.h, self.w = self.img.shape[:2]

        self.res = float(meta['resolution'])
        self.ox = float(meta['origin'][0]) + self.dx_m
        self.oy = float(meta['origin'][1]) + self.dy_m

        # VOLATILE subscriber (run this first, then publish /planned_path)
        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE)
        self.sub = self.create_subscription(PathMsg, '/planned_path', self.on_path, qos)
        self.get_logger().info(f"Waiting for /planned_path... (dx={self.dx_m} m, dy={self.dy_m} m, res={self.res} m/px)")

    def w2p(self, x, y):
        # round() avoids big jumps from floor truncation
        px = int(round((x - self.ox) / self.res))
        py_up = int(round((y - self.oy) / self.res))
        py = (self.h - 1) - py_up             # flip Y (map-up -> image-down)
        px = max(0, min(self.w - 1, px))
        py = max(0, min(self.h - 1, py))
        return px, py

    def on_path(self, msg: PathMsg):
        pts = [self.w2p(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(pts) >= 2:
            cv2.polylines(self.img, [np.array(pts, dtype=np.int32)], False, (0,0,255), 2, cv2.LINE_AA)
            for p in pts:
                cv2.circle(self.img, p, 2, (0,255,0), -1)
        cv2.imwrite(self.out_png, self.img)
        self.get_logger().info(f"Saved overlay to {self.out_png}")
        rclpy.shutdown()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map_yaml', nargs='?', default=str(Path.cwd() / "src/ve_behaviours/maps/forest.yaml"))
    ap.add_argument('out_png',  nargs='?', default=str(Path.cwd() / "src/ve_behaviours/tools/path_overlay.png"))
    ap.add_argument('--dx', type=float, default=0.0, help='x bias in metres (+right)')
    ap.add_argument('--dy', type=float, default=0.0, help='y bias in metres (+up)')
    args = ap.parse_args()

    rclpy.init()
    node = Saver(args.map_yaml, args.out_png, args.dx, args.dy)
    rclpy.spin(node)

if __name__ == "__main__":
    main()

