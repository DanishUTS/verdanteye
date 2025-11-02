# ======================================================================
# file: src/ve_behaviours/ve_behaviours/bamboo_randomizer.py
# ======================================================================
#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import os
import random
import subprocess
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from rclpy.node import Node

# Prefer ros_gz services; fall back to CLI if absent.
try:
    from ros_gz_interfaces.srv import SpawnEntity, RemoveEntity
    HAS_GZ = True
except Exception:
    HAS_GZ = False


@dataclass
class Obstacle:
    name: str
    x: float
    y: float
    radius: float  # conservative keepout


def _parse_pose_xyz(pose_text: str) -> Tuple[float, float, float, float]:
    parts = [float(p) for p in (pose_text or "0 0 0 0 0 0").split()]
    parts += [0.0] * (6 - len(parts))
    x, y, z, _, _, yaw = parts[:6]
    return x, y, z, yaw


def _world_bounds_from_walls(includes: List[Tuple[str, str, str]]) -> Tuple[float, float, float, float]:
    xs, ys = [], []
    for name, _, pose_text in includes:
        if name.startswith("forest_wall"):
            x, y, *_ = _parse_pose_xyz(pose_text)
            xs.append(x)
            ys.append(y)
    if not xs or not ys:
        # Fallback to safe defaults matching your world scale.
        return -12.0, 12.0, -12.0, 12.0
    return min(xs), max(xs), min(ys), max(ys)


def _radius_for(name: str, uri: str) -> float:
    if name.startswith(("oak", "pine")):
        return 1.4
    if name.startswith("rock"):
        return 0.8
    if name.startswith("platypus"):
        return 0.6
    if name.startswith("bamboo_thicket"):
        return 0.9
    if name.startswith("forest_wall"):
        return 0.9
    return 0.8


def _load_includes_from_sdf(world_sdf_path: str) -> List[Tuple[str, str, str]]:
    tree = ET.parse(world_sdf_path)
    root = tree.getroot()
    out: List[Tuple[str, str, str]] = []
    for inc in root.findall(".//include"):
        name_el = inc.find("name")
        uri_el = inc.find("uri")
        pose_el = inc.find("pose")
        name = (name_el.text if name_el is not None else "").strip()
        uri = (uri_el.text if uri_el is not None else "").strip()
        pose_text = (pose_el.text if pose_el is not None else "0 0 0 0 0 0").strip()
        out.append((name, uri, pose_text))
    return out


def _build_obstacles(includes: List[Tuple[str, str, str]]) -> List[Obstacle]:
    obs: List[Obstacle] = []
    for name, uri, pose_text in includes:
        if name == "forest_plane":
            continue
        x, y, _, _ = _parse_pose_xyz(pose_text)
        obs.append(Obstacle(name=name, x=x, y=y, radius=_radius_for(name, uri)))
    return obs


def _too_close(x: float, y: float, to_list: Iterable[Obstacle], pad: float = 0.0) -> Optional[str]:
    for o in to_list:
        if math.hypot(x - o.x, y - o.y) < (o.radius + pad):
            return o.name
    return None


def _too_close_xy(x: float, y: float, others_xy: Iterable[Tuple[float, float]], min_spacing: float) -> bool:
    for ox, oy in others_xy:
        if math.hypot(x - ox, y - oy) < min_spacing:
            return True
    return False


def _include_xml(model_uri: str, name: str) -> str:
    return f"""<sdf version='1.8'>
  <include>
    <name>{name}</name>
    <uri>{model_uri}</uri>
  </include>
</sdf>
"""


class BambooRandomizer(Node):
    """Spawns exactly N bamboo (50/50 RED/GREEN) into the target world, avoiding trees/rocks/walls/Husky."""

    def __init__(self) -> None:
        super().__init__("bamboo_randomizer")

        # ---------------- Parameters ----------------
        self.declare_parameter("world_name", "large_demo")
        self.declare_parameter("num", 6)
        self.declare_parameter("seed", 0)
        self.declare_parameter("min_spacing", 1.8)
        self.declare_parameter("wall_margin", 1.0)
        self.declare_parameter("husky_exclusion_radius", 2.5)
        self.declare_parameter("delete_existing_bamboo", True)
        # Z axis (added last as requested; default sits on ground)
        self.declare_parameter("z_height", 0.0)

        self.world_name = str(self.get_parameter("world_name").value)
        self.num = int(self.get_parameter("num").value)
        self.seed = int(self.get_parameter("seed").value)
        self.min_spacing = float(self.get_parameter("min_spacing").value)
        self.wall_margin = float(self.get_parameter("wall_margin").value)
        self.husky_exclusion_radius = float(self.get_parameter("husky_exclusion_radius").value)
        self.delete_existing_bamboo = bool(self.get_parameter("delete_existing_bamboo").value)
        self.z_height = float(self.get_parameter("z_height").value)

        random.seed(self.seed if self.seed else time.time())

        # ---------------- World parse ----------------
        bringup_share = get_package_share_directory("41068_ignition_bringup")
        world_sdf_path = os.path.join(bringup_share, "worlds", f"{self.world_name}.sdf")
        if not os.path.isfile(world_sdf_path):
            raise FileNotFoundError(f"World SDF not found: {world_sdf_path}")

        includes = _load_includes_from_sdf(world_sdf_path)
        xmin, xmax, ymin, ymax = _world_bounds_from_walls(includes)
        xmin += self.wall_margin
        xmax -= self.wall_margin
        ymin += self.wall_margin
        ymax -= self.wall_margin

        obstacles = _build_obstacles(includes)
        obstacles.append(Obstacle("husky_spawn", 0.0, 0.0, self.husky_exclusion_radius))

        if self.delete_existing_bamboo:
            self._delete_existing_bamboo([o.name for o in obstacles if o.name.startswith("bamboo_thicket_")])

        # ---------------- Sample placements ----------------
        placements_xy: List[Tuple[float, float]] = []
        colors_uri: List[str] = []
        attempts, max_attempts = 0, 300

        while len(placements_xy) < self.num and attempts < max_attempts:
            attempts += 1
            x = random.uniform(xmin, xmax)
            y = random.uniform(ymin, ymax)

            if _too_close(x, y, obstacles, pad=0.2):
                continue
            if _too_close_xy(x, y, placements_xy, self.min_spacing):
                continue

            uri = "model://bamboo_thicket_RED" if random.random() < 0.5 else "model://bamboo_thicket_GREEN"
            placements_xy.append((x, y))
            colors_uri.append(uri)

        if len(placements_xy) < self.num:
            self.get_logger().warn(
                f"Placed {len(placements_xy)}/{self.num} after {attempts} attempts. "
                f"Lower 'min_spacing' or margins if you want more density."
            )

        # ---------------- Spawn ----------------
        self._spawn_batch(placements_xy, colors_uri)

        summary = [
            {"name": f"bamboo_rand_{i+1:02d}", "x": float(x), "y": float(y), "uri": uri}
            for i, ((x, y), uri) in enumerate(zip(placements_xy, colors_uri))
        ]
        self.get_logger().info("Bamboo placements:\n" + json.dumps(summary, indent=2))

        # Exit once done
        rclpy.shutdown()

    # ---------------- Helpers ----------------
    def _available_runners(self) -> list[list[str]]:
        """Return candidate CLI runners in the order they should be tried."""
        combos = [
            ["ros2", "run", "ros_gz_sim", "create"],      # Gazebo Garden+
            ["ros2", "run", "ros_ign_gazebo", "create"],  # Ignition Fortress/Edifice
        ]
        available = []
        for cmd in combos:
            pkg = cmd[2]
            try:
                out = subprocess.run(["ros2", "pkg", "executables", pkg],
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
                if b"create" in out.stdout:
                    available.append(cmd)
            except Exception:
                pass
        return available or combos

    def _spawn_batch(self, xys: List[Tuple[float, float]], uris: List[str]) -> None:
        if not xys:
            self.get_logger().warn("No placements computed; nothing to spawn.")
            return

        # Try service first (ros_gz)
        if HAS_GZ:
            cli = self.create_client(SpawnEntity, f"/world/{self.world_name}/create")
            if cli.wait_for_service(timeout_sec=5.0):
                for i, ((x, y), uri) in enumerate(zip(xys, uris)):
                    name = f"bamboo_rand_{i+1:02d}"
                    xml = _include_xml(uri, name)
                    req = SpawnEntity.Request()
                    pose = Pose()
                    pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(self.z_height)

                    # Field name differs across distros; set what exists.
                    if hasattr(req, "initial_pose"):
                        req.initial_pose = pose
                    elif hasattr(req, "pose"):
                        setattr(req, "pose", pose)

                    req.name = name
                    if hasattr(req, "allow_renaming"):
                        req.allow_renaming = False
                    req.xml = xml
                    if hasattr(req, "robot_namespace"):
                        req.robot_namespace = ""

                    fut = cli.call_async(req)
                    rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
                    ok = bool(fut.result())
                    self.get_logger().info(f"spawn[{name}] {'OK' if ok else 'FAILED'} (service)")
                return
            self.get_logger().warn("Spawn service unavailable — falling back to CLI.")

        # CLI fallback: robustly try ros_gz_sim then ros_ign_gazebo
        runners = self._available_runners()
        last_err = None
        for runner in runners:
            ok_runner = True
            for i, ((x, y), uri) in enumerate(zip(xys, uris)):
                name = f"bamboo_rand_{i+1:02d}"
                xml = _include_xml(uri, name)
                cmd = runner + [
                    "-world", self.world_name,
                    "-name", name,
                    "-x", f"{x:.3f}",
                    "-y", f"{y:.3f}",
                    "-z", f"{self.z_height:.3f}",
                    "-string", xml,
                ]
                try:
                    subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    self.get_logger().info(f"spawn[{name}] OK via: {' '.join(runner)}")
                except subprocess.CalledProcessError as e:
                    ok_runner = False
                    last_err = e.stderr.decode(errors="ignore")
                    self.get_logger().error(f"spawn[{name}] FAILED via {' '.join(runner)}\n{last_err}")
                    break
            if ok_runner:
                return

        msg = last_err or "unknown error"
        self.get_logger().error(f"All CLI runners failed. Last error:\n{msg}")

    def _delete_existing_bamboo(self, names: List[str]) -> None:
        """Best-effort removal to guarantee ONLY N exist."""
        if not HAS_GZ or not names:
            return
        cli = self.create_client(RemoveEntity, f"/world/{self.world_name}/remove")
        if not cli.wait_for_service(timeout_sec=3.0):
            return
        for n in names:
            try:
                req = RemoveEntity.Request()
                if hasattr(req, "name"):
                    req.name = n
                elif hasattr(req, "entity") and hasattr(req.entity, "name"):
                    req.entity.name = n
                fut = cli.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
            except Exception:
                pass


def main() -> None:
    rclpy.init()
    BambooRandomizer()
