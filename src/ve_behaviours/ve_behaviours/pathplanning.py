#!/usr/bin/env python3
# ve_behaviours/ve_behaviours/pathplanning.py
#
# Global path planner over a saved occupancy grid with auto map discovery.
# - Auto-finds <package>/maps/forest_map.yaml (installed share or source tree).
# - Orders targets from origin by Euclidean distance (closest → furthest).
# - Builds inflated costmap (distance transform) for safer global A*.
# - Goal tolerance: if goal cell is blocked, snap to nearest free within tol.
# - Prints waypoints for each leg, publishes a single /planned_path for RViz.
#
# Params:
#   map_yaml          (string)  if empty -> auto-find maps/forest_map.yaml
#   targets_flat      (double[]) [x1,y1, x2,y2, ...] (plant poses)
#   origin_xy         (double[]) [x0, y0]  (default [0.0, 0.0])
#   goal_tolerance_m  (double)  default 1.0
#   clearance_m       (double)  inflation radius (m), default 0.40
#   spacing_m         (double)  waypoint spacing (m), default 0.75
#   publish_path      (bool)    default True
#
from __future__ import annotations
import os, math, yaml, cv2, numpy as np, rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from pathlib import Path as P
from typing import List, Tuple, Optional

# ---------- map <-> world helpers ----------
def world_to_px(meta, x, y) -> Tuple[int,int]:
    # Convert world metres (x,y) into map pixel indices (col,row) using map origin+resolution.
    # No rounding here—int() truncs towards zero which is fine for cell addressing.
    res = float(meta['resolution']); ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    return int((x - ox) / res), int((y - oy) / res)

def px_to_world(meta, px, py) -> Tuple[float,float]:
    # Convert map pixel back to world metres, sampling at each cell centre (+0.5).
    res = float(meta['resolution']); ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    return float(ox + (px + 0.5) * res), float(oy + (py + 0.5) * res)

# ---------- path utils ----------
def respace(points_world: List[Tuple[float,float]], spacing: float) -> List[Tuple[float,float]]:
    # Takes a polyline and redistributes points every ~spacing metres.
    # Keeps endpoints; linear interpolation along each segment.
    if len(points_world) < 2: return points_world
    segs=[]; d=[0.0]
    for i in range(len(points_world)-1):
        ax,ay = points_world[i]; bx,by = points_world[i+1]
        L = math.hypot(bx-ax, by-ay); segs.append(((ax,ay),(bx,by),L)); d.append(d[-1]+L)
    total = d[-1]
    if total < 1e-6: return points_world
    n = max(2, int(total/spacing)+1)  # at least 2 points
    targets = [i*total/(n-1) for i in range(n)]
    out=[]; si=0; acc=0.0
    for t in targets:
        # Advance until current segment covers the target distance
        while si < len(segs) and acc + segs[si][2] < t:
            acc += segs[si][2]; si += 1
        if si >= len(segs): out.append(points_world[-1]); continue
        (ax,ay),(bx,by),L = segs[si]; u = 0.0 if L<1e-6 else (t-acc)/L
        out.append((ax + u*(bx-ax), ay + u*(by-ay)))
    return out

def choose_goal_within_tolerance(free_mask: np.ndarray, goal_px: Tuple[int,int], tol_px: int) -> Optional[Tuple[int,int]]:
    # If the “exact” goal cell is blocked, search a small square window (radius tol_px)
    # and pick the closest free cell (Euclidean). Returns None if nothing is free.
    h, w = free_mask.shape[:2]
    gx, gy = goal_px
    best = None; best_d2 = 1e18
    r = tol_px
    x0, x1 = max(0, gx-r), min(w-1, gx+r)
    y0, y1 = max(0, gy-r), min(h-1, gy+r)
    for y in range(y0, y1+1):
        dy2 = (y-gy)*(y-gy)
        for x in range(x0, x1+1):
            d2 = (x-gx)*(x-gx) + dy2
            if d2 <= r*r and free_mask[y,x] == 255 and d2 < best_d2:
                best_d2, best = d2, (x,y)
    return best

def build_costmap(img_u8: np.ndarray, res: float, clearance_m: float) -> Tuple[np.ndarray, np.ndarray]:
    # Treat unknown (grey) as occupied to stay conservative.
    if img_u8.ndim == 3:
        img_u8 = cv2.cvtColor(img_u8, cv2.COLOR_BGR2GRAY)
    # Binary “free” mask: white-ish pixels are free, everything else not-free.
    free = (img_u8 > 250).astype(np.uint8) * 255
    # Inflate obstacles by robot clearance (erode free-space by radius).
    px_clear = max(0, int(clearance_m / res))
    if px_clear > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*px_clear+1, 2*px_clear+1))
        free = cv2.erode(free, k)
    # Distance transform on free space gives distance to nearest obstacle (in pixels).
    inv = (free == 255).astype(np.uint8)
    dist = cv2.distanceTransform(inv, cv2.DIST_L2, 3)  # pixels
    dist_m = dist * res
    # Cost rises near obstacles; stays ~1 when far. Smooth and bounded to avoid infinities.
    eps = 0.05
    cost = 1.0 + 3.0 * (1.0 / (eps + dist_m)) * (eps)  # near obstacle -> ~4, far -> ~1
    return free, cost.astype(np.float32)

def astar_weighted(free_u8: np.ndarray, cost_f32: np.ndarray, start: Tuple[int,int], goal: Tuple[int,int]) -> List[Tuple[int,int]]:
    import heapq
    h, w = free_u8.shape[:2]
    sx, sy = start; gx, gy = goal
    # Quick bounds & occupancy sanity checks
    if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h): return []
    if free_u8[sy, sx] != 255 or free_u8[gy, gx] != 255: return []
    # 8-connected grid with diagonal movement
    neigh = [(-1,0,1.0),(1,0,1.0),(0,-1,1.0),(0,1,1.0),(-1,-1,math.sqrt(2)),(-1,1,math.sqrt(2)),(1,-1,math.sqrt(2)),(1,1,math.sqrt(2))]
    g = {start:0.0}
    f = {start: math.hypot(gx-sx, gy-sy)}  # straight-line heuristic
    came = {}
    openh = [(f[start], start)]
    closed = set()
    while openh:
        _, u = heapq.heappop(openh)
        if u in closed: continue
        if u == goal:
            # Reconstruct path by walking parents backwards.
            path=[u]
            while u in came:
                u = came[u]; path.append(u)
            path.reverse(); return path
        closed.add(u)
        ux, uy = u
        for dx,dy,dl in neigh:
            vx, vy = ux+dx, uy+dy
            # Skip out-of-bounds and blocked cells.
            if vx<0 or vy<0 or vx>=w or vy>=h: continue
            if free_u8[vy, vx] != 255: continue
            # Weighted step cost: longer diagonal moves cost more; proximity to obstacles costs more.
            step_cost = dl * 0.5 * (cost_f32[uy,ux] + cost_f32[vy,vx])
            c = g[u] + step_cost
            if c < g.get((vx,vy), float('inf')):
                g[(vx,vy)] = c
                came[(vx,vy)] = u
                hcost = math.hypot(gx - vx, gy - vy)
                f[(vx,vy)] = c + hcost
                heapq.heappush(openh, (f[(vx,vy)], (vx,vy)))
    return []

# ---------- map auto-discovery ----------
def resolve_map_yaml(default_name: str = "forest_map.yaml") -> P:
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = P(get_package_share_directory('ve_behaviours'))
        candidate = (share_dir / 'maps' / default_name).resolve()
        if candidate.is_file():
            return candidate
    except Exception:
        pass
    here = P(__file__).resolve()
    src_maps = (here.parent.parent / 'maps' / default_name).resolve()
    if src_maps.is_file():
        return src_maps
    raise FileNotFoundError(f"Could not find {default_name} in package share or source maps folder.")

class TargetPlanner(Node):
    def __init__(self):
        super().__init__('pathplanning')

        # ----- parameters -----
        self.declare_parameter('map_yaml', '')
        # Flattened [x1,y1, x2,y2,...] target list.
        self.declare_parameter('targets_flat', [0.0,-5.0, -9.0,6.0, 5.0,1.0, -2.0,7.0, 8.0,-3.0, -7.0,1.0])
        self.declare_parameter('origin_xy', [0.0, 0.0])
        self.declare_parameter('goal_tolerance_m', 1.0)
        self.declare_parameter('clearance_m', 1.00)
        self.declare_parameter('spacing_m', 0.75)
        self.declare_parameter('publish_path', True)

        # ----- resolve map -----
        map_yaml = self.get_parameter('map_yaml').get_parameter_value().string_value.strip()
        if map_yaml:
            # If user gave a relative path, resolve it against current working dir.
            map_yaml_p = P(map_yaml)
            if not map_yaml_p.is_absolute():
                map_yaml_p = (P.cwd() / map_yaml_p).resolve()
        else:
            map_yaml_p = resolve_map_yaml("forest_map.yaml")

        # Load map metadata (YAML) and its image (pgm/png). Respect negate flag.
        meta = yaml.safe_load(open(map_yaml_p, 'r'))
        img_path = (map_yaml_p.parent / meta['image']).resolve()
        img = cv2.imread(str(img_path), cv2.IMREAD_UNCHANGED)
        if img is None:
            raise RuntimeError(f"Failed to read map image: {img_path}")
        if int(meta.get('negate', 0)) == 1:
            img = 255 - img

        res = float(meta['resolution'])
        clearance = float(self.get_parameter('clearance_m').value)
        free_u8, cost_f32 = build_costmap(img, res, clearance)  # free mask + “how risky” terrain cost

        # ----- targets & origin -----
        flat = list(self.get_parameter('targets_flat').value)
        if len(flat) % 2 != 0:
            # Debug-style error because this one is easy to fat-finger
            raise RuntimeError("targets_flat must have even length [x1,y1,x2,y2,...].")
        targets = [(float(flat[i]), float(flat[i+1])) for i in range(0, len(flat), 2)]
        ox, oy = list(self.get_parameter('origin_xy').value)
        origin = (float(ox), float(oy))

        # Reorder targets by Euclidean distance from origin (nearest first).
        def ed2(p): return (p[0]-origin[0])**2 + (p[1]-origin[1])**2
        ordered = sorted(targets, key=ed2)

        print(f"\n# Using map: {map_yaml_p}")
        print("# Visit order (closest → furthest from origin):")
        for i, t in enumerate(ordered):
            print(f"{i+1:02d}. ({t[0]:.2f}, {t[1]:.2f})  dist={math.hypot(t[0]-origin[0], t[1]-origin[1]):.2f} m")

        spacing = float(self.get_parameter('spacing_m').value)
        goal_tol_m = float(self.get_parameter('goal_tolerance_m').value)
        tol_px = max(0, int(goal_tol_m / res))  # convert tolerance in metres into pixels

        # Snap origin onto nearest free cell if it happens to be inside an obstacle.
        start_px = world_to_px(meta, origin[0], origin[1])
        if free_u8[start_px[1], start_px[0]] != 255:
            snap = choose_goal_within_tolerance(free_u8, start_px, max(tol_px, 2))
            if snap is None:
                raise RuntimeError(f"Origin {origin} is in collision and no free cell within {goal_tol_m} m.")
            start_px = snap
            origin = px_to_world(meta, *start_px)
            self.get_logger().warn(f"Origin snapped to free at {origin}")

        # Plan each leg in sequence: start -> target1 -> target2 -> ...
        current_px = start_px
        stitched_world: List[Tuple[float,float]] = []
        leg_counter = 0
        for tgt in ordered:
            goal_px = world_to_px(meta, tgt[0], tgt[1])
            # If a target falls on an obstacle, try a local “snap” within tolerance to find a nearby free cell.
            if free_u8[goal_px[1], goal_px[0]] != 255:
                sub = choose_goal_within_tolerance(free_u8, goal_px, tol_px)
                if sub is None:
                    self.get_logger().warn(f"Goal {tgt} not reachable within {goal_tol_m} m — skipping.")
                    continue
                goal_px = sub

            # Weighted A* through free space using the “risk” costmap
            path_px = astar_weighted(free_u8, cost_f32, current_px, goal_px)
            leg_counter += 1
            if not path_px:
                self.get_logger().warn(f"[Leg {leg_counter}] No path from {px_to_world(meta, *current_px)} to {tgt} — skipped.")
                continue

            # Greedy line-of-sight shortcut pass (keeps endpoints; jumps over “visible” intermediate points).
            def los(a, b):
                x0,y0 = a; x1,y1 = b
                dx = abs(x1-x0); dy = -abs(y1-y0)
                sx = 1 if x0<x1 else -1; sy = 1 if y0<y1 else -1
                err = dx + dy; x,y = x0,y0
                h,w = free_u8.shape[:2]
                while True:
                    if x<0 or y<0 or x>=w or y>=h or free_u8[y,x] != 255: return False
                    if x==x1 and y==y1: break
                    e2=2*err
                    if e2>=dy: err+=dy; x+=sx
                    if e2<=dx: err+=dx; y+=sy
                return True

            sp=[path_px[0]]; i=0
            while i < len(path_px)-1:
                j=len(path_px)-1
                # Walk j backwards until we find the furthest point still in line-of-sight from i
                while j>i+1 and not los(path_px[i], path_px[j]):
                    j-=1
                sp.append(path_px[j]); i=j

            # Convert pixels back to metres and smooth spacing for nicer downstream following.
            path_world = [px_to_world(meta, *p) for p in sp]
            path_world = respace(path_world, spacing)

            print(f"\n# Leg {leg_counter}: {px_to_world(meta,*current_px)}  →  {tgt}  (via {len(path_world)} wps)")
            print("#   idx     x (m)      y (m)    yaw (rad)")
            leg_wps=[]
            for k,(x,y) in enumerate(path_world):
                yaw = math.atan2(path_world[k+1][1]-y, path_world[k+1][0]-x) if k < len(path_world)-1 else 0.0
                leg_wps.append((x,y,yaw))
                print(f"    {k:03d}   {x:8.3f}  {y:8.3f}   {yaw:7.3f}")

            # Stitch legs: avoid duplicating the first point of each subsequent leg.
            if not stitched_world:
                stitched_world.extend([(x,y) for (x,y,_) in leg_wps])
            else:
                stitched_world.extend([(x,y) for (x,y,_) in leg_wps[1:]])

            # Move “current” to the last achieved goal for the next leg.
            current_px = (sp[-1][0], sp[-1][1])

        # Final orientation pass for the stitched list (compute yaw to the next point).
        final_wps=[]
        for i,(x,y) in enumerate(stitched_world):
            if i < len(stitched_world)-1:
                nx,ny = stitched_world[i+1]
                yaw = math.atan2(ny-y, nx-x)
            else:
                yaw = 0.0
            final_wps.append((x,y,yaw))

        print(f"\n# Total waypoints across all legs: {len(final_wps)}\n")

        # Publish once to /planned_path so RViz can visualise.
        if bool(self.get_parameter('publish_path').value) and final_wps:
            # NOTE: QoS is default (volatile). If late subscribers need it,
            # switch this to TRANSIENT_LOCAL in code later.
            pub = self.create_publisher(Path, '/planned_path', 10)
            msg = Path(); msg.header.frame_id = 'map'
            for x,y,yaw in final_wps:
                p = PoseStamped(); p.header.frame_id='map'
                p.pose.position.x = x; p.pose.position.y = y
                p.pose.orientation.z = math.sin(yaw/2.0); p.pose.orientation.w = math.cos(yaw/2.0)
                msg.poses.append(p)
            pub.publish(msg)
            # Tiny debug nudge—helps when sanity checking in the log.
            self.get_logger().info(f"Published /planned_path with {len(msg.poses)} poses.")
        rclpy.shutdown()

def main():
    # rclpy app bootstrap—Node is constructed and does all the work in __init__ (one-shot style).
    rclpy.init()
    TargetPlanner()

if __name__ == '__main__':
    main()
