#!/usr/bin/env python3
import rclpy, math, yaml, cv2, numpy as np
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def world_to_px(meta, x, y):
    res = meta['resolution']; ox, oy = meta['origin'][0], meta['origin'][1]
    px = int((x - ox) / res); py = int((y - oy) / res)
    return px, py

def px_to_world(meta, px, py):
    res = meta['resolution']; ox, oy = meta['origin'][0], meta['origin'][1]
    x = ox + (px + 0.5) * res; y = oy + (py + 0.5) * res
    return float(x), float(y)

def bresenham_los(occ, a, b):
    """Line-of-sight check on binary free=255 map; returns True if clear."""
    x0,y0 = a; x1,y1 = b
    dx = abs(x1-x0); dy = -abs(y1-y0)
    sx = 1 if x0<x1 else -1; sy = 1 if y0<y1 else -1
    err = dx + dy
    x,y = x0,y0
    while True:
        if occ[y, x] != 255: return False
        if x == x1 and y == y1: break
        e2 = 2*err
        if e2 >= dy: err += dy; x += sx
        if e2 <= dx: err += dx; y += sy
    return True

def astar(occ, start, goal):
    """A* on binary free map with 8-connectivity, returns list of px points."""
    import heapq
    if occ[start[1], start[0]] != 255 or occ[goal[1], goal[0]] != 255:
        return []
    w,h = occ.shape[1], occ.shape[0]
    neigh = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    g = {start:0.0}
    f = {start:math.hypot(goal[0]-start[0], goal[1]-start[1])}
    came = {}
    openh = [(f[start], start)]
    closed = set()
    while openh:
        _, u = heapq.heappop(openh)
        if u in closed: continue
        if u == goal:
            # reconstruct
            path=[u]
            while u in came:
                u = came[u]; path.append(u)
            path.reverse(); return path
        closed.add(u)
        for dx,dy in neigh:
            vx, vy = u[0]+dx, u[1]+dy
            if vx<0 or vy<0 or vx>=w or vy>=h: continue
            if occ[vy, vx] != 255: continue
            cost = g[u] + (1.0 if dx==0 or dy==0 else math.sqrt(2))
            if cost < g.get((vx,vy), float('inf')):
                g[(vx,vy)] = cost
                came[(vx,vy)] = u
                hcost = math.hypot(goal[0]-vx, goal[1]-vy)
                f[(vx,vy)] = cost + hcost
                heapq.heappush(openh, (f[(vx,vy)], (vx,vy)))
    return []

def shortcut_path(occ, path_px):
    """Greedy shortcutting: keep points only when LOS fails."""
    if not path_px: return []
    out=[path_px[0]]
    i=0
    while i < len(path_px)-1:
        j = len(path_px)-1
        # find the furthest j we can see from i
        while j > i+1 and not bresenham_los(occ, path_px[i], path_px[j]):
            j -= 1
        out.append(path_px[j])
        i = j
    return out

def respace(points_world, spacing):
    """Evenly space points along polyline; keep headings."""
    if len(points_world) < 2: return points_world
    # build cumulative lengths
    segs=[]; dists=[0.0]
    for i in range(len(points_world)-1):
        ax,ay = points_world[i]; bx,by = points_world[i+1]
        segs.append(((ax,ay),(bx,by), math.hypot(bx-ax, by-ay)))
        dists.append(dists[-1]+segs[-1][2])
    total = dists[-1]
    if total < 1e-6: return points_world
    n = max(2, int(total/spacing)+1)
    target = [i*total/(n-1) for i in range(n)]
    out=[]
    si=0; acc=0.0
    for t in target:
        # advance to segment containing distance t
        while si < len(segs) and acc + segs[si][2] < t:
            acc += segs[si][2]; si += 1
        if si >= len(segs): out.append(points_world[-1]); continue
        a,b,L = segs[si]
        u = 0.0 if L<1e-6 else (t-acc)/L
        x = a[0] + u*(b[0]-a[0]); y = a[1] + u*(b[1]-a[1])
        out.append((x,y))
    return out

class TargetPlanner(Node):
    def __init__(self):
        super().__init__('pathplanning')
        self.declare_parameter('map_yaml', 'maps/my_map.yaml')
        self.declare_parameter('targets', [[0.0,-5.0],[-9.0,6.0],[5.0,1.0],[-2.0,7.0],[8.0,-3.0],[-7.0,1.0]])
        self.declare_parameter('spacing_m', 0.75)
        self.declare_parameter('clearance_m', 0.4)
        self.declare_parameter('publish_path', True)

        map_yaml = self.get_parameter('map_yaml').get_parameter_value().string_value
        spacing = float(self.get_parameter('spacing_m').value)
        clearance = float(self.get_parameter('clearance_m').value)

        # Load map
        meta = yaml.safe_load(open(map_yaml,'r'))
        img = cv2.imread(str((__import__('pathlib').Path(map_yaml).parent / meta['image'])), cv2.IMREAD_UNCHANGED)
        if meta.get('negate',0)==1: img = 255 - img
        # Build free map and erode for clearance
        free = (img > 250).astype(np.uint8) * 255
        px_clear = max(0, int(clearance / meta['resolution']))
        if px_clear>0:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*px_clear+1, 2*px_clear+1))
            free = cv2.erode(free, k)

        targets = [tuple(map(float,t)) for t in self.get_parameter('targets').value]
        # Plan pairwise
        full_world=[]
        for i in range(len(targets)-1):
            sx,sy = targets[i]; gx,gy = targets[i+1]
            s_px = world_to_px(meta, sx, sy)
            g_px = world_to_px(meta, gx, gy)
            path_px = astar(free, s_px, g_px)
            if not path_px:
                self.get_logger().warn(f'No path from {targets[i]} to {targets[i+1]}')
                continue
            path_px = shortcut_path(free, path_px)
            path_world = [px_to_world(meta, *p) for p in path_px]
            path_world = respace(path_world, spacing)
            # stitch (avoid duplicate)
            if full_world and path_world:
                if math.hypot(full_world[-1][0]-path_world[0][0], full_world[-1][1]-path_world[0][1])<1e-6:
                    full_world.extend(path_world[1:])
                else:
                    full_world.extend(path_world)
            else:
                full_world.extend(path_world)

        # Compute yaws
        wps=[]
        for i,(x,y) in enumerate(full_world):
            if i < len(full_world)-1:
                nx,ny = full_world[i+1]
                yaw = math.atan2(ny-y, nx-x)
            else:
                yaw = 0.0
            wps.append((x,y,yaw))

        # Print to terminal
        print('\n# Waypoints (map frame): x y yaw_rad')
        for i,(x,y,yaw) in enumerate(wps):
            print(f'{i:03d}: {x:.3f} {y:.3f} {yaw:.3f}')
        print(f'# Total waypoints: {len(wps)}\n')

        # Optional RViz preview
        if bool(self.get_parameter('publish_path').value) and wps:
            self.path_pub = self.create_publisher(Path, '/planned_path', 10)
            path = Path(); path.header.frame_id = 'map'
            for x,y,yaw in wps:
                p = PoseStamped(); p.header.frame_id='map'
                p.pose.position.x = x; p.pose.position.y = y
                p.pose.orientation.z = math.sin(yaw/2.0); p.pose.orientation.w = math.cos(yaw/2.0)
                path.poses.append(p)
            self.path_pub.publish(path)
            self.get_logger().info(f'Published /planned_path with {len(path.poses)} poses.')
        rclpy.shutdown()

def main():
    rclpy.init()
    TargetPlanner()
    rclpy.spin(rclpy.node.Node('dummy'))  # immediately shutdown in __init__

if __name__ == '__main__':
    main()
