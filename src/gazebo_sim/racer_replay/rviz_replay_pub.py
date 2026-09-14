#!/usr/bin/env python3
"""Publish the SAVED 600s RACER-replay results as latched ROS2 topics for RViz.
No sim re-run: loads output_600s_v2/{map_botN.npy, rec_botN.csv, origins.json} +
the ground-truth occupancy, transforms each drone's accumulated SLAM cloud from
its LIO frame into the world, and publishes (transient_local so late-joining RViz
windows still get them):
  /env/obstacles      PointCloud2  — true environment (grey)
  /botN/slam_map      PointCloud2  — that drone's accumulated SLAM cloud (height)
  /botN/gt_path       Path         — ground-truth flight
  /botN/slam_path     Path         — SLAM-estimated flight
All in frame 'map' (a static identity TF map->viz is broadcast so RViz resolves it).
"""
import csv, json, math, os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster

DATA = os.environ.get("REPLAY_DIR", "/data/output_600s_v2")   # override to view other runs
OCC = "/racer/outputs/run_20260718_113818_726454/ground_truth_occupancy.npy"
BOTS = ["bot1", "bot2", "bot3", "bot4", "bot5", "bot6"]
MAP_CAP = 160000     # per-drone cloud cap for RViz responsiveness
PATH_STRIDE = 3


def latched():
    q = QoSProfile(depth=1)
    q.durability = DurabilityPolicy.TRANSIENT_LOCAL
    q.history = HistoryPolicy.KEEP_LAST
    return q


def to_world(m, org):
    """LIO-frame points -> world, matching racer_replay.slam_world (yaw + trans)."""
    ox, oy, oz, oyaw = org
    c, s = math.cos(oyaw), math.sin(oyaw)
    x = ox + c * m[:, 0] - s * m[:, 1]
    y = oy + s * m[:, 0] + c * m[:, 1]
    z = oz + m[:, 2]
    return np.column_stack([x, y, z]).astype(np.float32)


def cloud_msg(pts, frame, stamp):
    """PointCloud2 with x,y,z,intensity(=height) built straight from numpy."""
    n = len(pts)
    arr = np.zeros(n, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('intensity', 'f4')])
    arr['x'], arr['y'], arr['z'] = pts[:, 0], pts[:, 1], pts[:, 2]
    arr['intensity'] = pts[:, 2]
    m = PointCloud2()
    m.header.frame_id = frame
    m.header.stamp = stamp
    m.height, m.width = 1, n
    m.fields = [PointField(name=nm, offset=o, datatype=PointField.FLOAT32, count=1)
                for nm, o in (('x', 0), ('y', 4), ('z', 8), ('intensity', 12))]
    m.is_bigendian = False
    m.point_step = 16
    m.row_step = 16 * n
    m.data = arr.tobytes()
    m.is_dense = True
    return m


def path_msg(xyz, frame, stamp):
    p = Path()
    p.header.frame_id = frame
    p.header.stamp = stamp
    for r in xyz:
        ps = PoseStamped()
        ps.header.frame_id = frame
        ps.header.stamp = stamp
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = \
            float(r[0]), float(r[1]), float(r[2])
        ps.pose.orientation.w = 1.0
        p.poses.append(ps)
    return p


def load_rec(b):
    g, s = [], []
    with open(os.path.join(DATA, "rec_%s.csv" % b)) as f:
        for d in csv.DictReader(f):
            g.append([float(d["gt_x"]), float(d["gt_y"]), float(d["gt_z"])])
            sv = [float(d["sl_x"]), float(d["sl_y"]), float(d["sl_z"])]
            if not any(math.isnan(v) for v in sv):
                s.append(sv)
    return np.array(g)[::PATH_STRIDE], (np.array(s)[::PATH_STRIDE] if s else np.zeros((0, 3)))


class ReplayPub(Node):
    def __init__(self):
        super().__init__("rviz_replay_pub")
        q = latched()
        stamp = self.get_clock().now().to_msg()

        # static identity TF so 'map' resolves in RViz
        self.tfb = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "map"
        t.child_frame_id = "viz"
        t.transform.rotation.w = 1.0
        self.tfb.sendTransform(t)

        # environment (ground-truth occupancy voxel centres)
        occ = np.load(OCC) == 1
        ox, oy, oz = np.where(occ)
        env = (np.column_stack([ox, oy, oz]) + 0.5).astype(np.float32)
        self.env_pub = self.create_publisher(PointCloud2, "/env/obstacles", q)
        self.env_msg = cloud_msg(env, "map", stamp)

        org = json.load(open(os.path.join(DATA, "origins.json")))
        self.pubs = []
        for b in BOTS:
            m = np.load(os.path.join(DATA, "map_%s.npy" % b)).astype(np.float32)
            if len(m) > MAP_CAP:
                m = m[:: len(m) // MAP_CAP + 1]
            w = to_world(m, org[b])
            w = w[((w >= -3) & (w <= 53)).all(1)]     # drop stray far points
            gt, sl = load_rec(b)
            cp = self.create_publisher(PointCloud2, "/%s/slam_map" % b, q)
            gp = self.create_publisher(Path, "/%s/gt_path" % b, q)
            sp = self.create_publisher(Path, "/%s/slam_path" % b, q)
            self.pubs.append((b, cp, cloud_msg(w, "map", stamp),
                              gp, path_msg(gt, "map", stamp),
                              sp, path_msg(sl, "map", stamp), len(w)))

        self.publish_all()
        self.timer = self.create_timer(3.0, self.publish_all)   # re-latch for late RViz
        self.n = 0

    def publish_all(self):
        self.env_pub.publish(self.env_msg)
        for b, cp, cm, gp, gm, sp, sm, npts in self.pubs:
            cp.publish(cm); gp.publish(gm); sp.publish(sm)
        self.n = getattr(self, "n", 0) + 1
        if self.n <= 2:
            tot = sum(p[7] for p in self.pubs)
            self.get_logger().info("published env(%d) + 6 drone maps (%d pts) + paths"
                                   % (self.env_msg.width, tot))


def main():
    rclpy.init()
    n = ReplayPub()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.ok() and rclpy.shutdown()


if __name__ == "__main__":
    main()
