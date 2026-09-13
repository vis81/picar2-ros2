"""How good is the odometry? Measure it against the map.

    bench-odom BAG --map maps/F1b.yaml [--window 1 3 10] [--every 1.0]
    bench-odom --live 60 --map maps/F1b.yaml      # watch the robot for 60 s

Every --every seconds a lidar scan is matched against the static map by
exhaustive search around the localized pose (translation +-0.4 m, rotation
+-8 deg, then a fine pass). That match is the reference. From it:

  * odometry drift - how the EKF's odom->base motion over a window differs
    from the reference motion, split into yaw, along-track and cross-track,
    and reported separately for straights and corners. This is the number
    that decides how far AMCL has to be trusted between scans.
  * gyro drift - the raw IMU yaw rate integrated over the same windows, to
    tell a wheel/steering problem from a gyro problem.
  * localization error - where AMCL (map->base) sits relative to the
    reference, laterally and in yaw, with its own reported sigma.

Needs /lidar_node/scan, /tf, /tf_static, /odom, /imu/data in the bag (the
web UI's recorder includes them all). --live records the same topics from
the running robot for the given number of seconds and evaluates them.

The reference is only as good as the map: a spot where the map is wrong
shows up as a low "fit" for that scan, so the fit column is printed and
scans under --min-fit are dropped from the statistics.
"""

import argparse
import bisect
import math
import sys
import time

import numpy as np
import yaml
from pathlib import Path
from scipy.spatial import cKDTree

from .map_to_scenario import read_pgm


# ── inputs ───────────────────────────────────────────────────────────────

TOPICS = ['/lidar_node/scan', '/tf', '/tf_static', '/odom', '/imu/data']


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=''),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    missing = [t for t in TOPICS if t not in types]
    if missing:
        sys.exit(f'bag lacks {missing}')
    r.set_filter(rosbag2_py.StorageFilter(topics=TOPICS))
    out = {t: [] for t in TOPICS}
    while r.has_next():
        tp, data, ts = r.read_next()
        out[tp].append((ts / 1e9, deserialize_message(data, get_message(types[tp]))))
    return out


def record_live(seconds):
    """Subscribe to the robot for a while and hand back the same structure
    read_bag() returns, so one evaluator serves both."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import LaserScan, Imu
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage
    rclpy.init()
    n = Node('bench_odom')
    out = {t: [] for t in TOPICS}
    best = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
    latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def keep(topic):
        return lambda m: out[topic].append((time.time(), m))
    n.create_subscription(LaserScan, '/lidar_node/scan', keep('/lidar_node/scan'), best)
    n.create_subscription(Imu, '/imu/data', keep('/imu/data'), best)
    n.create_subscription(Odometry, '/odom', keep('/odom'), 50)
    n.create_subscription(TFMessage, '/tf', keep('/tf'), 100)
    n.create_subscription(TFMessage, '/tf_static', keep('/tf_static'), latched)
    t0 = time.time()
    last = -1
    while time.time() - t0 < seconds:
        rclpy.spin_once(n, timeout_sec=0.1)
        left = int(seconds - (time.time() - t0))
        if left != last and left % 10 == 0:
            print(f'  recording... {left} s left, {len(out["/lidar_node/scan"])} scans',
                  file=sys.stderr)
            last = left
    rclpy.try_shutdown()
    return out


# ── geometry ─────────────────────────────────────────────────────────────

def load_map(yaml_path):
    info = yaml.safe_load(Path(yaml_path).read_text())
    img = read_pgm(Path(yaml_path).parent / info['image'])
    h, w = img.shape
    res = info['resolution']
    ox, oy = info['origin'][:2]
    ys, xs = np.nonzero(img <= 1)
    return cKDTree(np.column_stack([ox + (xs + 0.5) * res, oy + (h - 1 - ys + 0.5) * res]))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Frames:
    """map->odom and odom->base_footprint over time, plus the static
    base_footprint->laser chain, from /tf and /tf_static."""

    def __init__(self, tf_msgs, static_msgs, laser_frame='laser_link'):
        mo, ob = [], []
        for _, m in tf_msgs:
            for tr in m.transforms:
                st = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                row = (st, tr.transform.translation.x, tr.transform.translation.y,
                       yaw_of(tr.transform.rotation))
                if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                    mo.append(row)
                elif tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_footprint':
                    ob.append(row)
        self.mo = np.array(mo)
        self.ob = np.array(ob)
        if not len(self.ob):
            sys.exit('no odom->base_footprint in /tf')
        self.has_map = len(self.mo) > 0
        static = {}
        for _, m in static_msgs:
            for tr in m.transforms:
                static[(tr.header.frame_id, tr.child_frame_id)] = (
                    tr.transform.translation.x, tr.transform.translation.y,
                    yaw_of(tr.transform.rotation))
        # base_footprint -> laser: compose the static chain
        x = y = th = 0.0
        for a, b in (('base_footprint', 'base_link'), ('base_link', laser_frame)):
            if (a, b) not in static:
                sys.exit(f'no static transform {a}->{b}')
            tx, ty, tth = static[(a, b)]
            x += tx * math.cos(th) - ty * math.sin(th)
            y += tx * math.sin(th) + ty * math.cos(th)
            th += tth
        self.laser = (x, y, th)

    @staticmethod
    def _at(arr, t):
        i = max(bisect.bisect(arr[:, 0], t) - 1, 0)
        return arr[i, 1:]

    def odom_pose(self, t):
        return tuple(self._at(self.ob, t))

    def map_pose(self, t):
        ox, oy, oth = self._at(self.ob, t)
        if not self.has_map:
            return ox, oy, oth
        mx, my, mth = self._at(self.mo, t)
        return (mx + ox * math.cos(mth) - oy * math.sin(mth),
                my + ox * math.sin(mth) + oy * math.cos(mth), oth + mth)


def scan_in_base(scan, laser, subsample=2, max_range=4.0):
    ang = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
    r = np.asarray(scan.ranges, dtype=float)
    ok = np.isfinite(r) & (r > 0.05) & (r < max_range)
    ang, r = ang[ok][::subsample], r[ok][::subsample]
    lx, ly, lth = laser
    return (r * np.cos(ang + lth) + lx, r * np.sin(ang + lth) + ly)


def place(px, py, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack([x + px * c - py * s, y + px * s + py * c])


def match(tree, px, py, x0, y0, yaw0):
    """Best (fit, dx, dy, dyaw) around the given pose: coarse 5 cm / 2 deg,
    then fine 2 cm / 0.5 deg. fit = fraction of beams within 4 cm of a wall."""
    best = (-1.0, 0.0, 0.0, 0.0)
    for dth in np.radians(np.arange(-8, 8.1, 2)):
        for dx in np.arange(-0.4, 0.41, 0.05):
            for dy in np.arange(-0.4, 0.41, 0.05):
                d, _ = tree.query(place(px, py, x0 + dx, y0 + dy, yaw0 + dth))
                sc = float(np.mean(d < 0.07))
                if sc > best[0]:
                    best = (sc, dx, dy, dth)
    _, dx, dy, dth = best
    fine = (-1.0, dx, dy, dth)
    for ddth in np.radians(np.arange(-2, 2.1, 0.5)):
        for ddx in np.arange(-0.04, 0.041, 0.02):
            for ddy in np.arange(-0.04, 0.041, 0.02):
                d, _ = tree.query(place(px, py, x0 + dx + ddx, y0 + dy + ddy, yaw0 + dth + ddth))
                sc = float(np.mean(d < 0.04))
                if sc > fine[0]:
                    fine = (sc, dx + ddx, dy + ddy, dth + ddth)
    return fine


# ── evaluation ───────────────────────────────────────────────────────────

def rel_motion(p0, p1):
    """Motion from pose p0 to p1 expressed in p0's frame: along, cross, dyaw."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    c, s = math.cos(-p0[2]), math.sin(-p0[2])
    return dx * c - dy * s, dx * s + dy * c, wrap(p1[2] - p0[2])


def evaluate(data, tree, every=1.0, windows=(1, 3, 10), min_fit=0.5,
             turn_deg=40.0, straight_deg=15.0, laser_frame='laser_link'):
    frames = Frames(data['/tf'], data['/tf_static'], laser_frame)
    scans = data['/lidar_node/scan']
    stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, m in scans]
    imu = np.array([(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.angular_velocity.z)
                    for _, m in data['/imu/data']])
    odom_v = np.array([(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                        m.twist.twist.linear.x) for _, m in data['/odom']])

    # Sample scans at the requested cadence and match each one.
    rows = []       # t, ref pose (map), odom pose, fit, amcl pose (map)
    t = stamps[0] + 1.0
    n_total = 0
    while t < stamps[-1] - 0.5:
        i = max(bisect.bisect(stamps, t) - 1, 0)
        ts = stamps[i]
        loc = frames.map_pose(ts)
        px, py = scan_in_base(scans[i][1], frames.laser)
        if len(px) > 20:
            fit, dx, dy, dth = match(tree, px, py, *loc)
            n_total += 1
            if fit >= min_fit:
                rows.append((ts, (loc[0] + dx, loc[1] + dy, loc[2] + dth),
                             frames.odom_pose(ts), fit, loc))
        t += every
        print(f'\r  matched {n_total} scans', end='', file=sys.stderr)
    print(file=sys.stderr)
    if len(rows) < 5:
        sys.exit(f'only {len(rows)} usable scans (fit >= {min_fit}); is the map right?')

    T = np.array([r[0] for r in rows])
    REF = [r[1] for r in rows]
    ODO = [r[2] for r in rows]
    FIT = np.array([r[3] for r in rows])
    LOC = [r[4] for r in rows]

    def integ(sig, a, b):
        s = sig[(sig[:, 0] >= a) & (sig[:, 0] <= b)]
        return float(np.trapz(s[:, 1], s[:, 0])) if len(s) > 1 else 0.0

    report = {'n': len(rows), 'dropped': n_total - len(rows), 'fit_median': float(np.median(FIT)),
              'duration': float(T[-1] - T[0]), 'windows': {}}
    for W in windows:
        recs = []
        for a in range(len(rows)):
            b = bisect.bisect_left(T, T[a] + W - 0.25)
            if b >= len(rows) or T[b] - T[a] > W + 0.75:
                continue
            ra, rc, rth = rel_motion(REF[a], REF[b])
            oa, oc, oth = rel_motion(ODO[a], ODO[b])
            g = integ(imu, T[a], T[b])
            recs.append((math.degrees(rth), math.degrees(oth - rth), math.degrees(g - rth),
                         oa - ra, oc - rc, math.hypot(ra, rc)))
        R = np.array(recs)
        if not len(R):
            continue
        turn = np.abs(R[:, 0]) > turn_deg
        straight = np.abs(R[:, 0]) < straight_deg
        w = {'n': len(R)}
        for name, sel in (('all', np.ones(len(R), bool)), ('straight', straight), ('turning', turn)):
            r = R[sel]
            if len(r) < 3:
                continue
            w[name] = {
                'n': int(len(r)),
                'ekf_yaw_abs': float(np.median(np.abs(r[:, 1]))),
                'ekf_yaw_bias': float(np.median(r[:, 1])),
                'gyro_yaw_abs': float(np.median(np.abs(r[:, 2]))),
                'gyro_yaw_bias': float(np.median(r[:, 2])),
                'along_cm': float(np.median(np.abs(r[:, 3])) * 100),
                'cross_cm': float(np.median(np.abs(r[:, 4])) * 100),
                'dist_m': float(np.median(r[:, 5])),
            }
            if name == 'turning':
                ratio = r[:, 0] + r[:, 1]
                w[name]['ekf_turn_ratio'] = float(np.median(ratio / r[:, 0]))
                w[name]['gyro_turn_ratio'] = float(np.median((r[:, 0] + r[:, 2]) / r[:, 0]))
        report['windows'][W] = w

    # Localization: AMCL pose vs reference, in the robot frame.
    lat, along, dyaw = [], [], []
    for ref, loc in zip(REF, LOC):
        a, c, th = rel_motion(loc, ref)
        along.append(a)
        lat.append(c)
        dyaw.append(math.degrees(th))
    lat, along, dyaw = np.array(lat), np.array(along), np.array(dyaw)
    report['loc'] = {
        'lateral_abs_cm': float(np.median(np.abs(lat)) * 100),
        'lateral_signed_cm': float(np.median(lat) * 100),
        'lateral_p90_cm': float(np.percentile(np.abs(lat), 90) * 100),
        'along_abs_cm': float(np.median(np.abs(along)) * 100),
        'yaw_abs_deg': float(np.median(np.abs(dyaw))),
        'yaw_signed_deg': float(np.median(dyaw)),
        'has_map_frame': frames.has_map,
    }
    # Whole-run accumulated yaw drift of the odometry.
    report['run_yaw_drift_deg'] = math.degrees(wrap((ODO[-1][2] - ODO[0][2]) - (REF[-1][2] - REF[0][2])))
    return report


def print_report(rep):
    print(f"\n{rep['n']} reference scans over {rep['duration']:.0f} s "
          f"(fit median {rep['fit_median']*100:.0f}%, {rep['dropped']} dropped for poor fit)")
    print('\nODOMETRY DRIFT per window  (median |error| ; bias in brackets)')
    print('  window   segment     n   EKF yaw        gyro yaw       along   cross   over')
    for W, w in rep['windows'].items():
        for name in ('all', 'straight', 'turning'):
            if name not in w:
                continue
            s = w[name]
            extra = ''
            if name == 'turning':
                extra = f"   turn ratio EKF {s['ekf_turn_ratio']:.3f}  gyro {s['gyro_turn_ratio']:.3f}"
            print(f"  {W:>4} s   {name:9} {s['n']:3}   "
                  f"{s['ekf_yaw_abs']:4.1f}° ({s['ekf_yaw_bias']:+.1f})   "
                  f"{s['gyro_yaw_abs']:4.1f}° ({s['gyro_yaw_bias']:+.1f})   "
                  f"{s['along_cm']:4.0f} cm  {s['cross_cm']:4.0f} cm  {s['dist_m']:.1f} m{extra}")
    print(f"\n  accumulated odometry yaw drift over the run: {rep['run_yaw_drift_deg']:+.0f}°")
    l = rep['loc']
    if l['has_map_frame']:
        print('\nLOCALIZATION (AMCL vs scan match, robot frame)')
        print(f"  lateral  median {l['lateral_abs_cm']:.0f} cm  (signed {l['lateral_signed_cm']:+.0f}, p90 {l['lateral_p90_cm']:.0f})")
        print(f"  along    median {l['along_abs_cm']:.0f} cm")
        print(f"  yaw      median {l['yaw_abs_deg']:.1f}°  (signed {l['yaw_signed_deg']:+.1f})")
    else:
        print('\n(no map->odom in the data: localization section skipped)')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', nargs='?', help='rosbag2 directory')
    ap.add_argument('--live', type=float, metavar='SECONDS', help='record from the robot instead')
    ap.add_argument('--map', required=True, help='map yaml (the one AMCL runs on)')
    ap.add_argument('--every', type=float, default=1.0, help='seconds between reference scans')
    ap.add_argument('--window', type=float, nargs='+', default=[1, 3, 10])
    ap.add_argument('--min-fit', type=float, default=0.5)
    ap.add_argument('--laser-frame', default='laser_link')
    ap.add_argument('--json', help='also write the report here')
    a = ap.parse_args()
    if not a.bag and not a.live:
        ap.error('give a bag or --live')
    tree = load_map(a.map)
    data = record_live(a.live) if a.live else read_bag(a.bag)
    rep = evaluate(data, tree, every=a.every, windows=tuple(a.window),
                   min_fit=a.min_fit, laser_frame=a.laser_frame)
    print_report(rep)
    if a.json:
        import json
        Path(a.json).write_text(json.dumps(rep, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
