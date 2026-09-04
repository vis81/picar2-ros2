#!/usr/bin/env python3
"""Republish cartographer's /map as a strictly trinary grid for explore_lite.

explore_lite's frontier search is a *descending* BFS
(frontier_search.cpp:70, `map_[nbr] <= map_[idx]`): starting from a
FREE_SPACE cell it can only ever step to other cells of cost 0. So it explores
the connected component of zero-cost space containing the robot, and a
frontier is only recognised when an unknown cell touches a cell of cost
exactly 0 (frontier_search.cpp:184).

Two things break that, both measured on the robot:

* Nav2's global costmap inflates (radius 0.40, scaling 10), which shatters
  zero-cost space into islands. The robot's island was 191 cells of 21453
  free (0.9%) and touched 0 of 901 frontier cells, so explore_lite reported
  "No frontiers found" and stopped permanently with the room half mapped.
* Raw /map is no better: cartographer publishes graded probabilities, so the
  cells bordering unknown space are small non-zero values, never exactly 0.
  Counted directly on a live map: 0 frontier cells under the ==0 rule.

Collapsing to {-1 unknown, 0 free, 100 occupied} fixes both. Nav2 keeps its
own inflated costmap for planning — safety stays Nav2's job; this grid only
answers "where is the unknown space".
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from nav_msgs.msg import OccupancyGrid


class TrinaryMap(Node):
    def __init__(self):
        super().__init__("explore_map_trinary")
        # Matches map_server's occupied_thresh rather than trusting cartographer
        # cells to reach a full 100.
        self.declare_parameter("lethal_threshold", 65)
        self.declare_parameter("in_topic", "/map")
        self.declare_parameter("out_topic", "/explore_costmap/costmap")
        self.thr = self.get_parameter("lethal_threshold").value

        # Transient-local so explore_lite gets a grid immediately on start
        # rather than waiting for cartographer's next publish; its own
        # subscription is volatile, which is compatible.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(
            OccupancyGrid, self.get_parameter("out_topic").value, qos)
        self.create_subscription(
            OccupancyGrid, self.get_parameter("in_topic").value, self.on_map, qos)
        self._logged = False

    def on_map(self, m):
        thr = self.thr
        out = OccupancyGrid()
        out.header = m.header
        out.info = m.info
        out.data = [(-1 if v < 0 else (100 if v >= thr else 0)) for v in m.data]
        self.pub.publish(out)
        if not self._logged:
            self._logged = True
            free = sum(1 for v in out.data if v == 0)
            self.get_logger().info(
                f"republishing {m.info.width}x{m.info.height} as trinary; "
                f"{free} free cells, threshold {thr}")


def main():
    rclpy.init()
    rclpy.spin(TrinaryMap())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
