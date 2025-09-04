#!/usr/bin/env python3
# save_rtabmap_map.py
# Usage: python3 save_rtabmap_map.py
# Requires: ROS2 (rclpy). Run while rtabmap is up and publishing the service.

import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetMap
import struct
import math
import sys
import os

OUTPUT_PGM = "my_map.pgm"
OUTPUT_YAML = "my_map.yaml"

def occupancy_to_byte(v):
    # Follow ROS map_saver convention:
    # unknown (-1) -> 205 (gray)
    # occupied 100 -> 0 (black)
    # free 0 -> 255 (white)
    if v == -1:
        return 205
    # clamp
    if v < 0: v = 0
    if v > 100: v = 100
    b = int(255 - (v * 255.0 / 100.0) + 0.5)
    if b < 0: b = 0
    if b > 255: b = 255
    return b

class MapSaver(Node):
    def __init__(self):
        super().__init__('rtabmap_map_saver')
        self.cli = self.create_client(GetMap, '/rtabmap/rtabmap/get_map')
        self.get_logger().info('Waiting for /rtabmap/rtabmap/get_map service...')
        if not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Service not available. Make sure rtabmap is running.")
            rclpy.shutdown()
            sys.exit(1)
        self.req = GetMap.Request()
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.save_map)

    def save_map(self, fut):
        try:
            resp = fut.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))
            rclpy.shutdown()
            return

        grid = resp.map
        w = grid.info.width
        h = grid.info.height
        res = grid.info.resolution
        origin = grid.info.origin
        self.get_logger().info(f"Map: {w} x {h}, res {res}, origin ({origin.position.x}, {origin.position.y})")

        # Write PGM (P5 binary)
        try:
            with open(OUTPUT_PGM, 'wb') as f:
                header = f"P5\n{w} {h}\n255\n"
                f.write(header.encode('ascii'))
                # nav_msgs/OccupancyGrid data is row-major, starting with (0,0)
                # map_saver writes rows from bottom to top; but occupancygrid uses
                # origin and data indexing: map_server/map_saver write data as-is.
                # We'll write the data in the same order (row-major).
                for v in grid.data:
                    f.write(struct.pack('B', occupancy_to_byte(v)))
        except Exception as e:
            self.get_logger().error(f"Failed to write PGM: {e}")
            rclpy.shutdown()
            return

        # Write YAML
        try:
            yaml_text = (
                f"image: {os.path.basename(OUTPUT_PGM)}\n"
                f"resolution: {res}\n"
                f"origin: [{origin.position.x}, {origin.position.y}, {0.0}]\n"
                f"negate: 0\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.196\n"
            )
            with open(OUTPUT_YAML, 'w') as f:
                f.write(yaml_text)
        except Exception as e:
            self.get_logger().error(f"Failed to write YAML: {e}")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Saved {OUTPUT_PGM} and {OUTPUT_YAML}")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
