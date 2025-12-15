#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np
import math
import json

class LidarDistanceAnalyzer(Node):
    def __init__(self):
        super().__init__('lidar_distance_analyzer')
        self.sub = self.create_subscription(LaserScan, '/scan_some', self.callback, 10)
        # self.pub = self.create_publisher(String, '/lidar_minmax', 10)
        self.get_logger().info("Lidar Distance Analyzer started")
        self.angles=[]
        self.ranges=[]
        self.angle_dict={}
        self.angle_keys = []

    def callback(self, msg: LaserScan):
        # Convert LaserScan data to numpy arrays
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Filter out invalid readings (inf or NaN)
        valid_mask = np.isfinite(ranges)
        
        ranges = ranges[valid_mask]
        angles = angles[valid_mask]

        positive_indices = np.where(ranges > 0)[0]
        ranges = ranges[positive_indices]
        angles = angles[positive_indices]

        if len(ranges) == 0:
            self.get_logger().warn("No valid LIDAR data!")
            return
        # min_angle_from_zero = angles[np.argmin(np.abs(angles))]

        self.angles=angles
        self.ranges=ranges
        # Get distance at 0 degrees if available

        self.angle_dict = {}
        for i,angle in enumerate(angles):
            angle_str = str(round(math.degrees(angle), 2))
            self.angle_dict[angle_str] = float(ranges[i])

        self.angle_keys = list(self.angle_dict.keys())
        json_path = "lidar_data.json"
        with open(json_path, "w") as f:
            json.dump(self.angle_dict, f, indent=4)
        
        # angle_json = json.dumps(self.angle_dict)
        # with open("lidar_data.json", "w") as f:
        #     f.write(angle_json)

        # zero_angle_indices = np.where(angles==min_angle_from_zero)[0]
        # if len(zero_angle_indices) > 0:
        #     zero_angle_index = zero_angle_indices[0]
        #     self.get_logger().info(f"Distance at 0 degrees: {ranges[zero_angle_index]:.2f} m")



        # # Find minimum and maximum distances
        # min_idx = np.argmin(ranges)
        # max_idx = np.argmax(ranges)
        # min_dist = ranges[min_idx]
        # max_dist = ranges[max_idx]
        # min_angle = math.degrees(angles[min_idx])
        # max_angle = math.degrees(angles[max_idx])

        # # Log info
        # # self.get_logger().info(
        # #     f"Min: {min_dist:.2f} m @ {min_angle:.2f}°, "
        # #     f"Max: {max_dist:.2f} m @ {max_angle:.2f}°"
        # # )

        # # Publish the result as a simple string (for visualization/testing)
        # msg_out = String()
        # msg_out.data = f"Min: {min_dist:.2f} m @ {min_angle:.2f}°, Max: {max_dist:.2f} m @ {max_angle:.2f}°"
        # self.pub.publish(msg_out)
    
    def get_distance_at_angle(self, target_angle_deg):
        if len(self.angle_keys)==0:
            self.get_logger().warn("No LIDAR data available!")
            return None
        
        
        closest_few_angles = sorted(self.angle_keys, key=lambda x: abs(x - target_angle_deg))[:40]
        min_distance = 50
        min_angle = None
        for angle in closest_few_angles:
            distance = self.angle_dict[angle]
            if distance < min_distance:
                min_distance = distance
                min_angle = angle
        # distance_of_10 = [self.angle_dict[angle] for angle in closest_10_angles]
        # distance = min(distance_of_10)
        # distance = self.angle_dict[closest_angle]
        self.get_logger().info(f"Distance at given angle: {min_distance:.2f} m")
        return min_distance, min_angle

        # target_angle_rad = math.radians(target_angle_deg)
        # Find the index of the angle closest to the target angle

def main(args=None):
    rclpy.init(args=args)
    node = LidarDistanceAnalyzer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
