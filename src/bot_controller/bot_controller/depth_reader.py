from io import BytesIO
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PIL import Image as PILImage
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import matplotlib.patches as patches

cnt = 0
start = 0
buf = BytesIO()

class depth_reader_node(Node):
    def __init__(self):  #
        super().__init__('depth_reader_node')
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera_depth',
            self.depth_callback,
            0
        )
        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera_rgb',
            self.rgb_callback,
            10
        )
        self.get_logger().info("Depth reader node started")
        self.bridge = CvBridge()
        self.threshold = 0.30  # meters
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.depth_map_pb = self.create_publisher(Image, '/depth_map', 10)

        # Store the latest RGB frame
        self.latest_rgb = None

    def depth_callback(self, msg):
        global cnt, start
        if not cnt:
            start = time.time_ns()

        # Convert depth ROS Image → OpenCV
        cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_array = np.array(cv_depth, dtype=np.float32)

        # Normalize depth to 8-bit
        img_vis = cv2.normalize(depth_array, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        # Convert depth to BGR for overlay
        depth_bgr = cv2.cvtColor(img_vis, cv2.COLOR_GRAY2BGR)

        # Check if we have a recent RGB frame
        if self.latest_rgb is not None:
            # Convert RGB → grayscale → edges
            gray_rgb = cv2.cvtColor(self.latest_rgb, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray_rgb, 200, 255)

            # Overlay edges on depth visualization (red edges)
            depth_bgr[edges > 0] = [0, 0, 255]

        # Draw bounding box on top
        cv2.rectangle(depth_bgr, (270, 215), (270+100, 215+100), (0, 255, 0), 2)  # green box

        # Publish final image
        depth_image_msg = self.bridge.cv2_to_imgmsg(depth_bgr, encoding="bgr8")
        depth_image_msg.header.stamp = self.get_clock().now().to_msg()
        depth_image_msg.header.frame_id = "camera_depth_frame"
        self.depth_map_pb.publish(depth_image_msg)

        cnt += 1
        print(cnt / ((time.time_ns() - start) / 1e9))

        # Find the minimum depth in the frame (ignoring zeros / NaNs)
        # valid_depths = depth_array[np.isfinite(depth_array)]
        # valid_depths = valid_depths[valid_depths > 0]
        bounding_depth = depth_array[215:315, 270:370]  # Focus on bounding box area
        self.get_logger().info(f"Valid depths count: {bounding_depth.shape}")


        if bounding_depth.size > 0:
            min_dist = np.min(bounding_depth)
            self.get_logger().info(f"Closest object: {min_dist:.3f} m")

            # Stop if obstacle within threshold
            if min_dist < self.threshold:
                self.get_logger().warn("Obstacle detected")
                self.take_u_turn()
                delay = 6  # seconds
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=delay))
                self.stop_robot()


    def rgb_callback(self, msg):
        # Store latest RGB frame as NumPy array
        cv_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_rgb = cv_rgb

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().warn("Robot stopped!")
    
    def take_u_turn(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -1.0
        self.cmd_pub.publish(twist)
        self.get_logger().warn("Taking U-Turn!!.")

def main():
    rclpy.init()
    node = depth_reader_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
