from io import BytesIO
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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
        self.threshold = 0.20  # meters
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.depth_map_pb = self.create_publisher(Image, '/depth_map', 10)

    def depth_callback(self, msg):
        global cnt, start
        if not cnt:
            start = time.time_ns()

        # Process the depth image message here
        # self.get_logger().info(f"Received depth image with width: {msg.width}, height: {msg.height}")
        # depth_array = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').tolist()
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_array = np.array(cv_image, dtype=np.float32)
        fig, ax = plt.subplots()
        ax.imshow(depth_array)

        # Add bounding box
        rect = patches.Rectangle((270, 215), 100, 100,
                                linewidth=2, edgecolor='r', facecolor='none')
        ax.add_patch(rect)

        # Save the figure (not just array)
        plt.savefig(buf, format='png', bbox_inches='tight')
        plt.close(fig)

        buf.seek(0)
        
        # plt.imshow(depth_array)
        # ax = plt.gca()
        # rect = patches.Rectangle((270, 215), 100, 100, linewidth=2, edgecolor='r', facecolor='none')
        # ax.add_patch(rect)
        
        # plt.savefig(buf, format='png', bbox_inches='tight')
        # plt.close(fig)
        # buf.seek(0)

        # Convert to uint8
        img_np = plt.imread(buf,format='png')
        if img_np.dtype in [np.float32, np.float64]:
            img_np = (img_np * 255).astype(np.uint8)

        # Determine encoding
        if len(img_np.shape) == 2:
            encoding = "mono8"
        elif img_np.shape[2] == 3:
            encoding = "rgb8"
        elif img_np.shape[2] == 4:
            encoding = "rgba8"
        depth_image_msg = self.bridge.cv2_to_imgmsg(img_np, encoding=encoding)

        depth_image_msg.header.stamp = self.get_clock().now().to_msg()
        depth_image_msg.header.frame_id = "camera_depth_frame"
        self.depth_map_pb.publish(depth_image_msg)
        buf.truncate(0)
        buf.seek(0)


        # plt.pause(0.000000000000000000000000000000000000001)  # Display for 0.5 seconds
        # plt.close()
        # plt.show()
        # delay = 0.5  # seconds
        # self.get_clock().sleep_for(rclpy.duration.Duration(seconds=delay))
        # plt.close()

        # Find the minimum depth in the frame (ignoring zeros / NaNs)
        # valid_depths = depth_array[np.isfinite(depth_array)]
        # valid_depths = valid_depths[valid_depths > 0]

        # if valid_depths.size > 0:
        #     min_dist = np.min(valid_depths)
        #     self.get_logger().info(f"Closest object: {min_dist:.3f} m")

        #     # Stop if obstacle within threshold
        #     if min_dist < self.threshold:
        #         self.get_logger().warn("Obstacle detected")
        #         self.take_u_turn()
        #         delay = 6  # seconds
        #         self.get_clock().sleep_for(rclpy.duration.Duration(seconds=delay))
        #         self.stop_robot()
        cnt += 1
        del depth_array
        print(cnt / ((time.time_ns() - start) / 1e9))


    def rgb_callback(self, msg):
        pass

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
        self.get_logger().warn("Took U-Turn!!.")

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
