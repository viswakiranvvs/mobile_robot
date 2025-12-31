import cv2
import numpy as np
import torch
import sys
sys.path.append("/home/robot2/Documents/isaac_sim/mobile_robot/src/depth_package/Depth-Anything-V2")
# from app import DEVICE
from depth_anything_v2.dpt import DepthAnythingV2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class RGB_TODepthNode(Node):
    def __init__(self):
        super().__init__('rgb_to_depth_node')
        self.get_logger().info('RGB to Depth Node has been started.')
        DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
        model_configs = {
            'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
            'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
            'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
            'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
        }

        encoder = 'vits' # or 'vits', 'vitb', 'vitg'

        model = DepthAnythingV2(**model_configs[encoder])
        model.load_state_dict(torch.load(f'/home/robot2/Documents/isaac_sim/mobile_robot/src/depth_package/Depth-Anything-V2/checkpoints/depth_anything_v2_{encoder}.pth', map_location='cpu'))
        self.model = model.to(DEVICE).eval()
        self.bridge = CvBridge()
        self.rgb_subscriber = self.create_subscription(
            Image,
            '/camera_rgb',
            self.rgb_callback,
            10
        )
        self.actual_depth_subscriber = self.create_subscription(
            Image,
            '/camera_depth',
            self.depth_callback,
            10
        )
        self.depth_pub =self.create_publisher(Image, '/custom_depth', 10)

    def depth_callback(self, msg):
        self.get_logger().info('Received actual depth image.')
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        self.get_logger().info(f'Actual depth image shape: {depth_image.shape}')
        
    def rgb_callback(self, msg):
        self.get_logger().info('Received RGB image for depth estimation.')
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        self.get_logger().info(f'RGB image shape: {cv_image.shape}')

        # raw_img = cv2.imread('/home/robot2/Downloads/mobilebot_image.jpg')
        depth = self.model.infer_image(rgb_image) # HxW raw depth map in numpy
        depth_map_normalized = cv2.normalize(depth*1000, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        cv2.imwrite('/home/robot2/Documents/isaac_sim/mobile_robot/src/depth_package/recent_est.png',depth*100)
        depth_img = cv2.imread('/home/robot2/Documents/isaac_sim/mobile_robot/src/depth_package/recent_est.png',cv2.IMREAD_UNCHANGED)
        # cv2.imshow('Depth Map', depth)
        # depth_image_msg = self.bridge.cv2_to_imgmsg(depth_map_normalized.astype(np.float32),encoding="32FC1")
        depth_image_msg = self.bridge.cv2_to_imgmsg(depth_img)

        # depth_image_msg = self.bridge.cv2_to_imgmsg(rgb_image,encoding="rgb8")
        depth_image_msg.header.stamp = self.get_clock().now().to_msg()
        depth_image_msg.header.frame_id = "camera_depth_estimation"
        self.depth_pub.publish(depth_image_msg)
        # cv2.imwrite('/home/robot2/Downloads/mobilebot_depth.png', depth*100) # save depth in mm

def main():
    rclpy.init()
    cam_node = RGB_TODepthNode()

    try:
        rclpy.spin(cam_node)
    except KeyboardInterrupt:
        pass
    finally:
        cam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()