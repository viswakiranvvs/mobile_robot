import cv2
import numpy as np
import torch
import sys
sys.path.append("/home/robot2/Documents/isaac_sim/mobile_robot/src/depth_package/Depth-Anything-V2/metric_depth")
# from app import DEVICE
from depth_anything_v2.dpt import DepthAnythingV2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer


class RGB_TODepthNode(Node):
    def __init__(self):
        super().__init__('rgb_to_depth_node')
        self.get_logger().info('RGB to Depth Node has been started.')
        DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
        model_configs = {
            'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
            'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
            'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]}
        }


        encoder = 'vitl' # or 'vits', 'vitb'
        dataset = 'hypersim' # 'hypersim' for indoor model, 'vkitti' for outdoor model
        max_depth = 20 # 20 for indoor model, 80 for outdoor model

        model = DepthAnythingV2(**{**model_configs[encoder], 'max_depth': max_depth})
        model.load_state_dict(torch.load(f'/home/robot2/Documents/isaac_sim/mobile_robot/src/depth_package/Depth-Anything-V2/checkpoints/depth_anything_v2_metric_{dataset}_{encoder}.pth', map_location='cpu'))
        self.model = model.to(DEVICE).eval()
        self.bridge = CvBridge()
        # self.rgb_subscriber = self.create_subscription(
        #     Image,
        #     '/camera_rgb',
        #     self.rgb_callback,
        #     10
        # )
        # self.actual_depth_subscriber = self.create_subscription(
        #     Image,
        #     '/camera_depth',
        #     self.depth_callback,
        #     10
        # )
        self.depth_pub =self.create_publisher(Image, '/custom_depth', 10)

        self.rgb_sub = Subscriber(self, Image, '/camera_rgb')
        self.gt_depth_sub = Subscriber(self, Image, '/camera_depth')

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.gt_depth_sub],
            queue_size=10,
            slop=0.05
        )
        self.sync.registerCallback(self.synced_callback)
    
    def synced_callback(self, rgb_msg, depth_msg):
        # RGB
        cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        self.get_logger().info('Depth Header: '+str(depth_msg.header))

        # Ground-truth depth (meters)
        gt_depth = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')

        # Estimated depth (relative)
        self.get_logger().info('GT Depth shape: '+str(gt_depth.shape))
        est_depth = self.model.infer_image(rgb_image).astype(np.float32)
        self.get_logger().info('Est Depth shape: '+str(est_depth.shape))
        # est_depth = est_depth*3
        # est_depth = (np.mean(est_depth) - est_depth)  # Invert depth
        est_depth_scaled = self.align_scale(est_depth, gt_depth)
        # est_depth_scaled = est_depth

        h, w = gt_depth.shape
        cy, cx = h // 2, w // 2

        top_left     = (cx - 100, cy - 1)
        bottom_right = (cx - 80,  cy + 30)
        center_3x3 = gt_depth[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]]

        self.get_logger().info("GT: "+str(np.mean(center_3x3)))

        center_3x3_est = est_depth_scaled[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]]
        self.get_logger().info("Est: "+str(np.mean(center_3x3_est)))

        # self.get_logger().info(est_depth_scaled[cy-1:cy+2, cx-1:cx+2])

       
        # self.compare_depths(est_depth, gt_depth)
        # error = np.abs(est_depth*100 - gt_depth)

        # error_vis = cv2.normalize(
        #     error, None, 0, 255, cv2.NORM_MINMAX
        # ).astype(np.uint8)

        # cv2.imshow("Depth Error", error_vis)
        # cv2.waitKey(1)

        valid_mask = np.isfinite(gt_depth) & (gt_depth > 0)

        est_depth_scaled = self.align_scale(est_depth, gt_depth)
        error = np.abs(est_depth_scaled - gt_depth)

        def normalize(img, mask):
            norm = cv2.normalize(
                img, None, 0, 255, cv2.NORM_MINMAX
            ).astype(np.uint8)

            norm[~mask] = 0
            return norm

        gt_vis   = normalize(gt_depth, valid_mask)
        est_vis  = normalize(est_depth_scaled, valid_mask)
        error_vis = normalize(error, valid_mask)
        gt_color    = cv2.applyColorMap(gt_vis, cv2.COLORMAP_JET)
        est_color   = cv2.applyColorMap(est_vis, cv2.COLORMAP_JET)
        error_color = cv2.applyColorMap(error_vis, cv2.COLORMAP_HOT)

        color = (0, 255, 0)  # Green square
        thickness = 2

        cv2.rectangle(gt_color, top_left, bottom_right, color, thickness)
        cv2.rectangle(est_color, top_left, bottom_right, color, thickness)
        cv2.rectangle(error_color, top_left, bottom_right, color, thickness)

        

        combined = np.hstack((gt_color, est_color, error_color))
        cv2.imshow("GT | Estimated | Error", combined)

        depth_image_msg = self.bridge.cv2_to_imgmsg(est_depth_scaled,encoding="32FC1")

        # depth_image_msg = self.bridge.cv2_to_imgmsg(rgb_image,encoding="rgb8")
        # depth_image_msg.header.stamp = self.get_clock().now().to_msg()
        # depth_image_msg.header.frame_id = "camera_depth_estimation"
        depth_image_msg.header = depth_msg.header
        self.depth_pub.publish(depth_image_msg)
        cv2.waitKey(1)

    def align_scale(self, est, gt):
        mask = np.isfinite(gt) & (gt > 0)

        scale = np.sum(gt[mask] * est[mask]) / np.sum(est[mask] ** 2)
        return est * scale

    def compute_metrics(self, est, gt):
        mask = np.isfinite(gt) & (gt > 0)

        est = est[mask]
        gt = gt[mask]

        abs_rel = np.mean(np.abs(est - gt) / gt)
        rmse = np.sqrt(np.mean((est - gt) ** 2))
        delta1 = np.mean(np.maximum(est / gt, gt / est) < 1.25)

        return abs_rel, rmse, delta1


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