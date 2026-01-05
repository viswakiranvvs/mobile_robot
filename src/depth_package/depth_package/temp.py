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

cap = cv2.VideoCapture(0)
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
model = model.to(DEVICE).eval()
bridge = CvBridge()

def normalize(img):
    norm = cv2.normalize(
        img, None, 0, 255, cv2.NORM_MINMAX
    ).astype(np.uint8)
    return norm

if not cap.isOpened():
    print("Error: Cannot open camera")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break   

    rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    depth_estimate = model.infer_image(rgb_image).astype(np.float32)
    # combined = np.hstack((rgb_image, depth_estimate))
    # cv2.imshow("GT | Estimated | Error", combined)
    cv2.imshow("Logitech Camera", frame)
    depth_vis = normalize(depth_estimate)
    depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
    cv2.imshow("Estimated Depth", depth_vis)
    # cv2.imshow("Estimated Depth", depth_estimate / np.max(depth_estimate))

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
