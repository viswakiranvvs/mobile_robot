import argparse
import pickle
import threading
# from mapReader.map_ui import MapUI
from mapReader.pose_to_veloc import PathFollower
import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetMap
import numpy as np
if not hasattr(np, 'float'):
    np.float = float
import matplotlib.pyplot as plt
import cv2
from rclpy.node import Node
from rclpy.client import Client
from rclpy.task import Future
from rtabmap_msgs.srv import GetMap as GetMapDataSrv  # Not nav_msgs GetMap
from nav_msgs.srv import GetMap as GetOccupancyMapSrv
# from mpl_toolkits.mplot3d import Axes3D
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
import math
# from tf_transformations import quaternion_from_euler  # If not available, use tf2 or custom method
from nav2_simple_commander.robot_navigator import BasicNavigator
import time
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from cv_bridge import CvBridge
# Import the service and message types
from sensor_msgs.msg import Image
import json
from rtabmap_msgs.srv import GetNodeData
from rtabmap_msgs.msg import Node as NodeData
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
from typing import List, Optional 
import tf_transformations
from mapReader.yolo_class import LightweightYolo
from geometry_msgs.msg import PoseWithCovarianceStamped

# def compute_orientation(from_point, to_point):
#     yaw = math.atan2(to_point.y - from_point.y, to_point.x - from_point.x)
#     q = quaternion_from_euler(0, 0, yaw)
#     return q  # (x, y, z, w)

def pose_stamped_to_tuple(pose_stamped):
    pos = pose_stamped.pose.position
    return (round(pos.x, 3), round(pos.y, 3), round(pos.z, 3))  # rounding to avoid float precision issues

def pose_to_tuple(pose):
    pos = pose.position
    return (round(pos.x, 3), round(pos.y, 3), round(pos.z, 3))  # rounding to avoid float precision issues


class MapClient(Node):
    def __init__(self):
        super().__init__('map_client')
        
        # Create client for occupancy map
        self.occ_map_client = self.create_client(GetOccupancyMapSrv, '/rtabmap/rtabmap/get_map')
        self.cli = self.occ_map_client
        self.yolo = LightweightYolo(model_path="yolov8n.pt", device="cpu", conf=0.35, imgsz=640)

        # while not self.occ_map_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for Occupancy Map service...')

        # Create client for RTAB-Map map data
        self.map_data_client = self.create_client(GetMapDataSrv, '/rtabmap/rtabmap/get_map_data')
        # while not self.map_data_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for Map Data service...')

        self.get_plan_client = self.create_client(GetPlan, '/rtabmap/rtabmap/get_plan')

        self.navigator = BasicNavigator()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # while not self.get_plan_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for /rtabmap/rtabmap/get_plan...')
        # self.send_request()
        # self.get_logger().info('Calling both services...')
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose', # The topic your NonlinearController listens to
            10
        )
        self.current_pose = None
        self.localization_pose = self.create_subscription(PoseWithCovarianceStamped, '/rtabmap/localization_pose',self.update_pose,10)

        self.get_node_info_client = self.create_client(GetNodeData, "/rtabmap/rtabmap/get_node_data")

        # while not self.client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("Service not available, waiting...")

        # self.req = GetNodeData.Request()

    def fetch_pose_once(self, timeout_sec=30.0):
        """Fetch one PoseStamped from /rtabmap/localization_pose."""
        self.current_pose = None

        # Temporary subscription
        # sub = self.create_subscription(
        #     PoseStamped,
        #     '/rtabmap/localization_pose',
        #     self.update_pose,
        #     1
        # )

        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        while rclpy.ok() and self.current_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = self.get_clock().now().seconds_nanoseconds()[0]
            if now - start_time > timeout_sec:
                self.get_logger().warn("Timeout waiting for pose message")
                break

        # self.destroy_subscription(sub)
        return self.current_pose
    
    def update_pose(self, msg):
        """Callback to update the latest pose."""
        self.get_logger().info(f"Pose: {msg}")
        self.current_pose = msg.pose.pose
    
    def get_node_data(self, node_ids: List[int]) -> Optional[List[NodeData]]:
        """Get node data for specified node IDs"""
        try:
            request = GetNodeData.Request()
            request.ids = node_ids
            request.images = True      # Request images
            request.scan = False       # Don't request scan data
            request.grid = False       # Don't request grid data
            request.user_data = False  # Don't request user data
            
            future = self.get_node_info_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            if response is not None:
                return response.data
            # if future.done():
            #     response = future.result()
            #     return response.data
            # else:
            #     self.get_logger().warn('Service call did not complete')
            #     return None
                
        except Exception as e:
            self.get_logger().error(f'Error calling service: {str(e)}')
            return None

    def send_request(self):
        self.req = GetMap.Request()
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            map_msg = future.result().map
            self.get_logger().info('Map received:')
            # Access all fields from the message
            print("Header Frame ID:", map_msg.header.frame_id)
            print("Map Resolution:", map_msg.info.resolution)
            print("Map Width x Height:", map_msg.info.width, 'x', map_msg.info.height)
            print("Origin Position:", map_msg.info.origin.position.x,
                  map_msg.info.origin.position.y, map_msg.info.origin.position.z)
            print("Origin Orientation:", map_msg.info.origin.orientation)
            print("Data length:", len(map_msg.data))

            # Save as Python structured object
            self.map_data = {
                "frame_id": map_msg.header.frame_id,
                "resolution": map_msg.info.resolution,
                "width": map_msg.info.width,
                "height": map_msg.info.height,
                "origin": {
                    "position": {
                        "x": map_msg.info.origin.position.x,
                        "y": map_msg.info.origin.position.y,
                        "z": map_msg.info.origin.position.z
                    },
                    "orientation": {
                        "x": map_msg.info.origin.orientation.x,
                        "y": map_msg.info.origin.orientation.y,
                        "z": map_msg.info.origin.orientation.z,
                        "w": map_msg.info.origin.orientation.w
                    }
                },
                "data": list(map_msg.data)  # convert to list of int8
            }
            self.visualize_occupancy_grid(self.map_data)

        else:
            self.get_logger().error('Service call failed')
    
    def visualize_occupancy_grid(self,map_data):
        width = map_data["width"]
        height = map_data["height"]
        data = map_data["data"]

        # Convert to numpy array and reshape to (height, width)
        grid = np.array(data, dtype=np.int8).reshape((height, width))

        # Convert to displayable image (0=free → white, 100=occupied → black, -1=unknown → gray)
        img = np.zeros((height, width), dtype=np.uint8)
        img[grid == -1] = 128  # Unknown = gray
        img[grid == 0] = 255   # Free = white
        img[grid > 0] = 0      # Occupied = black

        # Flip vertically (to match RViz view)
        img = np.flipud(img)

        # Show using OpenCV or matplotlib
        # cv2.imshow("Occupancy Grid", img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        # OR matplotlib (if headless)
        plt.imshow(img, cmap='gray')
        plt.title("RTAB-Map Occupancy Grid")
        plt.savefig("detections/occupancy_grid.png")
        # plt.show()
    
    def call_get_occupancy_map(self):
        request = GetOccupancyMapSrv.Request()
        future = self.occ_map_client.call_async(request)
        print("Calling occupancy service")
        rclpy.spin_until_future_complete(self, future)
        self.handle_occupancy_response(future)
        # future.add_done_callback(self.handle_occupancy_response)

    def call_get_map_data(self, retries=3, timeout_sec=10.0):
        request = GetMapDataSrv.Request()
        request.global_map = True
        request.optimized = True
        request.graph_only = False

        for attempt in range(retries):
            self.get_logger().info(f"Calling Map Data Service (attempt {attempt+1}/{retries})")

            future = self.map_data_client.call_async(request)

            start = time.time()
            while not future.done() and (time.time() - start) < timeout_sec:
                rclpy.spin_once(self, timeout_sec=0.1)

            if future.done():
                try:
                    response = future.result()
                    self.get_logger().info("Received, calling handler")
                    self.handle_map_data_response(future, False)
                    return response
                except Exception as e:
                    self.get_logger().error(f"Service call failed: {e}")
            else:
                self.get_logger().warn("Timeout waiting for response, retrying...")

        self.get_logger().error("All retries failed, service unavailable.")
        return None
        
        # future.add_done_callback(self.handle_map_data_response)

    def handle_occupancy_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Received Occupancy Grid map: {response.map.info.width}x{response.map.info.height}")
            # print(f"Received Occupancy Grid map: {response.map.info.width}x{response.map.info.height}")
            self.map_data = {
                "width": response.map.info.width,
                "height": response.map.info.height,
                "resolution": response.map.info.resolution,
                "origin": {
                    "x": response.map.info.origin.position.x,
                    "y": response.map.info.origin.position.y
                },
                "data": response.map.data
            }
            # You can visualize or save it here
            self.visualize_occupancy_grid(self.map_data)
        except Exception as e:
            print(f"Failed to get occupancy map: {e}")
            self.get_logger().error(f"Failed to get occupancy map: {e}")

    def visualize_map(self,data):
        fig = plt.figure(figsize=(15, 10))
        ax = fig.add_subplot(111, projection='3d')

        poses_x, poses_y, poses_z = [], [], []
        pts_x, pts_y, pts_z = [], [], []

        for node in data.nodes:
            # Extract pose position (robot location)
            pos = node.pose.position
            poses_x.append(pos.x)
            poses_y.append(pos.y)
            poses_z.append(pos.z)

            # Extract 3D feature points
            for pt in node.word_pts:
                pts_x.append(pt.x)
                pts_y.append(pt.y)
                pts_z.append(pt.z)

        # Plot robot poses
        ax.scatter(poses_x, poses_y, poses_z, c='blue', marker='o', label='Robot Poses')

        # Plot 3D feature points
        ax.scatter(pts_x, pts_y, pts_z, c='red', s=1, label='3D Map Points')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('RTAB-Map: Node Poses & 3D Landmarks')
        ax.legend()
        plt.tight_layout()
        # plt.show()
        plt.savefig("detections/3d_map.png")

    def plot_map_graph(self,graph):
        id_to_pose = {}  # Map from ID to (x, y) for fast lookup

        # Extract poses
        for i, node_id in enumerate(graph.poses_id):
            pos = graph.poses[i].position
            id_to_pose[node_id] = (pos.z, pos.x)

        fig, ax = plt.subplots(figsize=(10, 8))

        # Plot poses
        xs = [pos[0] for pos in id_to_pose.values()]
        ys = [pos[1] for pos in id_to_pose.values()]
        ax.scatter(xs, ys, c='blue', label='Optimized Poses')

        # Draw links (edges)
        for link in graph.links:
            from_id = link.from_id
            to_id = link.to_id

            if from_id in id_to_pose and to_id in id_to_pose:
                x0, y0 = id_to_pose[from_id]
                x1, y1 = id_to_pose[to_id]
                ax.plot([x0, x1], [y0, y1], c='gray', linewidth=0.8)

        ax.set_title('Optimized Pose Graph from RTAB-Map')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.axis('equal')
        ax.legend()
        plt.grid(True)
        plt.tight_layout()
        # plt.show()
        plt.savefig("detections/pose_graph.png")
    
    def load_map_data(self,path="map_data.pkl"):
        with open(path, "rb") as f:
            data = pickle.load(f)
        return data
    
    def extract_detected_images(self, node_id, detections, cv_image):
        images_dict = dict()
        for i, det in enumerate(detections):

            x1, y1, x2, y2 = det["bbox"]  # bounding box
            class_name = det["class_name"]
            confidence = det["confidence"]
            if confidence<=0.6:
                continue
            # Crop from original cv_image
            cropped = cv_image[y1:y2, x1:x2]

            # Optionally save each detection
            filename = f"detections/detection_{node_id}_{i}_{class_name}_.jpg"
            temp_dict = dict()
            temp_dict["bbox"] = det["bbox"]
            temp_dict["confidence"] = det["confidence"]
            temp_dict["file"] = filename
            images_dict[class_name].append(temp_dict) if class_name in images_dict else images_dict.update({class_name:[temp_dict]})
            cv2.imwrite(filename, cropped)

            # Or just display
            # cv2.imshow(f"Detection {i}", cropped)
            # cv2.waitKey(0)
        return images_dict

    
    def extract_images(self, node_data: NodeData) -> List[np.ndarray]:
        """Extract and decode images from node data"""
        images = []
        # node_dict=dict()
            # Initialize CV bridge if not already done
        if not hasattr(self, 'bridge'):
            self.bridge = CvBridge()
        
        # Extract left image (RGB)
        # if node_data.data.left:
        #     try:
        #         node_data.data.left.encoding = "bgr8"
        #         self.get_logger().info(f'Encoding: {node_data.data.left.encoding}')
        #         cv_image = self.bridge.imgmsg_to_cv2(node_data.data.left, "bgr8")
        #         images.append(('RGB Image (Left)', cv_image))
        #     except Exception as e:
        #         self.get_logger().warn(f'Failed to decode RGB image: {str(e)}')
        
        if node_data.data.left_compressed:
            try:
                np_arr = np.frombuffer(node_data.data.left_compressed, np.uint8)
                self.get_logger().info(f'left comp shape: {np_arr.shape}')
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                self.get_logger().info(f'left cv shape: {cv_image.shape}')
                detections, annotated = self.yolo.detect(cv_image, return_image=True)
                self.get_logger().info(f"Detections for node {node_data.id}: {detections}")
                images_dict = self.extract_detected_images(node_data.id,detections, cv_image)
                images.append(('RGB Image Compressed (Left)', annotated))
            except Exception as e:
                self.get_logger().warn(f'Failed to decode RGB Compressed image: {str(e)}')
        # node_dict[node_data.id]=images_dict
        if node_data.data.right_compressed:
            self.get_logger().info(f'right comp: {type(node_data.data.right_compressed)}')
            try:
                np_arr = np.frombuffer(node_data.data.right_compressed, np.uint8)
                self.get_logger().info(f'right comp shape: {np_arr.shape}')
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
                self.get_logger().info(f'right cv shape: {cv_image.shape}')
                images.append(('Depth Image Compressed (Right)', cv_image))
            except Exception as e:
                self.get_logger().warn(f'Failed to decode Depth Compressed image: {str(e)}')
        # json_file = 
        # Extract right image (Depth)
        # if node_data.data.right:
        #     self.get_logger().info(f'right depth: {type(node_data.data.right)}')
        #     self.get_logger().info(f'right depth shape: {(node_data.data.right.data.shape)}')
        #     try:
        #         node_data.data.right.encoding = "16UC1"
        #         cv_depth = self.bridge.imgmsg_to_cv2(node_data.data.right, "passthrough")
        #         # Normalize depth image for visualization
        #         if cv_depth.dtype == np.uint16:
        #             cv_depth_normalized = cv2.normalize(cv_depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        #             images.append(('Depth Image (Right)', cv_depth_normalized))
        #         else:
        #             images.append(('Depth Image (Right)', cv_depth))
        #     except Exception as e:
        #         self.get_logger().warn(f'Failed to decode depth image: {str(e)}')
        
        # Display each image individually
        if not images:
            self.get_logger().warn(f'No images found for node {node_data.id}')
            return
        
        self.get_logger().info(f'Found {len(images)} images for node {node_data.id}')
        
        # for title, image in images:
        #     # Create window with node ID in title
        #     if image is None or image.size == 0:
        #         self.get_logger().warn(f"Skipping empty image for {title}")
        #         continue
        #     window_name = f'Node {node_data.id}: {title}'
        #     cv2.imshow(window_name, image)
        #     break
        #     # cv2.waitKey(0)  # Short delay to allow window creation
        
        # # Wait for key press before closing
        # self.get_logger().info('Press any key to close images...')
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        
        return images_dict

        
        # return images
        # Extract compressed images if available
        # if node_data.data.image_compressed.format:
        #     try:
        #         cv_image = self.bridge.compressed_imgmsg_to_cv2(node_data.data.image_compressed, "bgr8")
        #         images.append(('Compressed RGB', cv_image))
        #     except Exception as e:
        #         self.get_logger().warn(f'Failed to decode compressed RGB image: {str(e)}')
        
        # if node_data.data.depth_compressed.format:
        #     try:
        #         cv_depth = self.bridge.compressed_imgmsg_to_cv2(node_data.data.depth_compressed, "passthrough")
        #         if cv_depth.dtype == np.uint16:
        #             cv_depth_normalized = cv2.normalize(cv_depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        #             images.append(('Compressed Depth', cv_depth_normalized))
        #     except Exception as e:
        #         self.get_logger().warn(f'Failed to decode compressed depth image: {str(e)}')
        
    def handle_map_data_response(self, future=None, dummy=False):
        try:
            if dummy:
                data=self.load_map_data()
            else:
                response = future.result()
                data = response.data
                save_path = "detections/map_data.pkl"
                with open(save_path, "wb") as f:
                    pickle.dump(data, f)
            self.get_logger().info(f"Received MapData: {len(data.nodes)} nodes, {len(data.graph.poses)} graph poses")

            # You can access the graph and node info like:
            # print(data.nodes[0])
            node_ids = []
            for i, node in enumerate(data.nodes):
                self.get_logger().info(f"Node {node.id}: pose = {node.pose}") # {node.pose.pose.position.y}
                node_ids.append(node.id)
            
            node_data_list = self.get_node_data(node_ids)
        
            if not node_data_list:
                self.get_logger().error('Failed to get node data')
                return
            
            all_images = dict()
            for i, node_data in enumerate(node_data_list):
                self.get_logger().info(f'Node {i}: ID={node_data.id}, Map ID={node_data.map_id}')
                images_dict = self.extract_images(node_data)
                if len(images_dict)>0:
                    all_images[node_data.id]=images_dict
                # all_images.append(images)
            
            with open("detections/node_images.json", "w") as f:
                json.dump(all_images, f, indent=4)
            self.get_logger().info(f'\nLength of images: {len(all_images)}\n')

            # for i in range(0,5):
            #     self.get_logger().info(f"Pose Id {data.graph.poses_id[i]}")
            #     self.get_logger().info(f"Pose {data.graph.poses[i]}")
                # self.get_logger().info(f"Pose Id {data.graph.poses[0].id}: pose = {pose}") # {node.pose.pose.position.y}
            self.visualize_map(data)
            self.plot_map_graph(data.graph)
            # self.get_logger().info(data.graph.poses[0].position)
            # poses = self.get_plan(data.graph.poses[0],data.graph.poses[-1],tolerance=0.1,frame_id='map')
            # self.publish_poses(poses)

        except Exception as e:
            self.get_logger().error(f"Failed to get map data: {e}")



    def transform_pose(self, msg: PoseStamped) -> PoseStamped:
        R_map_to_odom = np.array([
            [0, -1,  0],
            [0,  0, -1],
            [1,  0,  0]
        ])
        # Extract translation
        t = np.array([msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z])

        # Apply rotation to translation
        t_new = np.dot(t,R_map_to_odom)

        # Extract quaternion
        q = [msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w]

        # Convert quaternion to rotation matrix
        R_q = tf_transformations.quaternion_matrix(q)[0:3, 0:3]

        # R_new = R_map_to_odom @ R_q
        R_new = np.dot(R_q, R_map_to_odom) # R_map_to_odom @ R_q

        # Convert back to quaternion
        T_new = np.eye(4)
        T_new[0:3, 0:3] = R_new
        q_new = tf_transformations.quaternion_from_matrix(T_new)
        q_new = [q[0], q[2], -1*q[1], q[3]]

        # Apply extra rotation
        # R_new = np.dot(R_q, R_map_to_odom) # R_map_to_odom @ R_q

        # # Convert back to quaternion
        # q_new = tf_transformations.quaternion_from_matrix(
        #     np.block([
        #         [R_new, np.zeros((3,1))],
        #         [np.zeros((1,3)), np.array([[1]])]
        #     ])
        # )

        # Build new PoseStamped
        out = PoseStamped()
        out.header = msg.header
        out.header.frame_id = "odom"   # or base_link_frame
        out.pose.position.x = t_new[0]
        out.pose.position.y = t_new[1]
        out.pose.position.z = t_new[2]
        out.pose.orientation.x = q_new[0]
        out.pose.orientation.y = q_new[1]
        out.pose.orientation.z = q_new[2]
        out.pose.orientation.w = q_new[3]

        return out

    def map_to_baselink(self, msg: PoseStamped):
        try:
            # Transform pose from "map" → "drone_base_link"
            # target_frame = 'drone_base_link'
            # Inside your ROS2 node
            odom_frame = 'odom'
            base_link_frame = 'drone_base_link'
            map_frame = 'map'
            camera_frame = 'Camera_OmniVision_OV9782_Color'


            R = np.array([
                [0, -1,  0],
                [0,  0, -1],
                [1,  0,  0]
            ])

            map_to_odom_pose = self.tf_buffer.transform(
                msg,
                base_link_frame,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            # odom_to_camera_pose = self.tf_buffer.transform(
            #     map_to_odom_pose,
            #     camera_frame,
            #     timeout=rclpy.duration.Duration(seconds=0.5)
            # )
            
            # transformed_pose = self.tf_buffer.transform(
            #     msg,
            #     target_frame,
            #     timeout=rclpy.duration.Duration(seconds=0.5)
            # )
            # self.get_logger().info(
            #     f"Pose in {target_frame}: {transformed_pose.pose.position}"
            # )
            return map_to_odom_pose
        except Exception as e:
            self.get_logger().warn(f"Transform failed: {str(e)}")
   
    def publish_poses(self,poses):
        # from rclpy.duration import Duration

        # Get frame graph
        frames_yaml = self.tf_buffer.all_frames_as_yaml()
        self.get_logger().info(frames_yaml)

        # Or as a string
        frames_str = self.tf_buffer.all_frames_as_string()
        self.get_logger().info(frames_str)

        for pose in poses:
            # pose.pose.position.z = -1*pose.pose.position.z
            transformedPose = self.transform_pose(pose)
            # temp=transformedPose.pose.position.z
            # transformedPose.pose.position.z = -1*transformedPose.pose.position.x  # Maintain 1m altitude above ground
            # transformedPose.pose.position.x = -1*temp
            self.goal_publisher.publish(transformedPose)
            self.get_logger().info(f"Published goal pose: ({transformedPose.pose.position.x:.2f}, {transformedPose.pose.position.y:.2f}, {transformedPose.pose.position.z:.2f})")
            self.get_logger().info(f"Orientation: ({transformedPose.pose.orientation.x:.2f}, {transformedPose.pose.orientation.y:.2f}, {transformedPose.pose.orientation.z:.2f}, {transformedPose.pose.orientation.w:.2f})")
            # self.goal_publisher.publish(pose)
            # self.get_logger().info(f"Published goal pose: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, {pose.pose.position.z:.2f})")
            time.sleep(1)
    
    def navigate_To_Node(self,node_id):
        data=self.load_map_data("/home/robot2/Documents/isaac_sim/mobile_robot/src/ui/detections/map_data.pkl")
        goal_pose = None
        current_pose = self.fetch_pose_once()
        if current_pose is None:
            self.get_logger().error("Failed to get current pose, cannot navigate")
            return
        for i,node in enumerate(data.nodes):
            if node.id==node_id:
                goal_pose = node.pose
                self.get_logger().info(f"Types of goal: {type(goal_pose)}\ncurrent pose: {type(current_pose)}")
                break
        poses = self.get_plan(current_pose,goal_pose)
        self.publish_poses(poses)

    def get_plan(self,Start, End, tolerance=0.5, frame_id="map"):
        req = GetPlan.Request()

        # req.start = PoseStamped()
        # req.start.header.frame_id = frame_id
        # req.start.pose.position.x = Start.position.x
        # req.start.pose.position.y = Start.position.y
        # req.start.pose.position.z = Start.position.z

        # req.start.pose.orientation.x = Start.orientation.x
        # req.start.pose.orientation.y = Start.orientation.y
        # req.start.pose.orientation.z = Start.orientation.z
        # req.start.pose.orientation.w = Start.orientation.w



        self.get_logger().info(f"Path from ({Start.position.x:.2f}, {Start.position.y:.2f})")
        self.get_logger().info(f"Path to ({End.position.x:.2f}, {End.position.y:.2f})")

        req.goal = PoseStamped()
        req.goal.header.frame_id = frame_id
        req.goal.pose.position.x = End.position.x
        req.goal.pose.position.y = End.position.y
        req.goal.pose.position.z = End.position.z

        req.goal.pose.orientation.x = End.orientation.x
        req.goal.pose.orientation.y = End.orientation.y
        req.goal.pose.orientation.z = End.orientation.z
        req.goal.pose.orientation.w = End.orientation.w


        # now = self.get_clock().now().to_msg()
        # req.start.header.stamp = now
        # req.goal.header.stamp = now

        req.tolerance = tolerance
        self.get_logger().info("Calling Get Plan Service")

        future = self.get_plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            path = future.result().plan
            set1 = path.poses
            self.get_logger().info(f"Received path with {len(path.poses)} poses")

        req.goal.pose.position.x = Start.position.x
        req.goal.pose.position.y = Start.position.y
        req.goal.pose.position.z = Start.position.z

        req.goal.pose.orientation.x = Start.orientation.x
        req.goal.pose.orientation.y = Start.orientation.y
        req.goal.pose.orientation.z = Start.orientation.z
        req.goal.pose.orientation.w = Start.orientation.w

        self.get_logger().info("Calling Get Plan Service")

        future = self.get_plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            # print(future.result())
            path = future.result().plan
            set2 = path.poses
            self.get_logger().info(f"Received path with {len(path.poses)} poses")
        
        if set1 and set2:

            # print(path)
            # for i, pose in enumerate(path.poses[:5]):
            #     self.get_logger().info(f"Pose {i}: {pose.pose.position}")
            pose_set1 = set(pose_stamped_to_tuple(p) for p in set1)
            pose_set2 = set(pose_stamped_to_tuple(p) for p in set2)
            # print(pose_set1)
            # print(pose_set2)
            if len(pose_set2) > len(pose_set1):
                # common_poses = pose_set2.difference(pose_set1)
                pose_set1_tuples = set(pose_stamped_to_tuple(p) for p in set1)
                common_poses = [p for p in set2 if pose_stamped_to_tuple(p) not in pose_set1_tuples]
            else:
                # common_poses = pose_set1.difference(pose_set2)
                pose_set2_tuples = set(pose_stamped_to_tuple(p) for p in set2)
                common_poses = [p for p in set1 if pose_stamped_to_tuple(p) not in pose_set2_tuples]

            start_tuple = pose_to_tuple(Start)
            end_tuple = pose_to_tuple(End)

            common_pose_tuples = set(pose_stamped_to_tuple(p) for p in common_poses)

            # If Start or End are missing, insert at beginning (reverse order if you want End first)
            if end_tuple not in common_pose_tuples:
                end_pose_stamped = PoseStamped()
                end_pose_stamped.header.frame_id = frame_id
                end_pose_stamped.pose = End
                common_poses.insert(0, end_pose_stamped)

            if start_tuple not in common_pose_tuples:
                start_pose_stamped = PoseStamped()
                start_pose_stamped.header.frame_id = frame_id
                start_pose_stamped.pose = Start
                common_poses.insert(0, start_pose_stamped)
            # sorted_common_poses = sorted(common_poses, key=lambda p: (p[0], p[1]))
            # common_poses.insert(0)

            for pose in reversed(common_poses):
                print(pose.pose.position.x,pose.pose.position.y,pose.pose.position.z)
                print(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w)
                print("====")
            # Extract x, y from path
            x_vals = [pose.pose.position.x for pose in common_poses]
            y_vals = [pose.pose.position.y for pose in common_poses]

            # x_vals = [pose[0] for pose in sorted_common_poses]
            # y_vals = [pose[1] for pose in sorted_common_poses]

            

            # Plot path
            plt.figure(figsize=(8, 6))
            plt.plot(x_vals, y_vals, marker='o', linestyle='-', label='Planned Path')

            # for i, (x, y) in enumerate(zip(x_vals, y_vals)):
            #     if i % 10 == 0:
            #         plt.plot(x, y, 'ko')  # black dots
            # Plot start and end
            plt.scatter(Start.position.x, Start.position.y, c='green', s=100, label='Start')
            plt.scatter(End.position.x, End.position.y, c='red', s=100, label='Goal')

            plt.title("RTAB-Map Global Path")
            plt.xlabel("X")
            plt.ylabel("Y")
            plt.legend()
            plt.grid(True)
            plt.axis('equal')
            plt.savefig("detections/planned_path.png")
            # plt.show()
            return common_poses
        else:
            self.get_logger().error("Service call failed")

def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", type=str, default="normal")
    parser.add_argument("--node", type=int, default=-1)
    args_ = parser.parse_args()

    print("Mode:", args_.mode)
    print("Count:", args_.node)
    rclpy.init(args=args)
    map_client = MapClient()
    # ui = MapUI(map_client)
    # ui_thread = threading.Thread(target=ui.run)
    # ui_thread.start()
    # map_client.send_request()
    # map_client.call_get_occupancy_map()
    # map_client.call_get_occupancy_map()
    while rclpy.ok():
        rclpy.spin_once(map_client, timeout_sec=0.1)
        if args_.mode=="mapRead":
            map_client.call_get_map_data()
        elif args_.mode=="navigate":
            map_client.navigate_To_Node(args_.node)
        # map_client.get_plan()
        map_client.destroy_node()
        rclpy.shutdown()
    # map_client.handle_map_data_response(None,True)
    # map_client.destroy_node()
    # rclpy.shutdown()
    # ui_thread.join()

if __name__ == '__main__':
    main()
