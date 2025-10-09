from datetime import datetime
import math
import shutil
import time
from bot_controller.lidar_reader import LidarDistanceAnalyzer
from bot_controller.marker_publish import ObjectMarkerPublisher
from mapReader.yolo_class import LightweightYolo
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from nav_msgs.msg import Odometry
import os
import json
import tf2_ros
# import tf_transformations
from geometry_msgs.msg import PoseStamped
import numpy as np
if not hasattr(np, 'float'):
    np.float = float
# import numpy as np
import numpy as np
import tf2_geometry_msgs
from rclpy.executors import MultiThreadedExecutor

class CamReader(Node):
    def __init__(self):
        self.yolo = LightweightYolo(model_path="yolov8n.pt", device="cpu", conf=0.35, imgsz=640)
        self.marker_pub = ObjectMarkerPublisher()
        # self.lidar_reader = lidar_node
        super().__init__('cam_reader_node')
        self.bridge = CvBridge()
        self.mesh_folder = os.path.join(os.getcwd(), "meshes")
        self.install_mesh_folder = os.path.join(os.getcwd(), "install/bot_controller/share/bot_controller/meshes")
        os.makedirs(self.install_mesh_folder, exist_ok=True)
        # os.makedirs(self.mesh_folder, exist_ok=True)
        shutil.rmtree(self.mesh_folder)
        os.makedirs(self.mesh_folder, exist_ok=True)
        self.cam_subscription = self.create_subscription(
            Image,
            '/camera_rgb',
            self.cam_callback,
            10
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.odom = None
        self.prev_odom = None
        self.yolo_image =self.create_publisher(Image, '/yolo_image', 10)
        self.accepted_detections = ["truck","cone","bottle","vase","box",
                    "extinguisher","banana","person","bench","ball","sports ball",]
        self.object_markers=[]
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # detections, annotated = self.yolo.detect(cv_image, return_image=True)
    
    def odom_callback(self, msg):
        # self.prev_odom = self.odom
        self.odom = msg
    
    def tranform_pose(self, pose, from_frame='odom', to_frame='map'):
        """
        Transform a PoseStamped from one frame to another using tf2.
        """
        
        pose_in_odom = PoseStamped()
        pose_in_odom.header.frame_id = from_frame
        pose_in_odom.pose = pose

        self.get_logger().info(self.tf_buffer.all_frames_as_string())

        # Transform to map frame
        try:
            pose_in_map = self.tf_buffer.transform(pose_in_odom, to_frame, timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(f"Transform failed: {e}")
            return None
        return pose_in_map
    
    def extract_detected_images(self, node_id, det, cv_image, image_filename):
        images_dict = dict()
        # for i, det in enumerate(detections):
        x1, y1, x2, y2 = det["bbox"]  # bounding box
        class_name = det["class_name"]
        confidence = det["confidence"]
        # if confidence<=0.6:
        #     continue
        # Crop from original cv_image
        cropped = cv_image[y1:y2, x1:x2]

        # Optionally save each detection
        image_path = os.path.join(self.mesh_folder, image_filename)
        cv2.imwrite(image_path, cropped)
        shutil.copy(image_path, self.install_mesh_folder)
        # filename = f"detections/detection_{node_id}_{i}_{class_name}_.jpg"
        # temp_dict = dict()
        # temp_dict["bbox"] = det["bbox"]
        # temp_dict["confidence"] = det["confidence"]
        # temp_dict["file"] = filename
        # images_dict[class_name].append(temp_dict) if class_name in images_dict else images_dict.update({class_name:[temp_dict]})
        # cv2.imwrite(filename, cropped)

            # Or just display
            # cv2.imshow(f"Detection {i}", cropped)
            # cv2.waitKey(0)
        return images_dict

    def odom_movement_diff(self):
        if self.prev_odom is None or self.odom is None:
            return 1, 0.6  # No movement if we don't have previous odom

        # Extract position
        x1 = self.prev_odom.pose.pose.position.x
        y1 = self.prev_odom.pose.pose.position.y
        z1 = self.prev_odom.pose.pose.position.z

        x2 = self.odom.pose.pose.position.x
        y2 = self.odom.pose.pose.position.y
        z2 = self.odom.pose.pose.position.z

        # Euclidean distance
        linear_diff = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
        angular_diff = 0.0
        # Extract yaw difference from orientation
        # import tf_transformations
        # q1 = self.prev_odom.pose.pose.orientation
        # q2 = self.odom.pose.pose.orientation
        # _, _, yaw1 = tf_transformations.euler_from_quaternion([q1.x, q1.y, q1.z, q1.w])
        # _, _, yaw2 = tf_transformations.euler_from_quaternion([q2.x, q2.y, q2.z, q2.w])
        # angular_diff = abs(yaw2 - yaw1)
        # angular_diff = (angular_diff + math.pi) % (2*math.pi) - math.pi  # normalize [-pi, pi]
        
        return linear_diff, angular_diff
    
    def check_if_already_detected(self, det, pose, threshold=3):
        for marker in self.object_markers:
            dist = math.sqrt((pose.pose.position.x - marker["x"])**2 + (pose.pose.position.y - marker["y"])**2)
            if dist < threshold and det['class_name'] == marker["object_name"]:
                return True
        return False
    
    def get_object_direction(self,bbox, image_width, fov_deg=98):
        """
        bbox: (xmin, ymin, xmax, ymax)
        fov_deg: camera horizontal field of view
        """
        x_center = (bbox[0] + bbox[2]) / 2
        angle_per_pixel = fov_deg / image_width
        offset_from_center = x_center - (image_width / 2)
        angle_deg = offset_from_center * angle_per_pixel
        return angle_deg
    
    def get_distance_from_lidar(self, target_angle_deg):
        json_path = os.path.join(os.getcwd(), "lidar_data.json")
        angle_dict = {}
        with open(json_path, "r") as f:
            angle_dict = json.load(f)
        angle_keys = (list(angle_dict.keys()))
        angle_keys = [float(k) for k in angle_keys]
        closest_few_angles = sorted(angle_keys, key=lambda x: abs(x - target_angle_deg))[:40]
        min_distance = 50
        min_angle = None
        for angle in closest_few_angles:
            distance = angle_dict[str(angle)]
            if distance < min_distance:
                min_distance = distance
                min_angle = angle
        return min_distance, min_angle
    
    def cam_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        detections, annotated = self.yolo.detect(cv_image, return_image=True)
        yolo_image_msg = self.bridge.cv2_to_imgmsg(annotated)
        yolo_image_msg.header.stamp = self.get_clock().now().to_msg()
        yolo_image_msg.header.frame_id = "camera_yolo_detections"
        self.yolo_image.publish(yolo_image_msg)

        linear_diff, angular_diff = self.odom_movement_diff()
        # self.get_logger().info(f"Frame received: {len(detections)} detections, linear movement {linear_diff:.3f}, angular movement {angular_diff:.3f}")
        if linear_diff is not None and angular_diff is not None:
            if linear_diff < 1:
                # self.get_logger().info(f"Skipping frame due to low movement: linear {linear_diff:.3f}, angular {angular_diff:.3f}")
                return  # Skip processing this frame
        # elif self.prev_odom is None:
        #     return  # Skip if we don't have odom data yet
        
        detected_objects = [det for det in detections if det['class_name'] in self.accepted_detections]
        if len(detected_objects)>0 and self.odom is not None:
            detected_objects.sort(key=lambda d: d['confidence'], reverse=True)
            
            if detected_objects[0]['confidence']<0.5:
                return
            if detected_objects[0]['class_name']=="person" and detected_objects[0]['confidence']<0.7:
                return
            id = int(time.time() * 1000) % 1000000
            degree = -1* self.get_object_direction(detected_objects[0]['bbox'], cv_image.shape[1])
            if abs(degree)>20:
                self.get_logger().info(f"Skipping object {detected_objects[0]['class_name']} at {degree:.2f} degrees (out of range)")
                return
            self.get_logger().info(f"Degree {degree}")
            # obj_distance,angle = self.lidar_reader.get_distance_at_angle(degree)
            obj_distance,angle = self.get_distance_from_lidar(degree)
            angle = math.radians(-1*angle)
            if obj_distance is None or obj_distance>10.0:
                return
            

            
            self.prev_odom = self.odom
            x_offset = obj_distance * math.cos(angle)
            y_offset = obj_distance * math.sin(angle)
            self.odom.pose.pose.position.x += x_offset
            self.odom.pose.pose.position.y += y_offset
            tranform_pose = self.tranform_pose(self.odom.pose.pose, from_frame='odom', to_frame='map')
            if tranform_pose is None:
                self.get_logger().warn("Skipping frame due to transform failure")
                return
            
            if self.check_if_already_detected(detected_objects[0], tranform_pose):
                self.get_logger().info("Object already detected nearby, skipping...")
                return

            dae_filename, image_filename = self.create_mesh(annotated,self.get_clock().now().to_msg())
            self.extract_detected_images(id, detected_objects[0], cv_image, image_filename)
            
            new_marker = {
                "object_name": detected_objects[0]['class_name'],
                "mesh_name": dae_filename,
                # "x": self.odom.pose.pose.position.x,
                # "y": self.odom.pose.pose.position.y,
                "x": tranform_pose.pose.position.x,
                "y": tranform_pose.pose.position.y,
                "id": int(id)
            }
            self.get_logger().info(f"Detected {detected_objects[0]['class_name']} with confidence {detected_objects[0]['confidence']:.2f} at position ({new_marker['x']:.2f}, {new_marker['y']:.2f})")
            self.get_logger().info(f"Id for marker: {id} type: {type(id)}")
            self.object_markers.append(new_marker)
            json_path = os.path.join(self.mesh_folder, "detected_objects.json")
            with open(json_path, "w") as jf:
                json.dump(self.object_markers, jf, indent=4)
            for marker in self.object_markers:
                self.marker_pub.publish_object(int(marker["id"]),marker["object_name"],marker["mesh_name"], marker["x"], marker["y"], 0.1)
           
           
            # self.marker_pub.publish_object(detected_objects[0],dae_filename, self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, 0.1)
        # for det in detections:
        #     x1, y1, x2, y2 = det['bbox']
        #     class_name = det['class_name']
        #     self.get_logger().info(f"Detected {class_name} at [{x1}, {y1}, {x2}, {y2}]")
            # For demonstration, publish marker at fixed height (e.g., z=0.1)
            # self.marker_pub.publish_object(class_name, (x1+x2)/2000.0, (y1+y2)/2000.0, 0.1)
        # Optionally display annotated image
        # cv2.imshow("Detections", annotated)
        # cv2.waitKey(1)
    
    def create_mesh(self,image,time):
        """
        Create a plane DAE mesh for a detected image with timestamp.
        Saves both .png and .dae files in `self.mesh_folder`.
        """
        # 1️⃣ Generate unique timestamp string
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

        # 2️⃣ Save the image
        image_filename = f"detect_{timestamp_str}.png"
        # image_path = os.path.join(self.mesh_folder, image_filename)
        # cv2.imwrite(image_path, image)

        # 3️⃣ Generate corresponding .dae filename
        dae_filename = f"detect_{timestamp_str}.dae"
        dae_path = os.path.join(self.mesh_folder, dae_filename)

        # 4️⃣ Create minimal DAE content pointing to the PNG
        dae_content = f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><up_axis>Z_UP</up_axis></asset>
  <library_images>
    <image id="texture-image" name="texture-image">
      <init_from>{image_filename}</init_from>
    </image>
  </library_images>
  <library_effects>
    <effect id="material-effect">
      <profile_COMMON>
        <newparam sid="surface"><surface type="2D"><init_from>texture-image</init_from></surface></newparam>
        <newparam sid="sampler"><sampler2D><source>surface</source></sampler2D></newparam>
        <technique sid="common"><phong><diffuse><texture texture="sampler" texcoord="UVSET0"/></diffuse></phong></technique>
      </profile_COMMON>
    </effect>
  </library_effects>
  <library_materials>
    <material id="material" name="material"><instance_effect url="#material-effect"/></material>
  </library_materials>
  <library_geometries>
    <geometry id="plane" name="plane">
      <mesh>
        <source id="positions">
          <float_array id="positions-array" count="12">-0.5 -0.5 0 0.5 -0.5 0 0.5 0.5 0 -0.5 0.5 0</float_array>
          <technique_common><accessor source="#positions-array" count="4" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common>
        </source>
        <source id="uvs">
          <float_array id="uvs-array" count="8">1 0 1 1 0 1 0 0</float_array>
          <technique_common><accessor source="#uvs-array" count="4" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common>
        </source>
        <vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
        <triangles count="2">
          <input semantic="VERTEX" source="#vertices" offset="0"/>
          <input semantic="TEXCOORD" source="#uvs" offset="1" set="0"/>
          <p>0 0 1 1 2 2 0 0 2 2 3 3</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene">
      <node id="PlaneNode"><instance_geometry url="#plane">
        <bind_material>
          <technique_common><instance_material symbol="material" target="#material"/></technique_common>
        </bind_material>
      </instance_geometry></node>
    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>
"""
        # 5️⃣ Save the .dae file
        with open(dae_path, "w") as f:
            f.write(dae_content)

        # self.get_logger().info(f"Created mesh: {dae_path} with texture: {image_path}")
        
        shutil.copy(dae_path, self.install_mesh_folder)
        

        return dae_filename, image_filename


def main():
    rclpy.init()
    # lidar_node = LidarDistanceAnalyzer()
    cam_node = CamReader()
    # executor = MultiThreadedExecutor()
    # executor.add_node(lidar_node)
    # executor.add_node(cam_node)
    try:
        # while rclpy.ok():
        #     # Manually check timers (since this timer is not tied to executor)
        #     node.timer.is_ready()
        # rclpy.spin(lidar_node)
        rclpy.spin(cam_node)
        # executor.spin()

    except KeyboardInterrupt:
        pass
    finally:
        # lidar_node.destroy_node()
        cam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()