from datetime import datetime
import math
import shutil
import time
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
# import numpy as np

class CamReader(Node):
    def __init__(self):
        self.yolo = LightweightYolo(model_path="yolov8n.pt", device="cpu", conf=0.35, imgsz=640)
        self.marker_pub = ObjectMarkerPublisher()
        super().__init__('cam_reader_node')
        self.bridge = CvBridge()
        self.mesh_folder = os.path.join(os.getcwd(), "meshes")
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
                    "extinguisher","banana","person","scissors","cup","mug","marker","car"]
        self.object_markers=[]
        # detections, annotated = self.yolo.detect(cv_image, return_image=True)
    
    def odom_callback(self, msg):
        # self.prev_odom = self.odom
        self.odom = msg
    
    import math

    def odom_movement_diff(self):
        if self.prev_odom is None or self.odom is None:
            return 0.2, 0.2  # No movement if we don't have previous odom

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

    
    def cam_callback(self, msg):
        
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        detections, annotated = self.yolo.detect(cv_image, return_image=True)
        yolo_image_msg = self.bridge.cv2_to_imgmsg(annotated)
        yolo_image_msg.header.stamp = self.get_clock().now().to_msg()
        yolo_image_msg.header.frame_id = "camera_yolo_detections"
        self.yolo_image.publish(yolo_image_msg)

        linear_diff, angular_diff = self.odom_movement_diff() or (None, None)
        if linear_diff is not None and angular_diff is not None:
            if linear_diff < 0.3:
                # self.get_logger().info(f"Skipping frame due to low movement: linear {linear_diff:.3f}, angular {angular_diff:.3f}")
                return  # Skip processing this frame
        # elif self.prev_odom is None:
        #     return  # Skip if we don't have odom data yet
        detections.sort(key=lambda d: d['confidence'], reverse=True)
        detected_objects = [det['class_name'] for det in detections if det['class_name'] in self.accepted_detections]
        if detected_objects and self.odom is not None:
            self.prev_odom = self.odom
            dae_filename, _ = self.create_mesh(annotated,self.get_clock().now().to_msg())
            new_marker = {
                "object_name": detected_objects[0],
                "mesh_name": dae_filename,
                "x": self.odom.pose.pose.position.x,
                "y": self.odom.pose.pose.position.y,
                "id": int(time.time() * 1000) % 1000000
            }
            self.object_markers.append(new_marker)
            json_path = os.path.join(self.mesh_folder, "detected_objects.json")
            with open(json_path, "w") as jf:
                json.dump(self.object_markers, jf, indent=4)
            for marker in self.object_markers:
                self.marker_pub.publish_object(marker["id"],marker["object_name"],marker["mesh_name"], marker["x"], marker["y"], 0.1)
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
        image_path = os.path.join(self.mesh_folder, image_filename)
        cv2.imwrite(image_path, image)

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
          <float_array id="uvs-array" count="8">0 0 1 0 1 1 0 1</float_array>
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
        install_mesh_folder = os.path.join(os.getcwd(), "install/bot_controller/share/bot_controller/meshes")
        os.makedirs(install_mesh_folder, exist_ok=True)
        shutil.copy(dae_path, install_mesh_folder)
        shutil.copy(image_path, install_mesh_folder)

        return dae_filename, image_filename


def main():
    rclpy.init()
    node = CamReader()
    try:
        # while rclpy.ok():
        #     # Manually check timers (since this timer is not tied to executor)
        #     node.timer.is_ready()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()