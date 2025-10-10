#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np
import math
import json
from visualization_msgs.msg import Marker
import os

class DummyMarkerPublisher(Node):
    def __init__(self): 
        super().__init__("object_marker_publisher")
        self.pub = self.create_publisher(Marker, "object_markers", 10)
        # self.timer = Timer(self.get_clock(), 1.0, self.timer_callback)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.mesh_folder = os.path.join(os.getcwd(), "meshes")
        
        self.publish()
        

    
    def timer_callback(self):
        # Example usage: publish a marker for an object named "example_object" at (1.0, 2.0, 0.0)
        # self.publish_object("bus", -1.0, 2.0, 0.1)
        self.publish()

    def publish(self):
        json_path = os.path.join(self.mesh_folder, "detected_objects.json")
        with open(json_path, "r") as f:
            detected_objects = json.load(f)
        for obj in detected_objects:
            id = obj.get("id", 0)
            object_name = obj.get("object_name", "unknown")
            mesh_name = obj.get("mesh_name", "unknown.dae")
            x = obj.get("x", 0.0)
            y = obj.get("y", 0.0)
            # z = obj.get("z", 0.0)
            self.publish_object(id, object_name, mesh_name, x, y)

    def publish_object(self, id,object_name,mesh_name, x, y, z=0.0):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "detected_objects"
        marker.id = id
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = f'package://bot_controller/meshes/{mesh_name}'
        marker.mesh_use_embedded_materials = True 
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        # marker.pose.orientation.y = 1.0
        marker.pose.orientation.w = 0.2
        marker.scale.x = 0.7
        marker.scale.y = 0.7
        marker.scale.z = 0.7
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.pub.publish(marker)
        
        # Text marker
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "detected_objects"
        text_marker.id = id + 1000  # Different ID for text
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = x+0.1
        text_marker.pose.position.y = y+0.1
        text_marker.pose.position.z = 0.3
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.2
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = object_name
        self.pub.publish(text_marker)

def main(args=None):
    rclpy.init(args=args)
    node = DummyMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
