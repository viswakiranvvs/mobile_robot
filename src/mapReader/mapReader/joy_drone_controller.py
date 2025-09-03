#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
import math

class JoystickController(Node):

    def __init__(self):
        super().__init__('joystick_controller')
        
        # Subscribe to the joystick input
        self.subscription = self.create_subscription(
            Joy,
            'joy', # Standard topic for joystick data
            self.joy_callback,
            10
        )
        
        # Publisher for the goal pose
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose', # The topic your NonlinearController listens to
            10
        )
        
        # Initialize the current target position
        self.current_pose = PoseStamped()
        self.current_pose.pose.position = Point(x=0.0, y=0.0, z=2.0) # Start at 2m altitude
        self.current_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) # Default orientation
        self.current_pose.header.frame_id = 'world'
        
        # Control parameters (adjust these for sensitivity)
        self.linear_speed = 0.5  # meters per second per joystick push
        self.angular_speed = 0.5 # radians per second per joystick push
        self.update_rate = 10  # Hz
        self.last_update = self.get_clock().now()
        
        self.get_logger().info('Joystick Controller Node Started. Waiting for /joy messages...')

    def joy_callback(self, msg):
        # Get the current time
        now = self.get_clock().now()
        
        # Calculate time since last update to make movement frame-rate independent
        dt = (now - self.last_update).nanoseconds / 1e9
        if dt < (1.0 / self.update_rate):
            return # Skip if it's too soon to update
            
        self.last_update = now
        
        # --- Map Joystick Axes to Movements ---
        # AXES: [Left-LR, Left-UD, LT, Right-LR, Right-UD, RT, Cross-LR, Cross-UD]
        # Example for a standard gamepad. You might need to adjust these indices!
        left_stick_lr = msg.axes[0]   # Axis 0: Left Stick Left/Right
        left_stick_ud = msg.axes[1]   # Axis 1: Left Stick Up/Down
        right_stick_lr = msg.axes[2]  # Axis 3: Right Stick Left/Right (Yaw)
        right_stick_ud = msg.axes[3]  # Axis 4: Right Stick Up/Down (Altitude)

        # self.get_logger().info(f'Joystick Axes: {msg.axes}')

        # self.get_logger().info(f'Joystick Input - Left Stick: ({left_stick_lr:.2f}, {left_stick_ud:.2f}), Right Stick: ({right_stick_lr:.2f}, {right_stick_ud:.2f})', throttle_duration_sec=1.0)
        
        # --- Update the target position based on joystick input ---
        # Scale the joystick input by speed and time for smooth control
        self.current_pose.pose.position.x += left_stick_lr * self.linear_speed * dt
        self.current_pose.pose.position.y += left_stick_ud * self.linear_speed * dt
        self.current_pose.pose.position.z += right_stick_ud * self.linear_speed * dt
        
        # --- Update the target yaw (orientation) ---
        current_yaw = 2 * math.atan2(self.current_pose.pose.orientation.z, 
                                     self.current_pose.pose.orientation.w)
        new_yaw = current_yaw + right_stick_lr * self.angular_speed * dt
        
        # Convert the new yaw back to a quaternion
        self.current_pose.pose.orientation.z = math.sin(new_yaw / 2)
        self.current_pose.pose.orientation.w = math.cos(new_yaw / 2)
        
        # --- Publish the new goal pose ---
        self.current_pose.header.stamp = now.to_msg()
        self.goal_publisher.publish(self.current_pose)
        
        # Optional: Log the command for debugging
        pos = self.current_pose.pose.position
        self.get_logger().info(f'Goal: X={pos.x:.2f}, Y={pos.y:.2f}, Z={pos.z:.2f}, Yaw={new_yaw:.2f}', throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    joystick_controller = JoystickController()
    rclpy.spin(joystick_controller)
    joystick_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()