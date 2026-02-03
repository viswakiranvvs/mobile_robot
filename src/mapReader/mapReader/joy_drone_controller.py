#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json

class JoystickController(Node):

    def __init__(self):
        super().__init__('joystick_controller')
        self.bridge = CvBridge()
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

        self.manipulator_publisher = self.create_publisher(
            PoseStamped,
            '/manipulator_goal', # The topic your NonlinearController listens to
            10
        )

        self.camera_publisher = self.create_publisher(
            PoseStamped,
            '/camera_goal', # The topic your NonlinearController listens to
            10
        )

        self.grip_publisher = self.create_publisher(
            PoseStamped,
            '/grip_pose', # The topic your NonlinearController listens to
            10
        )

        self.manipulatorCamera = self.create_subscription(
            Image,
            '/camera_rgb',
            self.cam_callback,
            10
        )

        self.manipulator_pose = PoseStamped()
        self.manipulator_pose.pose.position = Point(x=0.0, y=0.0, z=0.0) # Start at 2m altitude
        self.manipulator_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=0.0) # Default orientation
        self.manipulator_pose.header.frame_id = 'world'

        self.camera_pose = PoseStamped()
        self.camera_pose.pose.position = Point(x=0.0, y=0.0, z=0.0) # Start at 2m altitude
        self.camera_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=0.0) # Default orientation
        self.camera_pose.header.frame_id = 'world'

        # Initialize the current target position
        self.current_pose = PoseStamped()
        self.current_pose.pose.position = Point(x=0.0, y=0.0, z=2.0) # Start at 2m altitude
        self.current_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.5, w=0.5) # Default orientation
        self.current_pose.header.frame_id = 'world'

        self.grip_pose = PoseStamped()
        self.grip_pose.pose.position = Point(x=0.0, y=0.0, z=60.0) # Start at 2m altitude
        self.grip_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.5, w=0.5) # Default orientation
        self.grip_pose.header.frame_id = 'world'
        
        # Control parameters (adjust these for sensitivity)
        self.linear_speed = 0.5  # meters per second per joystick push
        self.angular_speed = 0.5 # radians per second per joystick push
        self.update_rate = 10  # Hz
        self.last_update = self.get_clock().now()
        self.grip_closed = False
        self.logging = False
        self.currentImage = None
        self.event_log = []
        self.get_logger().info('Joystick Controller Node Started. Waiting for /joy messages...')
        self.prev_buttons = None
    
    def cam_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.currentImage = cv_image

    def joy_callback(self, msg):
        # Get the current time
        now = self.get_clock().now()
        if not self.prev_buttons:
            self.prev_buttons = list(msg.buttons)
            return
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
        # self.get_logger().info(f'Joystick Buttons: {msg.buttons}')
        manip_up = msg.buttons[6] 
        manip_down = msg.buttons[8]
        current_manip_z = self.manipulator_pose.pose.position.z
        if manip_up == 1:
            self.manipulator_pose.pose.position.z = max(current_manip_z + 0.01,0.55)
        if manip_down == 1:
            self.manipulator_pose.pose.position.z = min(current_manip_z - 0.01,-0.07)
        self.manipulator_pose.header.stamp = now.to_msg()
        self.manipulator_publisher.publish(self.manipulator_pose)

        cam_up = msg.buttons[7] 
        cam_down = msg.buttons[9]
        if cam_up == 1:
            self.camera_pose.pose.position.z += 1
        if cam_down == 1:
            self.camera_pose.pose.position.z -= 1
        self.camera_pose.header.stamp = now.to_msg()
        self.camera_publisher.publish(self.camera_pose)

        startLog = msg.buttons[1]
        if startLog==1:
            if self.logging==False:
                self.logging=True
                self.event_log = []
                self.get_logger().info('Started logging joystick events.')
            elif self.logging==True:
                self.logging=False
                self.get_logger().info('Stopped logging joystick events.')
                logs = {
                    "events": self.event_log
                }
                with open('joystick_event_log.json', 'w') as f:
                    json.dump(logs, f, indent=4)
                self.get_logger().info('Saved joystick event log to joystick_event_log.json.')
                self.event_log=[]


        grip = msg.buttons[0]

        if grip == 1 and self.prev_buttons[0] == 0:
            if self.grip_closed:
                self.grip_pose.pose.position.z = 60.0
                self.grip_closed = False
                self.log_event(
                    now,
                    event_type="gripper",
                    activity="open"
                )
            else:
                self.grip_pose.pose.position.z = -60.0
                self.grip_closed = True
                self.log_event(
                    now,
                    event_type="gripper",
                    activity="close"
                )
        #     self.grip_pose.pose.position.z = 55.0
        # else:
        #     self.grip_pose.pose.position.z = 60.0
        self.grip_pose.header.stamp = now.to_msg()
        self.grip_publisher.publish(self.grip_pose)

        self.get_logger().info(f'Manipulator Z: {self.manipulator_pose.pose.position.z:.2f}, Camera Z: {self.camera_pose.pose.position.z:.2f}', throttle_duration_sec=1.0)

        # self.get_logger().info(f'Joystick Input - Left Stick: ({left_stick_lr:.2f}, {left_stick_ud:.2f}), Right Stick: ({right_stick_lr:.2f}, {right_stick_ud:.2f})', throttle_duration_sec=1.0)
        
        # --- Update the target position based on joystick input ---
        # Scale the joystick input by speed and time for smooth control
        # self.current_pose.pose.position.x += left_stick_lr * self.linear_speed * dt
        # self.current_pose.pose.position.y += left_stick_ud * self.linear_speed * dt
        self.current_pose.pose.position.z += right_stick_ud * self.linear_speed * dt
        
        # --- Update the target yaw (orientation) ---
        current_yaw = 2.0 * math.atan2(self.current_pose.pose.orientation.z, 
                                     self.current_pose.pose.orientation.w)
        
        # Joystick inputs in BODY frame
        vx_body = left_stick_ud * self.linear_speed    # forward/back
        vy_body = left_stick_lr * self.linear_speed    # left/right
        vz = right_stick_ud * self.linear_speed
        dyaw = right_stick_lr * self.angular_speed
        # Rotate into WORLD frame using yaw
        vx_world = vx_body * math.cos(current_yaw) - vy_body * math.sin(current_yaw)
        vy_world = vx_body * math.sin(current_yaw) + vy_body * math.cos(current_yaw)

        # Apply movement
        self.current_pose.pose.position.x += vx_world * dt
        self.current_pose.pose.position.y += vy_world * dt

        new_yaw = current_yaw + right_stick_lr * self.angular_speed * dt
        
        # Convert the new yaw back to a quaternion
        self.current_pose.pose.orientation.z = math.sin(new_yaw / 2.0)
        self.current_pose.pose.orientation.w = math.cos(new_yaw / 2.0)
        
        # --- Publish the new goal pose ---
        self.current_pose.header.stamp = now.to_msg()
        self.goal_publisher.publish(self.current_pose)
        
        # Optional: Log the command for debugging
        pos = self.current_pose.pose.position
        self.get_logger().info(f'Goal: X={pos.x:.2f}, Y={pos.y:.2f}, Z={pos.z:.2f}, Yaw={new_yaw:.2f}', throttle_duration_sec=1.0)
        dx = vx_world * dt
        dy = vy_world * dt
        dz = vz * dt
        yaw_delta = dyaw * dt
        if abs(dx) > 1e-4 or abs(dy) > 1e-4 or abs(dz) > 1e-4:
            self.log_event(
                now,
                event_type="drone",
                activity="move",
                dx=dx,
                dy=dy,
                dz=dz
            )
        if abs(yaw_delta) > 1e-4:
            self.log_event(
                now,
                event_type="drone",
                activity="yaw",
                dyaw=yaw_delta
            )
        self.prev_buttons = list(msg.buttons)

    def log_event(self, now, event_type, activity, dx=0.0, dy=0.0, dz=0.0, dyaw=0.0):
        self.event_log.append({
            "timestamp": now.nanoseconds / 1e9,
            "type": event_type,
            "activity": activity,
            "displacement": {
                "x": dx,
                "y": dy,
                "z": dz,
                "yaw": dyaw
            }
        })

def main(args=None):
    rclpy.init(args=args)
    joystick_controller = JoystickController()
    rclpy.spin(joystick_controller)
    joystick_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()