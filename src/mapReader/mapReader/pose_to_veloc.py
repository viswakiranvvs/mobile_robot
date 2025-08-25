from time import sleep
from mapReader.joy_controller import JoystickMotorController
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def get_velocity_command(current_pose, target_pose, max_linear=0.2, max_angular=0.5):
    dx = target_pose.position.x - current_pose.position.x
    dy = target_pose.position.y - current_pose.position.y

    distance = math.sqrt(dx**2 + dy**2)
    angle_to_target = math.atan2(dy, dx)

    # Get current robot orientation (assuming quaternion)
    q = current_pose.orientation
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    angle_diff = angle_to_target - yaw
    angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # normalize

    twist = Twist()
    twist.linear.x = min(max_linear, distance)
    twist.angular.z = max(-max_angular, min(max_angular, angle_diff))

    return twist




class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        

        self.path = []  # List of PoseStamped
        self.current_index = 0
        self.tolerance = 0.1  # meters
        self.current_pose=None
        # self.motor_controller = JoystickMotorController()

        # You'll need to subscribe to /odom or TF to update this
    
    def start(self,poses):
        self.path = poses
        self.current_pose_cli = self.create_subscription(Odometry,"/rtabmap/odom",self.odom_update) 
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        if self.current_pose is None or self.current_index >= len(self.path):
            return

        target = self.path[self.current_index].pose
        dist = math.sqrt(
            (target.position.x - self.current_pose.position.x)**2 +
            (target.position.y - self.current_pose.position.y)**2
        )

        if dist < self.tolerance:
            self.current_index += 1
        else:
            twist = get_velocity_command(self.current_pose, target)
            self.cmd_pub.publish(twist)
    
    def odom_update(self, msg: Odometry):
        pose_stamped = PoseStamped()
        if msg:
            pose_stamped.header = msg.header
            pose_stamped.pose = msg.pose.pose
            self.current_pose = pose_stamped
            self.get_logger().debug(f"Updated current pose: x={pose_stamped.pose.position.x:.2f}, y={pose_stamped.pose.position.y:.2f}")

    def quaternion_to_yaw(self, orientation):
        """Convert quaternion to yaw angle (rotation around Y-axis for X-Z plane)"""
        # For X-Z coordinate system, yaw is rotation around Y-axis
        # Standard formula still works since it gives rotation around vertical axis
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        
        # Yaw (rotation around Y-axis) calculation
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def send_motor_commands(self, left_rpm, right_rpm):
        """
        Send RPM commands to motors
        Implement this based on your specific motor controller
        """
        # Example implementation (adjust for your hardware):
        self.motor_controller.send_motor_command(1, left_rpm)   # Front left
        self.motor_controller.send_motor_command(2, right_rpm)  # Front right  
        self.motor_controller.send_motor_command(3, right_rpm)  # Rear right
        self.motor_controller.send_motor_command(4, left_rpm)   # Rear left
        
        self.get_logger().info(f"Motor commands: Left={left_rpm:.1f}, Right={right_rpm:.1f}")

    def navigate_path(self, path_poses, current_pose=None, threshold=0.2, 
                 linear_speed=0.4, angular_speed=1.0, timeout=30.0):
        """
        Follow waypoints using X-Z coordinates (camera coordinate system)
        where X is forward and Z is left/right instead of traditional X-Y
        """
        if current_pose is None:
            self.get_logger().warn("Current pose not yet received!")
            return False
        
        self.current_pose = current_pose
        
        for i, target in enumerate(path_poses):
            start_time = self.get_clock().now()
            reached = False
            self.get_logger().info(f"Navigating to waypoint {i+1}/{len(path_poses)}")
            
            while not reached and (self.get_clock().now() - start_time).nanoseconds < timeout * 1e9:
                # Get latest pose
                if self.current_pose is None:
                    self.get_logger().warn("Lost current pose!")
                    return False
                
                # Current position (using X-Z coordinates from camera)
                curr_x = self.current_pose.position.x
                curr_z = self.current_pose.position.z  # Z represents left/right instead of Y
                
                # Target position (also in X-Z coordinates)
                target_x = target.pose.position.x
                target_z = target.pose.position.z  # Z represents left/right instead of Y
                
                # Calculate distance and angle in X-Z plane
                dx = target_x - curr_x
                dz = target_z - curr_z  # dz instead of dy
                distance = math.hypot(dx, dz)
                
                # Calculate angle in X-Z plane (atan2(dz, dx) instead of atan2(dy, dx))
                angle_to_target = math.atan2(dz, dx)
                
                # Current orientation as yaw
                orientation = self.current_pose.orientation
                yaw = self.quaternion_to_yaw(orientation)
                
                # Calculate angle difference
                angle_diff = self.normalize_angle(angle_to_target - yaw)
                
                # Control logic
                left_rpm, right_rpm = 0, 0
                
                if abs(angle_diff) > 0.2:  # Rotation threshold
                    # Rotate to face target
                    if angle_diff < 0:
                        # Turn right (clockwise)
                        left_rpm = angular_speed
                        right_rpm = -angular_speed
                    else:
                        # Turn left (counter-clockwise)
                        left_rpm = -angular_speed
                        right_rpm = angular_speed
                        
                elif distance > threshold:  # Distance threshold
                    # Move forward
                    left_rpm = linear_speed
                    right_rpm = linear_speed
                else:
                    # Target reached
                    reached = True
                    self.get_logger().info(f"Reached waypoint {i+1} at X={target_x:.2f}, Z={target_z:.2f}")
                    # Stop motors
                    self.send_motor_commands(0, 0)
                    break
                
                # Send motor commands
                self.send_motor_commands(left_rpm, right_rpm)
                
                # Log debug information
                self.get_logger().info(
                    f"Current: X={curr_x:.2f}, Z={curr_z:.2f}, "
                    f"Target: X={target_x:.2f}, Z={target_z:.2f}, "
                    f"Distance: {distance:.2f}, Angle diff: {angle_diff:.2f} rad"
                )

                sleep(1)

                self.current_pose = target.pose
                reached = True
                
                # Sleep to avoid spinning too fast
                # self.rate.sleep()
            
            if not reached:
                self.get_logger().error(f"Timeout reaching waypoint {i+1}")
                self.send_motor_commands(0, 0)  # Stop motors
                return False
            
            # Brief pause after reaching each waypoint
            # rclpy.spin_once(self, timeout_sec=1.0)
        
        self.get_logger().info("All waypoints reached successfully!")
        return True



