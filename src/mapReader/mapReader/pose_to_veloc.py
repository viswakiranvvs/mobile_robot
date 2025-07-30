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
        self.motor_controller = JoystickMotorController()

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

    def quaternion_to_yaw(self, q):
        # Converts quaternion to yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        # Normalize angle to [-pi, pi]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def navigate_path(self, common_poses, threshold=0.2, linear_speed=180, angular_speed=100):
        """
        Follow waypoints from common_poses using odometry + serial motor commands.
        """
        if self.current_pose is None:
            self.get_logger().warn("Current pose not yet received!")
            return

        for target in common_poses:
            reached = False
            while not reached:
                if self.current_pose is None:
                    continue

                # Current position
                curr_x = self.current_pose.pose.position.x
                curr_y = self.current_pose.pose.position.y

                # Target position
                target_x = target.pose.position.x
                target_y = target.pose.position.y

                # Calculate distance and angle
                dx = target_x - curr_x
                dy = target_y - curr_y
                distance = math.hypot(dx, dy)
                angle_to_target = math.atan2(dy, dx)

                # Current orientation as yaw
                orientation = self.current_pose.pose.orientation
                yaw = self.quaternion_to_yaw(orientation)

                angle_diff = self.normalize_angle(angle_to_target - yaw)

                # Rotate to face target
                if abs(angle_diff) > 0.2:
                    left_rpm = -angular_speed if angle_diff < 0 else angular_speed
                    right_rpm = -left_rpm
                elif distance > threshold:
                    left_rpm = right_rpm = linear_speed
                else:
                    self.motor_controller.send_motor_command(1, 0)
                    self.motor_controller.send_motor_command(2, 0)
                    self.motor_controller.send_motor_command(3, 0)
                    self.motor_controller.send_motor_command(4, 0)
                    reached = True
                    break

                # Send motor RPMs
                self.motor_controller.send_motor_command(1, left_rpm)
                self.motor_controller.send_motor_command(3, right_rpm)
                self.motor_controller.send_motor_command(2, right_rpm)
                self.motor_controller.send_motor_command(4, left_rpm)
        return True

