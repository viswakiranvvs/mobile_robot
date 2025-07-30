#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import time

class JoystickMotorController(Node):
    def __init__(self):
        super().__init__('joystick_motor_controller')
        self.serial_conn = None

        try:
            self.serial_conn = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("Serial connected to /dev/ttyACM0")
            time.sleep(2)  # Allow Arduino to reset
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")

        # self.subscription = self.create_subscription(
        #     Joy,
        #     '/joy',
        #     self.joy_callback,
        #     10
        # )
        self.get_logger().info("Joystick subscriber started")

    def joy_callback(self, msg):
        if self.serial_conn is None:
            self.get_logger().warn("Serial not connected. Ignoring joystick input.")
            return

        try:
            linear_axis = msg.axes[2]   # Forward/backward (up/down)
            angular_axis = msg.axes[1]  # Left/right turning (twist)

            linear_rpm = int(linear_axis * 210)
            angular_rpm = int(angular_axis * 210)

            # Calculate wheel RPMs
            # Assuming motors 1 & 3 (Left), 2 & 4 (Right)
            # Left wheels need reverse RPMs due to flipped orientation
            left_rpm = (linear_rpm - angular_rpm)
            right_rpm =  (linear_rpm + angular_rpm)

            self.get_logger().info(
                f"Linear: {linear_rpm}, Angular: {angular_rpm}, "
                f"Left RPM: {left_rpm}, Right RPM: {right_rpm}"
            )

            # Map motor IDs accordingly
            self.send_motor_command(1, left_rpm)   # Rear Left
            self.send_motor_command(2, right_rpm)  # Rear Right
            self.send_motor_command(4, left_rpm)   # Front Left
            self.send_motor_command(3, right_rpm)  # Front Right

        except Exception as e:
            self.get_logger().error(f"Error in joy_callback: {e}")

    def send_motor_command(self, motor_id, rpm):
        try:
            cmd = f"{motor_id}:{rpm}\n"
            self.serial_conn.write(cmd.encode('utf-8'))
            self.get_logger().info(f"Sent to motor {motor_id}: RPM {rpm}")
        except Exception as e:
            self.get_logger().error(f"Failed to send to motor {motor_id}: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = JoystickMotorController()

#     if node.serial_conn is None:
#         node.get_logger().error("Exiting due to serial connection failure.")
#         node.destroy_node()
#         rclpy.shutdown()
#         return

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Shutting down joystick motor controller")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
