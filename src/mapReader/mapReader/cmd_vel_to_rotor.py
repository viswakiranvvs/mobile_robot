#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class CmdVelToRotor(Node):
    def __init__(self):
        super().__init__('cmdvel_to_rotor')
        
        # Subscribers
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publishers to rotor references
        self.rotor_pubs = [
            self.create_publisher(Float64, f'/drone0/control/rotor{i}/ref', 10)
            for i in range(4)
        ]
        
        # Baseline hover thrust (adjust experimentally)
        self.hover = 0.6
        self.max_thrust = 1.0
        self.min_thrust = 0.0
        
        # Commanded values
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0
        
        # Update rate
        self.timer = self.create_timer(0.05, self.update_rotors)  # 20 Hz

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vz = msg.linear.z
        self.yaw_rate = msg.angular.z

    def update_rotors(self):
        """
        Simple mixer for quadrotor in X configuration
        R0: front-left, R1: front-right, R2: rear-right, R3: rear-left
        """
        # Simple proportional mapping (for testing)
        # Adjust scaling factors as needed
        k_vz = 0.5      # linear z → thrust
        k_yaw = 0.2     # yaw → differential thrust
        k_xy = 0.0      # lateral x/y ignored for now

        # Compute rotor thrusts
        r0 = self.hover + k_vz*self.vz - k_yaw*self.yaw_rate
        r1 = self.hover + k_vz*self.vz + k_yaw*self.yaw_rate
        r2 = self.hover + k_vz*self.vz + k_yaw*self.yaw_rate
        r3 = self.hover + k_vz*self.vz - k_yaw*self.yaw_rate

        # Clamp values
        speed = 0.0
        r0 = max(self.min_thrust, min(self.max_thrust, r0))*speed
        r1 = max(self.min_thrust, min(self.max_thrust, r1))*speed
        r2 = max(self.min_thrust, min(self.max_thrust, r2))*speed
        r3 = max(self.min_thrust, min(self.max_thrust, r3))*speed

        r0,r1,r2,r3 = speed,speed,speed,speed

        # Publish to rotors
        rotor_values = [r0, r1, r2, r3]
        for pub, val in zip(self.rotor_pubs, rotor_values):
            msg = Float64()
            msg.data = val
            pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToRotor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
