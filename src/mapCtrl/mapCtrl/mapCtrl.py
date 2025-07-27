import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty

class ButtonServiceCaller(Node):
    def __init__(self):
        super().__init__('button_service_caller')
        self.joySubscription = self.create_subscription(Joy,'/joy',self.joy_callback,1)
        self.reset_odom = self.create_client(Empty,'/rtabmap/reset_odom')
        self.reset_rtab = self.create_client(Empty,'/rtabmap/rtabmap/reset')
        self.resume_rtab = self.create_client(Empty,'/rtabmap/rtabmap/resume')
        self.pause_rtab = self.create_client(Empty,'/rtabmap/rtabmap/pause')
        self.trigger_new_rtab = self.create_client(Empty,'/rtabmap/rtabmap/trigger_new_map')
        self.paused=False
        self.prev_buttons = []

         # Wait for service to be available
        # while not self.reset_odom.wait_for_service(timeout_sec=1.0) and not self.reset_rtab.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for /reset_odometry service...')

        self.get_logger().info('Ready to listen for button press.')
    
    def joy_callback(self, msg: Joy):
        # Initialize prev_buttons if first time
        if not self.prev_buttons:
            self.prev_buttons = [0] * len(msg.buttons)

        # Loop through buttons you care about
        # Button A (index 0) – reset
        if msg.buttons[0] == 1 and self.prev_buttons[0] == 0:
            self.get_logger().info('Button A pressed! Sending service request...')
            self.call_service(self.reset_rtab)
            self.call_service(self.reset_odom)
            self.get_logger().info('Resetting Odometry and Map....')

        # Button B (index 1) – pause/resume
        if msg.buttons[1] == 1 and self.prev_buttons[1] == 0:
            self.get_logger().info('Button B pressed! Sending service request...')
            if self.paused:
                self.call_service(self.resume_rtab)
                self.paused = False
                self.get_logger().info('Resuming Map build....')
            else:
                self.call_service(self.pause_rtab)
                self.paused = True
                self.get_logger().info('Pausing Map build....')

        # Button LB (index 4) – trigger new map
        if msg.buttons[4] == 1 and self.prev_buttons[4] == 0:
            self.get_logger().info('Button Y pressed! Sending service request...')
            self.call_service(self.trigger_new_rtab)
            self.get_logger().info('Triggering new map...')

        # Update previous button states
        self.prev_buttons = msg.buttons

    def call_service(self,service):
        request = Empty.Request()
        future = service.call_async(request)
        future.add_done_callback(self.service_response_callback)
        # request = Empty.Request()
        # future = self.reset_rtab.call_async(request)
        # future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Service call succeeded.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    btCtrl = ButtonServiceCaller()
    rclpy.spin(btCtrl)
    btCtrl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
