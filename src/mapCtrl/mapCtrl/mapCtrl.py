import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty

class ButtonServiceCaller(Node):
    def __init__(self):
        super().__init__('button_service_caller')
        self.joySubscription = self.create_subscription(Joy,'',self.joyCallback,10)
        self.mapClient = self.create_client(Empty,'/reset_odometry')
         # Wait for service to be available
        while not self.mapClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /reset_odometry service...')

        self.get_logger().info('Ready to listen for button press.')
    
    def joy_callback(self, msg: Joy):
        if msg.buttons[0] == 1:
            self.get_logger().info('Button pressed! Sending service request...')
            self.call_service()

    def call_service(self):
        request = Empty.Request()
        future = self.mapClient.call_async(request)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Service call succeeded.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)

if __name__ == '__main__':
    main()
