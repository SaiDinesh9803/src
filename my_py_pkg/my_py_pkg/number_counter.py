#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from std_srvs.srv import SetBool

class NumberCounter(Node): 
    def __init__(self):
        super().__init__("number_counter") 
        self.subscriber_ = self.create_subscription(Int64, 'number', self.number_counter, 10)
        self.publisher_ = self.create_publisher(Int64, 'number_count', 10)
        self.service_ = self.create_service(SetBool, 'reset_number_count', self.callback_reset_number_count)
        self.counter_ = 0
        self.get_logger().info('Number Counting has started')
        self.get_logger().info("Reset Number Service has been started")
        
        
    def callback_reset_number_count(self, request, response):
        if request.data:
            self.counter_ = 0
            response.success = True
            response.message = 'Counter has been reset!'
        else:
            response.success = False
            response.message = "Counter has not been reset"
        
        return response

    def number_counter(self, msg):
        self.counter_ += msg.data 
        pub_msg = Int64()
        pub_msg.data = self.counter_
        self.publisher_.publish(pub_msg)
        
        
def main(args = None):
    rclpy.init(args=args)
    node = NumberCounter() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()