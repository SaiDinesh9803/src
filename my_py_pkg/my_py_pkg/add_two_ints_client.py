#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial

from example_interfaces.srv import AddTwoInts

class AddTwoIntsClientNode(Node): 
    def __init__(self):
        super().__init__("add_two_ints_client") 
        self.call_add_two_ints_server(6, 7)
        self.call_add_two_ints_server(7, 12)
        self.call_add_two_ints_server(65, 890)
        
    def call_add_two_ints_server(self, a, b):
        client_ = self.create_client(AddTwoInts, "add_two_ints")
        while not client_.wait_for_service(1):
            self.get_logger().warn("Waiting for Server Add Two Ints ...")
            
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        future = client_.call_async(request)
        future.add_done_callback(partial(self.callback_add_two_ints, a = a, b = b))
        
    def callback_add_two_ints(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(str(a) + " + " + str(b) + " = " + str(response.sum))
        except Exception as e:
            self.get_logger().error("Service call Failed %r" % (e, ))

def main(args = None):
    rclpy.init(args=args)
    node = AddTwoIntsClientNode()  
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()