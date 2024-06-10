#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import BatteryState
from functools import partial

class BatteryStatePublisherNode(Node):
    def __init__(self):
        super().__init__("battery_state_publisher")
        self.counter_ = 0
        self.timer_ = self.create_timer(1, self.increment_counter)    
        self.current_led_state = False
        self.set_battery_state(3, self.current_led_state)
        
    def increment_counter(self):
        
        self.counter_ = self.counter_ + 1
        
        if self.counter_ == 4 and self.current_led_state == False:
            self.set_battery_state(3, True)
            self.counter_ = 0
            
        if self.counter_ == 6 and self.current_led_state == True:
            self.set_battery_state(3, False)
            self.counter_ = 0
        
    def set_battery_state(self, led_number, led_state):
        self.client_ = self.create_client(BatteryState, "set_led")   
        while not self.client_.wait_for_service(0.5):
            self.get_logger().info("Waiting for Service...")
            
        request = BatteryState.Request()
        
        request.led_number = led_number
        request.state = led_state
        self.current_led_state = led_state
        
        future = self.client_.call_async(request)
        future.add_done_callback(partial(self.callback_battery_service, led_state = led_state))
        
            
        
    def callback_battery_service(self, future, led_state):
        try:
            self.get_logger().info("Battery State Has been successfully set to " + str(led_state) )
        except Exception as e:
            self.get_logger().error("Service call Failed %r" % (e, ))
            

def main(args = None):
    rclpy.init(args = args)
    node = BatteryStatePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    