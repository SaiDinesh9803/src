#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import BatteryState
from my_robot_interfaces.msg import LedMessage

class LedPanelNode(Node): 
    def __init__(self):
        super().__init__("led_panel_node") 
        self.server_ = self.create_service(BatteryState, "set_led", self.set_led_status)
        self.get_logger().info("Led Service has been started!")
        self.current_led_state = False
        self.led_states = [False, False, self.current_led_state]
        
        self.publisher_ = self.create_publisher(LedMessage, 'led_states', 10)
        self.timer_ = self.create_timer(1, self.publish_led_states)

    def set_led_status(self, request, response):
        response.success = True
        self.current_led_state = request.state
        self.led_states = [False, False, self.current_led_state]
        self.get_logger().info("The Led " + str(request.led_number) + " is  " + str(request.state))
        return response
    
    def publish_led_states(self):
        msg = LedMessage()
        msg.led_states = self.led_states
        self.publisher_.publish(msg)

def main(args = None):
    rclpy.init(args=args)
    node = LedPanelNode()  
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()