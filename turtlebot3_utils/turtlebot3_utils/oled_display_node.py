#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import board
import busio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import math

from rclpy.executors import ExternalShutdownException

class OLEDDisplayNode(Node):
    def __init__(self):
        super().__init__('oled_display_node')
        
        # Initialize I2C and OLED
        i2c = busio.I2C(board.SCL, board.SDA)
        self.oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c, addr=0x3c)
        
        # Clear display
        self.oled.fill(0)
        self.oled.show()
        
        # Create image for drawing
        self.image = Image.new("1", (128, 64))
        self.draw = ImageDraw.Draw(self.image)
        
        # Data storage
        self.battery_voltage = 0.0
        self.battery_percentage = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        
        # Subscribers
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Update timer (5 Hz)
        self.timer = self.create_timer(0.2, self.update_display)
        
        self.get_logger().info('OLED Display Node started')
    
    def battery_callback(self, msg):
        self.battery_voltage = msg.voltage
        self.battery_percentage = msg.percentage
    
    def odom_callback(self, msg):
        self.linear_vel = msg.twist.twist.linear.x
        self.angular_vel = msg.twist.twist.angular.z
    
    def cmd_vel_callback(self, msg):
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z
    
    def update_display(self):
        # Clear image
        self.draw.rectangle((0, 0, 128, 64), outline=0, fill=0)
        
        # Draw text
        self.draw.text((0, 0), "Don's TurtleBot3", fill=255)
        self.draw.text((0, 12), f"Batt: {self.battery_voltage:.2f}V ({self.battery_percentage:.0f}%)", fill=255)
        self.draw.text((0, 24), f"Vel: {self.linear_vel:.2f} m/s", fill=255)
        self.draw.text((0, 36), f"Ang: {self.angular_vel:.2f} r/s", fill=255)
        self.draw.text((0, 48), f"Cmd: {self.cmd_linear:.2f} m/s", fill=255)
        
        # Display image
        self.oled.image(self.image)
        self.oled.show()

def main(args=None):
    rclpy.init(args=args)
    node = OLEDDisplayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.oled.fill(0)
        node.oled.show()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
