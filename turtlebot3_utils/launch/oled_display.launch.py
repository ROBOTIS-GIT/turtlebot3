from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    oled_display_node = Node(
        package='turtlebot3_utils',
        executable='oled_display_node',
        name='oled_display_node',
        output='screen'
    )
    
    return LaunchDescription([
        oled_display_node
    ])