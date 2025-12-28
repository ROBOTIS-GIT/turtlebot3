from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Include the robot bringup launch file
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_bringup'),
                'launch',
                'robot.launch.py'
            ])
        ])
    )
    
    # Joy node to read joystick
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node'
    )
    
    # Teleop node to convert joy to cmd_vel
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[{
            'axis_linear.x': 1,
            'axis_angular.yaw': 3,
            'publish_stamped_twist': True,
            'scale_linear.x': 0.5,
            'scale_angular.yaw': 0.5,
        }]
    )
    
    return LaunchDescription([
        robot_bringup,
        joy_node,
        teleop_node
    ])