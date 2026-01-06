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
        parameters=[PathJoinSubstitution([
            FindPackageShare('turtlebot3_teleop'),
            'config',
            'joy_teleop.yaml'
        ])]
    )
    
    return LaunchDescription([
        robot_bringup,
        joy_node,
        teleop_node
    ])