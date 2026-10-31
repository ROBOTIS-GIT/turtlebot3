from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_device_arg = DeclareLaunchArgument(
        'video_device', default_value='/dev/video0',
        description='USB camera device')
    width_arg = DeclareLaunchArgument('width', default_value='640')
    height_arg = DeclareLaunchArgument('height', default_value='480')

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_size': [640, 480],
            'pixel_format': 'YUYV',
            'camera_frame_id': 'camera_link',
        }],
        output='screen',
    )

    return LaunchDescription([
        video_device_arg,
        width_arg,
        height_arg,
        camera_node,
    ])