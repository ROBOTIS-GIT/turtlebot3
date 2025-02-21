from setuptools import find_packages
from setuptools import setup

package_name = 'turtlebot3_example'

setup(
    name=package_name,
    version='2.2.4',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # To be added
        # ('share/' + package_name + '/launch', glob.glob(os.path.join('launch',
        #                                                 'turtlebot3_interactive_marker.launch.py'))),
        # ('share/' + package_name + '/launch', glob.glob(os.path.join('launch',
        #                                                 'turtlebot3_obstacle_detection.launch.py'))),
        # ('share/' + package_name + '/rviz', glob.glob(os.path.join('rviz',
        #                                               'turtlebot3_interactive_marker.rviz'))),
    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author=['JunyeongJeong', 'Jeonggeun Lim', 'Will Son', 'Ryan Shim', 'Gilbert'],
    author_email=['junyeong4321@gmail.com', 'ljg@robotis.com', 'willson@robotis.com',
                  'jhshim@robotis.com', 'kkjong@robotis.com'],
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    keywords=['ROS', 'ROS2', 'examples', 'rclpy'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Examples of Different TurtleBot3 Usage.'
    ),
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'turtlebot3_interactive_marker = '
            'turtlebot3_example.turtlebot3_interactive_marker.'
            'turtlebot3_interactive_marker:main',
            'turtlebot3_patrol_server = \
                turtlebot3_example.turtlebot3_patrol.turtlebot3_patrol_server:main',
            'turtlebot3_patrol_client = \
                turtlebot3_example.turtlebot3_patrol.turtlebot3_patrol_client:main',
        ],
    },
)
