from setuptools import find_packages
from setuptools import setup

package_name = 'turtlebot3_example'

setup(
    name=package_name,
    version='2.3.3',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author=['Jeonggeun Lim',
            'Will Son',
            'Ryan Shim',
            'Gilbert',
            'Wonho Yun',
            'Junyeong Jeong',
            'YeonSoo Noh'],
    author_email=['ljg@robotis.com',
                  'willson@robotis.com',
                  'jhshim@robotis.com',
                  'kkjong@robotis.com',
                  'ywh@robotis.com',
                  'junyeong4321@gmail.com',
                  'nys8277@gmail.com'],
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
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot3_interactive_marker = '
            'turtlebot3_example.turtlebot3_interactive_marker.'
            'turtlebot3_interactive_marker:main',

            'turtlebot3_patrol_server = \
                turtlebot3_example.turtlebot3_patrol.turtlebot3_patrol_server:main',
            'turtlebot3_patrol_client = \
                turtlebot3_example.turtlebot3_patrol.turtlebot3_patrol_client:main',

            'turtlebot3_obstacle_detection = '
            'turtlebot3_example.turtlebot3_obstacle_detection.'
            'turtlebot3_obstacle_detection:main',

            'turtlebot3_absolute_move = '
            'turtlebot3_example.turtlebot3_absolute_move.'
            'turtlebot3_absolute_move:main',

            'turtlebot3_relative_move = '
            'turtlebot3_example.turtlebot3_relative_move.'
            'turtlebot3_relative_move:main'
        ],
    },
)
