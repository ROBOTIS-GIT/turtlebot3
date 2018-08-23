from setuptools import setup

package_name = 'turtlebot3_teleop'

setup(
    name=package_name,
    version='1.1.0',
    packages=[],
    py_modules=[
        'turtlebot3_teleop_key'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Darby Lim',
    author_email='thlim@robotis.com',
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Provides teleoperation using keyboard for TurtleBot3.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot3_teleop_key = turtlebot3_teleop_key:main'
        ],
    },
)
