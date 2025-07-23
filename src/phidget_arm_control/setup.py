from setuptools import setup

package_name = 'phidget_arm_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    	('share/' + package_name + '/launch', ['launch/phidget_control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ibrahim',
    maintainer_email='your_email@example.com',
    description='ROS2 node for controlling a Phidget-driven robotic arm using MoveIt2 trajectories.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_listener = phidget_arm_control.trajectory_listener:main',
            'phidget_arm_driver = phidget_arm_control.phidget_arm_driver:main',
            'ik_keyboard_control = phidget_arm_control.ik_keyboard_control:main',
            
        ],
    },
)
