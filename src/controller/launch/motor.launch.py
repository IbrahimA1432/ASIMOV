from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([


       # Node action to launch motor_driver with parameters
       Node(
           package='controller', 
           executable='motor_driver', 
           name='motor_driver',
       ),
   ])
