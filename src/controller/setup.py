from setuptools import find_packages, setup
import glob
import os


package_name = 'controller'


setup(
   name=package_name,
   version='0.0.0',
   packages=find_packages(exclude=['test']),
   data_files=[
       ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
       ('share/' + package_name, ['package.xml']),
       (os.path.join("share/", package_name, "launch/"), glob.glob("launch/*.launch.py")),
       (os.path.join("share/", package_name, "config/"), glob.glob("config/*.yaml")),
   ],
   install_requires=['setuptools'],
   zip_safe=True,
   maintainer='rida2',
   maintainer_email='rida2@todo.todo',
   description='TODO: Package description',
   license='TODO: License declaration',
   tests_require=['pytest'],
   entry_points={
       'console_scripts': [
           'joystick = controller.joystick:main',
           'motor= controller.motor:main',
           'motor_driver = controller.motor_driver:main',
           'keyboard_node = controller.keyboard_node:main',
           'arm_driver = controller.arm_driver:main',
       ],
   },
)
