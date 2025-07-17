from setuptools import setup

package_name = 'fk_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ibrahim',
    maintainer_email='your@email.com',
    description='Forward kinematics test publisher for robotic arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'joint_state_pub = fk_test.joint_state_pub:main',
        'fk_publisher = fk_test.fk_publisher:main',
        'inverse_kinematics_node = fk_test.inverse_kinematics_node:main'
        ],
    },
)
