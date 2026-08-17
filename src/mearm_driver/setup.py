from setuptools import find_packages, setup

package_name = 'mearm_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/servo_config.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RTK2026 Team',
    maintainer_email='kamilisxakof@gmail.com',
    description='MeArm robotic arm driver for ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mearm_servo_controller = mearm_driver.servo_controller:main',
            'mearm_manual_controller = mearm_driver.manual_controller:main',
            'mearm_joint_state_pub = mearm_driver.joint_state_publisher:main',
        ],
    },
)
