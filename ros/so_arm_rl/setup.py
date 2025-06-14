from setuptools import find_packages, setup

package_name = 'so_arm_rl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/arm.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zozo',
    maintainer_email='snknitheesh@gmail.com',
    description='so 101 arm reinforcement learning package for ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'so101_agent = so_arm_rl.so101_agent:main',
            'so101 = so_arm_rl.so101:main',
        ],
    },
)
