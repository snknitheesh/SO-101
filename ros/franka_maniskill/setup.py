from setuptools import find_packages, setup

package_name = 'franka_maniskill'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',['launch/franka_sim.launch.py']),
        ('share/' + package_name + '/urdfs',['urdfs/fr3_franka_hand.urdf'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zozo',
    maintainer_email='snknitheesh@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'franka_agent = franka_maniskill.franka_agent:main',
            'franka = franka_maniskill.franka:main',
        ],
    },
)
