from setuptools import find_packages, setup
import os

from glob import glob

package_name = 'nav_swarm_marl'

launch_files = [os.path.join('launch', f) for f in os.listdir('launch') if f.endswith('.launch.py')]
param_files = glob('param/*.yaml')
map_files = glob('map/*')
config_files = glob('config/*')

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=['nav_swarm_marl', 'nav_swarm_marl.*'], exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', launch_files),
        ('share/' + package_name + '/param', param_files),
        ('share/' + package_name + '/map', map_files),
        ('share/' + package_name + '/config', config_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sachin Kumar',
    maintainer_email='sachinkumar.ar97@gmail.com',
    description='This package contains the code for running navigation for swarm of robots using MARL',
    license='MIT',
    tests_require=['pytest'],
    # extra_require={"test": ["pytest"]},
    entry_points={
        'console_scripts': [
            'laserscan_filter_node = nav_swarm_marl.laserscan_filter_node:main',
            'task_allocator_node = nav_swarm_marl.task_allocator_node:main',
            'scenario_executor_node = nav_swarm_marl.scenario_executor_node:main',
            'navigation_server_node = nav_swarm_marl.navigation_server:main',
            'cmd_vel2stamp_node = nav_swarm_marl.cmd_vel2stamp_node:main',
            'laser_republisher_node = nav_swarm_marl.laser_republisher:main',
            'emergency_cmd_node = nav_swarm_marl.emergency_cmd_node:main',
            'test_emergency_cmd_node = nav_swarm_marl.test_emergency_cmd_node:main',
        ],
    },
)
