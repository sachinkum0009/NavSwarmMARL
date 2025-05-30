from setuptools import find_packages, setup
import os

package_name = 'nav_swarm_marl'

launch_files = [os.path.join('launch', f) for f in os.listdir('launch') if f.endswith('.launch.py')]

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=['nav_swarm_marl', 'nav_swarm_marl.*'], exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', launch_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sachin Kumar',
    maintainer_email='sachinkumar.ar97@gmail.com',
    description='This package contains the code for running navigation for swarm of robots using MARL',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laserscan_filter_node = nav_swarm_marl.laserscan_filter_node:main',
            'task_allocator_node = nav_swarm_marl.task_allocator_node:main',
            'scenario_executor_node = nav_swarm_marl.scenario_executor_node:main',
        ],
    },
)
