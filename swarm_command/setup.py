from setuptools import find_packages, setup

package_name = 'swarm_command'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/swarm_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='uttam',
    maintainer_email='uttam@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'pid_position_controller = swarm_command.pid_position_controller:main',
            'go_to_pose_publisher = swarm_command.go_to_pose_publisher:main',
            'forward_cmd_vel_multi = swarm_command.forward_cmd_vel_multi:main',
            'swarm_manager = swarm_command.swarm_manager:main',
            'pid_multirobot = swarm_command.pid_multirobot:main',
        ],
    },
)
