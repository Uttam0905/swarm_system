#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    robots = ['robot1', 'robot2', 'robot3']

    return LaunchDescription([

        Node(
            package='swarm_command',
            executable='pid_multirobot',
            name='pid_multirobot',
            output='screen',
            parameters=[{
                'robots': robots,
                'control_rate': 10.0
            }]
        ),

        Node(
            package='swarm_command',
            executable='swarm_manager',
            name='swarm_manager',
            output='screen',
            parameters=[{
                'robots': robots,
                'line_offset': 5.0
            }]
        ),
    ])
