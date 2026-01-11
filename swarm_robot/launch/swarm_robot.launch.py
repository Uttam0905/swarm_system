from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro



def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('swarm_robot'))
    xacro_file_bot1 = os.path.join(pkg_path,'description','robot1.urdf.xacro')
    robot_description_config_bot1 = xacro.process_file(xacro_file_bot1)

    xacro_file_bot2 = os.path.join(pkg_path,'description','robot2.urdf.xacro')
    robot_description_config_bot2 = xacro.process_file(xacro_file_bot2)

    xacro_file_bot3 = os.path.join(pkg_path,'description','robot3.urdf.xacro')
    robot_description_config_bot3 = xacro.process_file(xacro_file_bot3)

    rsp1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot1',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_config_bot1.toxml()}],
        output='screen',
    )

    rsp2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot2',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_config_bot2.toxml()}],
        output='screen',
    )

    rsp3 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot3',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_config_bot3.toxml()}],
        output='screen',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    spawn_entity1 = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot1/robot_description',
                  '-entity', 'bot1',
                  '-x','0',
                  '-y','0',
                  '-z','0',
                  '-Y','0',],
                    output='screen')
    
    spawn_entity2 = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot2/robot_description',
                  '-entity', 'bot2',
                  '-x','4',
                  '-y','8',
                  '-z','0',
                  '-Y','0',],
                    output='screen')

    spawn_entity3 = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot3/robot_description',
                  '-entity', 'bot3',
                  '-x','-8',
                  '-y','-6',
                  '-z','0',
                  '-Y','0',],
                    output='screen')

    return LaunchDescription([
        gazebo,
        rsp1, spawn_entity1,
        rsp2, spawn_entity2,
        rsp3, spawn_entity3,
    ])
