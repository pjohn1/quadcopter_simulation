from launch import LaunchDescription
from launch.actions import GroupAction, TimerAction
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    urdf_quad = '/mnt/c/Desktop/quadctrl/install/quad_model/share/quad_model/urdf/quadcopter.urdf'
    urdf_map = '/mnt/c/Desktop/quadctrl/install/quad_model/share/quad_model/urdf/map.urdf'
    with open(urdf_quad,'r') as quad_urdf:
        quad_desc = quad_urdf.read()

    with open(urdf_map) as f:
        map_desc = f.read()

    return LaunchDescription([

        # Launch the robot_state_publisher

        # Map robot_state_publisher
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('quad_model'), 'params.yaml'
            ]),
            description='params.yaml'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_transformer',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'map'],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='quad_transform',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'map'],
            output='screen'
        ),

        Node(
            package='quad_model',
            executable='physics_sim',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'map'],
            parameters = [LaunchConfiguration('params_file')],
            output='screen'
        ),

        Node(
            package='quad_model',
            executable='controller_node',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'map'],
            parameters = [LaunchConfiguration('params_file')],
            output='screen'
        ),


        Node(
            package='quad_model',
            executable='velocity_converter',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'map'],
            output='screen'
        ),

        Node(
            package='quad_model',
            executable='physics_node',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'map'],
            output='screen'
        ),

        TimerAction(
            period=1.0,
            actions=[
                GroupAction([
                    PushRosNamespace('robot_map'),
                    Node(
                        package='robot_state_publisher',
                        executable='robot_state_publisher',
                        name='robot_state_publisher',
                        parameters=[{'robot_description': map_desc}],
                        output='screen',
                        arguments=['--ros-args', '--remap', 'robot_description:=/robot_map/robot_description']
                    )
                ]),
                GroupAction([
                    PushRosNamespace('quadcopter'),
                    Node(
                        package='robot_state_publisher',
                        executable='robot_state_publisher',
                        name='robot_state_publisher',
                        parameters=[{'robot_description': quad_desc}],
                        output='screen',
                        arguments=['--ros-args', '--remap', 'robot_description:=/quadcopter/robot_description']
                    )
                ]),
            ],
        ),
        # Launch RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),

        # Launch the map_server with the map YAML file
    ])
