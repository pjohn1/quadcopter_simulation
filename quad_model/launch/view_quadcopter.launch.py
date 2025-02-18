from launch import LaunchDescription
from launch.actions import GroupAction, TimerAction
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    urdf_quad = get_package_share_directory('quad_model')+'/urdf/quadcopter.urdf'
    urdf_map = get_package_share_directory('quad_model')+'/urdf/map.urdf'
    pcd_file = get_package_share_directory('quad_model')+'/meshes/city.pcd'
    params = get_package_share_directory('quad_model')+"/launch/params.yaml"

    with open(urdf_quad,'r') as quad_urdf:
        quad_desc = quad_urdf.read()

    with open(urdf_map) as f:
        map_desc = f.read()
    return LaunchDescription([
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
            package='tf2_ros',
            executable='static_transform_publisher',
            name='quad_transform',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
            output='screen'
        ),

        Node(
            package='quad_model',
            executable='physics_sim',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'map'],
            output='screen',
            parameters=[params]
        ),

        Node(
            package='quad_model',
            executable='controller_node',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'map'],
            output='screen',
            parameters=[params]
        ),


        Node(
            package='quad_model',
            executable='velocity_controller',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'map'],
            output='screen',
            parameters=[params]
        ),

        Node(
            package='quad_model',
            executable='physics_node',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'map'],
            output='screen',
            parameters=[params]
        ),

        TimerAction(
            period=1.5,
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

        Node(
            package='pcl_ros',
            executable='pcd_to_pointcloud',
            name='pcd_publisher',
            parameters=[{'file_name': pcd_file}],
            output='screen'
        ),

        Node(
            package='path_planning',
            executable='remap_pcd',
            output='screen'
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='path_planning',
                    executable='visualize_free_space',
                    output='screen'
                ),
                Node(
                    package='path_planning',
                    executable='path_plan_main',
                    output='screen'
                ),
                Node(
                    package='path_planning',
                    executable='path_point_publisher',
                    output='screen'
                ),
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
