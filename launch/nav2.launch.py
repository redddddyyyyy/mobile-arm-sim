"""Nav2 bringup for mobile_arm_sim.

Starts map_server, AMCL, planner_server, controller_server, behavior_server,
bt_navigator, waypoint_follower and smoother_server, plus a lifecycle_manager
that sequences configure/activate on them in order. Expects the robot,
Gazebo and robot_state_publisher to already be running (this file is
included from autonomous.launch.py after a short delay).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_arm_sim')
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_share, 'maps', 'autonomous_map.yaml')

    # Order matters: map_server before amcl (amcl subscribes to /map);
    # planner/controller/behavior/bt_navigator can come up in any order after.
    lifecycle_nodes = [
        'map_server',
        'amcl',
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'smoother_server',
    ]

    common_params = [params_file, {'use_sim_time': True}]

    return LaunchDescription([
        Node(package='nav2_map_server', executable='map_server',
             name='map_server', output='screen',
             parameters=[params_file,
                         {'use_sim_time': True, 'yaml_filename': map_file}]),
        Node(package='nav2_amcl', executable='amcl',
             name='amcl', output='screen',
             parameters=common_params),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=common_params),
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=common_params),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen',
             parameters=common_params),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             parameters=common_params),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen',
             parameters=common_params),
        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', output='screen',
             parameters=common_params),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{
                 'use_sim_time': True,
                 'autostart': True,
                 'node_names': lifecycle_nodes,
             }]),
    ])
