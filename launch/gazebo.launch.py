"""Launch Gazebo Classic with the robot + ros2_control controllers.
Strips XML comments from the URDF to work around a bug in gazebo_ros2_control 0.4.10
where '--' inside comments breaks CLI argument parsing for the controller_manager.
"""
import os
import re
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_arm_sim')
    urdf_file = os.path.join(pkg_share, 'urdf', 'mobile_arm.urdf.xacro')

    # Run xacro to expand the URDF, then strip ALL XML comments.
    # The comment-stripping is critical: gazebo_ros2_control 0.4.10 chokes on '--'
    # appearing inside XML comments when it passes the URDF as a CLI parameter.
    urdf_raw = subprocess.check_output(['xacro', urdf_file]).decode('utf-8')
    urdf_clean = re.sub(r'<!--.*?-->', '', urdf_raw, flags=re.DOTALL)

    robot_description = {
        'robot_description': urdf_clean,
        'use_sim_time': True,
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py',
            ])
        ]),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'mobile_arm', '-z', '0.1'],
        output='screen',
    )

    spawn_jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'])
    spawn_arm = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'])
    spawn_gripper = Node(
        package='controller_manager', executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'])

    return LaunchDescription([
        gazebo, rsp, spawn_robot,
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[spawn_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_arm])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_arm, on_exit=[spawn_gripper])),
    ])
