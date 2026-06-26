"""Autonomous pick-and-place scene in the AWS small_house world."""
import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_arm_sim')
    aws_share = get_package_share_directory('aws_robomaker_small_house_world')

    urdf_file = os.path.join(pkg_share, 'urdf', 'mobile_arm.urdf.xacro')
    world_file = os.path.join(aws_share, 'worlds', 'small_house.world')

    urdf_raw = subprocess.check_output(['xacro', urdf_file]).decode('utf-8')
    urdf_clean = re.sub(r'<!--.*?-->', '', urdf_raw, flags=re.DOTALL)
    robot_description = {'robot_description': urdf_clean, 'use_sim_time': True}

    # Set here so users don't need to export GAZEBO_MODEL_PATH in every terminal.
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(aws_share, 'models') + ':' + os.environ.get('GAZEBO_MODEL_PATH', ''),
    )

    # gzserver via gazebo_ros launch (handles ROS plugin paths cleanly).
    # gzclient is the BARE binary, not gzclient.launch.py — the launch-file
    # version injects libgazebo_ros_eol_gui.so which null-derefs a Camera
    # shared_ptr and crashes the window. Bare gzclient skips that plugin.
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzserver.launch.py'])
        ]),
        launch_arguments={'world': world_file}.items(),
    )
    gzclient = ExecuteProcess(cmd=['gzclient'], output='screen')

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'mobile_arm',
                   '-x', '-3.0', '-y', '1.0', '-z', '0.1'],
        output='screen',
    )

    spawn_target = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', os.path.join(pkg_share, 'urdf', 'block.sdf'),
                   '-entity', 'target_block',
                   '-x', '-1.0', '-y', '1.0', '-z', '0.025'],
        output='screen',
    )

    spawn_dist_orange = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', os.path.join(pkg_share, 'urdf', 'distractor_orange.sdf'),
                   '-entity', 'distractor_orange',
                   '-x', '-2.0', '-y', '0.0', '-z', '0.025'],
        output='screen',
    )
    spawn_dist_magenta = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', os.path.join(pkg_share, 'urdf', 'distractor_magenta.sdf'),
                   '-entity', 'distractor_magenta',
                   '-x', '-1.5', '-y', '-1.0', '-z', '0.025'],
        output='screen',
    )
    spawn_dist_brown = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', os.path.join(pkg_share, 'urdf', 'distractor_brown.sdf'),
                   '-entity', 'distractor_brown',
                   '-x', '0.0', '-y', '2.0', '-z', '0.025'],
        output='screen',
    )

    spawn_table = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', os.path.join(pkg_share, 'urdf', 'table.sdf'),
                   '-entity', 'target_table',
                   '-x', '1.5', '-y', '-1.5', '-z', '0.075'],
        output='screen',
    )

    spawn_jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    spawn_arm = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )
    spawn_gripper = Node(
        package='controller_manager', executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
    )

    return LaunchDescription([
        set_model_path,
        gzserver,
        # 3s lets gzserver advertise its master before gzclient tries to connect.
        TimerAction(period=3.0, actions=[gzclient]),
        rsp,
        spawn_robot,
        # 8s gives the AWS world (68 meshes) time to settle before we spawn props.
        TimerAction(period=8.0, actions=[
            spawn_target, spawn_dist_orange, spawn_dist_magenta, spawn_dist_brown, spawn_table,
        ]),
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[spawn_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_arm])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_arm, on_exit=[spawn_gripper])),
    ])
