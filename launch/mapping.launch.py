"""SLAM run for the AWS small_house world.

Loads the robot into the empty (no-props) small_house and runs
slam_toolbox in online_async mapping mode. Teleop drive every room
until RViz shows a complete map, then save with:

    mkdir -p src/mobile_arm_sim/maps
    ros2 run nav2_map_server map_saver_cli \\
        -f src/mobile_arm_sim/maps/autonomous_map

Teleop (in a separate terminal):
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
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
    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox_mapping.yaml')

    urdf_raw = subprocess.check_output(['xacro', urdf_file]).decode('utf-8')
    urdf_clean = re.sub(r'<!--.*?-->', '', urdf_raw, flags=re.DOTALL)
    robot_description = {'robot_description': urdf_clean, 'use_sim_time': True}

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(aws_share, 'models') + ':' + os.environ.get('GAZEBO_MODEL_PATH', ''),
    )

    # Same gzserver + bare-gzclient split as autonomous.launch.py — see
    # project_aws_small_house_gotchas memory for why the launch-file gzclient crashes.
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

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
    )

    return LaunchDescription([
        set_model_path,
        gzserver,
        TimerAction(period=3.0, actions=[gzclient]),
        rsp,
        spawn_robot,
        # Start SLAM after robot is up — needs /scan and the odom→base_footprint TF.
        TimerAction(period=8.0, actions=[slam]),
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[spawn_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_arm])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_arm, on_exit=[spawn_gripper])),
    ])
