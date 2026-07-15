"""Autonomous pick-and-place scene in the AWS small_house world."""
import os
import re
import subprocess
import tempfile

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

    # gazebo_ros_state is a WORLD plugin — it only works from inside <world>
    # in the SDF, and the AWS file doesn't load it. Its /set_entity_state
    # service is what the magic grasp rides. Patch a copy at launch time
    # instead of vendoring Amazon's whole world file into this repo.
    state_plugin = (
        '    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">\n'
        '      <ros><namespace>/</namespace></ros>\n'
        '      <update_rate>1.0</update_rate>\n'
        '    </plugin>\n'
        '  </world>')
    with open(world_file) as f:
        world_xml = f.read()
    world_file = os.path.join(tempfile.gettempdir(), 'small_house_with_state.world')
    with open(world_file, 'w') as f:
        f.write(world_xml.replace('</world>', state_plugin))

    urdf_raw = subprocess.check_output(['xacro', urdf_file]).decode('utf-8')
    urdf_clean = re.sub(r'<!--.*?-->', '', urdf_raw, flags=re.DOTALL)
    robot_description = {'robot_description': urdf_clean, 'use_sim_time': True}

    # Set here so users don't need to export GAZEBO_MODEL_PATH in every terminal.
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(aws_share, 'models') + ':' + os.environ.get('GAZEBO_MODEL_PATH', ''),
    )

    # gzserver.launch.py doesn't source /usr/share/gazebo/setup.bash, so the OGRE
    # shader lib and Gazebo/* material scripts are unreachable and camera sensors
    # fail with "Failed to initialize scene / Unable to create CameraSensor".
    # LIDAR (CPU ray) survives; RGB cameras do not.
    set_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value='/usr/share/gazebo-11:' + os.environ.get('GAZEBO_RESOURCE_PATH', ''),
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

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mobile_arm_sim'), 'launch', 'nav2.launch.py'
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
        # Open floor on the far side of the living area (2 m clearance on
        # the map). The old spot (-3.0, 1.0) sat in a pocket behind a door
        # gap too tight for the 0.25 m robot radius — first goals kept
        # aborting on the progress checker while DWB inched through it.
        # Keep amcl's initial_pose in nav2_params.yaml in sync with this.
        arguments=['-topic', 'robot_description', '-entity', 'mobile_arm',
                   '-x', '4.5', '-y', '-1.5', '-z', '0.1', '-Y', '3.14159'],
        output='screen',
    )

    spawn_target = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', os.path.join(pkg_share, 'urdf', 'block.sdf'),
                   '-entity', 'target_block',
                   '-x', '-7.6', '-y', '-0.1', '-z', '0.025'],
        output='screen',
    )

    spawn_dist_orange = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', os.path.join(pkg_share, 'urdf', 'distractor_orange.sdf'),
                   '-entity', 'distractor_orange',
                   '-x', '1.4', '-y', '2.4', '-z', '0.025'],
        output='screen',
    )
    spawn_dist_magenta = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', os.path.join(pkg_share, 'urdf', 'distractor_magenta.sdf'),
                   '-entity', 'distractor_magenta',
                   '-x', '-2.2', '-y', '-0.8', '-z', '0.025'],
        output='screen',
    )
    spawn_dist_brown = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        # North of the first search waypoint: the camera picks it up during
        # the spin there, but no leg of the route drives past it. Its old
        # spot (3.6, -1.2) sat right in front of the robot spawn — below
        # the lidar plane, so Nav2 couldn't avoid it and ran it over on the
        # way out.
        arguments=['-file', os.path.join(pkg_share, 'urdf', 'distractor_brown.sdf'),
                   '-entity', 'distractor_brown',
                   '-x', '0.7', '-y', '2.7', '-z', '0.025'],
        output='screen',
    )

    spawn_table = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', os.path.join(pkg_share, 'urdf', 'table.sdf'),
                   '-entity', 'target_table',
                   '-x', '4.0', '-y', '-2.5', '-z', '0.075'],
        output='screen',
    )

    block_detector = Node(
        package='mobile_arm_sim', executable='block_detector.py',
        # Skip ~/.local site-packages: a pip-user numpy 2.x there breaks the
        # numpy-1.x-built cv_bridge that ships with Humble.
        additional_env={'PYTHONNOUSERSITE': '1'},
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
        set_resource_path,
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
        # Nav2 needs the spawned robot's /scan, /odom and TF tree. 15s over
        # the old 10: a cold-started gzserver once wasn't ready yet and the
        # lifecycle manager wedged mid-configure.
        TimerAction(period=15.0, actions=[nav2_launch]),
        # The detector needs the map frame, which only exists once AMCL is
        # up — same 15 s margin as Nav2, or it spends the gap warning about
        # TF it can't have yet.
        TimerAction(period=15.0, actions=[block_detector]),
    ])
