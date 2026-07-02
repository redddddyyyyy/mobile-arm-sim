"""Pick-and-place scene with block farther away to showcase the mobile base."""
import os, re, subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_arm_sim')
    urdf_file = os.path.join(pkg_share, 'urdf', 'mobile_arm.urdf.xacro')

    urdf_raw = subprocess.check_output(['xacro', urdf_file]).decode('utf-8')
    urdf_clean = re.sub(r'<!--.*?-->', '', urdf_raw, flags=re.DOTALL)

    robot_description = {'robot_description': urdf_clean, 'use_sim_time': True}

    # gazebo_ros's gzserver.launch.py does not source /usr/share/gazebo/setup.bash,
    # so OGRE can't find its shader lib and camera sensors fail silently. Set here
    # so /camera/image_raw actually publishes.
    set_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value='/usr/share/gazebo-11:' + os.environ.get('GAZEBO_RESOURCE_PATH', ''),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
    )

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[robot_description], output='screen')

    spawn_robot = Node(package='gazebo_ros', executable='spawn_entity.py',
                       arguments=['-topic', 'robot_description', '-entity', 'mobile_arm', '-z', '0.1'],
                       output='screen')

    # Block at 1.0m straight ahead of robot start position
    spawn_block = Node(package='gazebo_ros', executable='spawn_entity.py',
                       arguments=['-file', os.path.join(pkg_share, 'urdf', 'block.sdf'),
                                  '-entity', 'target_block',
                                  '-x', '1.00', '-y', '0.00', '-z', '0.025'],
                       output='screen')

    # Table positioned where robot ends up after pivoting left + driving 0.4m more
    spawn_table = Node(package='gazebo_ros', executable='spawn_entity.py',
                       arguments=['-file', os.path.join(pkg_share, 'urdf', 'table.sdf'),
                                  '-entity', 'target_table',
                                  '-x', '0.65', '-y', '0.80', '-z', '0.075'],
                       output='screen')

    spawn_jsb = Node(package='controller_manager', executable='spawner',
                     arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'])
    spawn_arm = Node(package='controller_manager', executable='spawner',
                     arguments=['arm_controller', '--controller-manager', '/controller_manager'])
    spawn_gripper = Node(package='controller_manager', executable='spawner',
                         arguments=['gripper_controller', '--controller-manager', '/controller_manager'])

    return LaunchDescription([
        set_resource_path,
        gazebo, rsp, spawn_robot,
        TimerAction(period=3.0, actions=[spawn_block, spawn_table]),
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[spawn_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_arm])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_arm, on_exit=[spawn_gripper])),
    ])
