"""Launch RViz with the robot model and joint state sliders. Pure visualization, no Gazebo."""
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('mobile_arm_sim')

    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'mobile_arm.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'view.rviz'])

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', urdf_file]),
            value_type=str,
        )
    }

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
