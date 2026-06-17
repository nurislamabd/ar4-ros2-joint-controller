from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('ar4_joint_controller')

    # Robot description (self-contained plain URDF, read at launch time).
    urdf_path = os.path.join(pkg_share, 'urdf', 'ar4_arm.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    controller_config = os.path.join(pkg_share, 'config', 'arm_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'arm.rviz')

    return LaunchDescription([
        # Publishes link transforms from /joint_states. Handles the gripper
        # <mimic> joint automatically. arm_controller is the only publisher of
        # /joint_states (no joint_state_publisher_gui).
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # Trapezoidal-profile joint controller (the single /joint_states source).
        Node(
            package='ar4_joint_controller',
            executable='arm_controller',
            name='arm_controller',
            parameters=[controller_config],
            output='screen',
        ),

        # Visualization.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
