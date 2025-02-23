from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cybair_6dof_gripper_control_cpp',
            name="teleop",
            executable='teleop_gripper_key',
        )
    ])