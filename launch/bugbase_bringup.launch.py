from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
      get_package_share_directory('bugbase_node'),
      'config',
      'rc_control.yaml'
    )

    odrive_node = Node(
        package="bugbase_node",
        executable="odrive_node",
        name="odrive_node",
        namespace="bugbase_bringup",
        emulate_tty=True,
        parameters=[config],
        remappings=[
            ("cmd_vel", "/cmd_vel")
        ]
    )

    joy_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_node",
        namespace="bugbase_bringup",
        emulate_tty=True
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_node",
        namespace="bugbase_bringup",
        emulate_tty=True,
        parameters=[config],
        remappings=[
            ("cmd_vel", "/cmd_vel")
        ]
    )

    connect_gamepad_node = Node(
        package="bugbase_node",
        executable="connect_gamepad_node",
        name="connect_gamepad_node",
        namespace="bugbase_bringup",
        emulate_tty=True
    )

    ld.add_action(odrive_node)
    ld.add_action(joy_node)
    ld.add_action(teleop_node)
    ld.add_action(connect_gamepad_node)

    return ld