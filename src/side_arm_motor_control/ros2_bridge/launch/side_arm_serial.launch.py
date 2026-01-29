import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('side_arm_motor_control_bridge')
    default_params = os.path.join(pkg_share, 'config', 'side_arm_params.yaml')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='ESP32 serial device',
    )
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate',
    )
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='YAML params file',
    )

    node = Node(
        package='side_arm_motor_control_bridge',
        executable='serial_bridge',
        name='side_arm_serial_bridge',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
            },
        ],
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        params_arg,
        node,
    ])

