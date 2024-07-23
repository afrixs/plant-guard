from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('plantguard_bringup')

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            # prefix='gnome-terminal -- gdb -ex run --args'
        ),

        Node(
            package='pg_pump_crane',
            executable='pump_crane_control',
            name='pump_crane_control',
            namespace='pump_crane',
            output='screen',
        ),

        Node(
            package='bagtube_server',
            executable='bagtube_server',
            name='bagtube_server',
            parameters=[
                os.path.join(pkg_dir, 'params', 'bagtube.yaml'),
            ],
            output='screen',
        ),

        Node(
            package='pg_job_management',
            executable='job_server',
            name='job_server',
            output='screen',
        ),

        # ros2 run zenoh_bridge_dds zenoh_bridge_dds -d 23
        # Node(
        #     package='zenoh_bridge_dds',
        #     executable='zenoh_bridge_dds',
        #     name='zenoh_bridge_dds',
        #     arguments=['-d', '23'],
        #     output='screen',
        # ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'zenoh_bridge_dds', 'zenoh_bridge_dds', '-d', '23', '-a',
                 '^rt/image_raw/h264$|^rt/pump_crane/angle$|^rt/pump_crane/movement_dir_cmd$|^rt/pump_crane/pump$|^rt/pump_crane/pump_cmd$'],
            output='screen',
        ),
    ])
