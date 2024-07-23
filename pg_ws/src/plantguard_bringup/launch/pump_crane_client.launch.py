from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory('plantguard_bringup')

    return LaunchDescription([
        Node(
            package='image_transport',
            executable='republish',
            name='h264_decoder',
            arguments=['h264', 'raw'],
            remappings=[
                ('in/h264', 'image_stream/h264'),
                ('out', 'image_uncompressed'),
            ],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', bringup_dir + '/rviz/plant_guard.rviz'],
            output='screen',
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'zenoh_bridge_dds', 'zenoh_bridge_dds', '-d', '0', '-a',
                 '^rt/image_raw/h264$|^rt/pump_crane/angle$|^rt/pump_crane/movement_dir$|^rt/pump_crane/pump$'],
            output='screen',
        ),
    ])
