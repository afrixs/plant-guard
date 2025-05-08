from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('plantguard_bringup')
    zenoh_version = LaunchConfiguration('zenoh_version')
    stream_over_internet = LaunchConfiguration('stream_over_internet')

    streamed_topics_0_7_2 = (
        '^rt/image_stream/ffmpeg$|'
        '^rt/moisture_sensor/moistures_stream$|^rt/moisture_sensor/config_stamped_stream$|'
        '^rt/pump_crane/angle_stream$|^rt/pump_crane/angle_cmd$|^rt/pump_crane/movement_dir_cmd$|^rt/pump_crane/pump_stream$|^rt/pump_crane/pump_cmd$|'
        '^rq/control_playbackRequest$|^rr/control_playbackReply$|^rq/edit_bagRequest$|^rr/edit_bagReply$|'
        '^rq/enable_livestreamRequest$|^rr/enable_livestreamReply$|^rq/get_bag_listRequest$|^rr/get_bag_listReply$|'
        '^rq/edit_jobRequest$|^rr/edit_jobReply$|^rq/get_job_listRequest$|^rr/get_job_listReply$|'
        '^rq/edit_deviceRequest$|^rr/edit_deviceReply$|^rq/get_device_listRequest$|^rr/get_device_listReply$|'
        '^rt/play_bag/_action/feedback$|^rt/play_bag/_action/status$|'
        '^rq/play_bag/_action/cancel_goalRequest$|^rr/play_bag/_action/cancel_goalReply$|'
        '^rq/play_bag/_action/get_resultRequest$|^rr/play_bag/_action/get_resultReply$|'
        '^rq/play_bag/_action/send_goalRequest$|^rr/play_bag/_action/send_goalReply$|'
        '^rt/record_bag/_action/feedback$|^rt/record_bag/_action/status$|'
        '^rq/record_bag/_action/cancel_goalRequest$|^rr/record_bag/_action/cancel_goalReply$|'
        '^rq/record_bag/_action/get_resultRequest$|^rr/record_bag/_action/get_resultReply$|'
        '^rq/record_bag/_action/send_goalRequest$|^rr/record_bag/_action/send_goalReply$|'
        '^$'
    )
    streamed_topics = PythonExpression(['"', streamed_topics_0_7_2, '" if "', zenoh_version, '" == "0.7.2" else "',
                                        streamed_topics_0_7_2, '".replace("$|^$", "").replace("^", "").replace("$", "")'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'zenoh_version', default_value='0.5.0',
            description='Zenoh version (0.5.0 or 0.7.2)'),
        DeclareLaunchArgument(
            'stream_over_internet', default_value='true',
            description='Stream topics over internet (server: archeryarena.org) if true'),

        Node(
            package='image_transport_tmp',
            executable='republish',
            name='h264_decoder',
            # arguments=['h264', 'raw'],
            remappings=[
                ('in/ffmpeg', 'image_stream/ffmpeg'),
                ('out', 'image_uncompressed'),
            ],
            parameters=[
                { 'in_transport': 'ffmpeg',
                  'out_transport': 'raw',
                  'qos_overrides./image_stream/ffmpeg.subscription.reliability': 'best_effort',
                },
                os.path.join(pkg_dir, 'params', 'camera.yaml'),
            ],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', pkg_dir + '/rviz/plant_guard.rviz'],
            output='screen',
            # prefix='gnome-terminal -- gdb -ex run --args'
        ),

        ExecuteProcess(
            condition=IfCondition(stream_over_internet),
            cmd=['ros2', 'run', 'zenoh_bridge_dds', 'zenoh_bridge_dds', '-m', 'client', '-e', 'tcp/archeryarena.org:7447', '-d', '0', '-a', streamed_topics],
            respawn=True,  # try to reconnect if zenoh router is down
            output='screen',
        ),
        ExecuteProcess(
            condition=UnlessCondition(stream_over_internet),
            cmd=['ros2', 'run', 'zenoh_bridge_dds', 'zenoh_bridge_dds', '-d', '0', '-a', streamed_topics],  # zenoh 0.5.0, LAN/localhost
            respawn=True,  # try to reconnect if zenoh router is down
            output='screen',
        ),
    ])
