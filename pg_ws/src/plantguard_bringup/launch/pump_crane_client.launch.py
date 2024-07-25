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
            # cmd=['ros2', 'run', 'zenoh_bridge_dds', 'zenoh_bridge_dds', '-d', '0', '-a',
            #      '^rt/image_stream/h264$|^rt/pump_crane/angle$|^rt/pump_crane/angle_cmd$|^rt/pump_crane/movement_dir_cmd$|^rt/pump_crane/pump$|^rt/pump_crane/pump_cmd$|'
            #      '^rq/control_playbackRequest$|^rr/control_playbackReply$|^rq/edit_bagRequest$|^rr/edit_bagReply$|^rq/edit_jobRequest$|^rr/edit_jobReply$|'
            #      '^rq/enable_livestreamRequest$|^rr/enable_livestreamReply$|^rq/get_bag_listRequest$|^rr/get_bag_listReply$|^rq/get_job_listRequest$|^rr/get_job_listReply$'],  # zenoh 0.7.2, LAN/localhost
            # cmd=['ros2', 'run', 'zenoh_bridge_dds', 'zenoh_bridge_dds', '-m', 'client', '-e', 'tcp/archeryarena.org:7447', '-d', '0', '-a',
            #      '^rt/image_stream/h264$|^rt/pump_crane/angle$|^rt/pump_crane/angle_cmd$|^rt/pump_crane/movement_dir_cmd$|^rt/pump_crane/pump$|^rt/pump_crane/pump_cmd$|'
            #      '^rq/control_playbackRequest$|^rr/control_playbackReply$|^rq/edit_bagRequest$|^rr/edit_bagReply$|^rq/edit_jobRequest$|^rr/edit_jobReply$|'
            #      '^rq/enable_livestreamRequest$|^rr/enable_livestreamReply$|^rq/get_bag_listRequest$|^rr/get_bag_listReply$|^rq/get_job_listRequest$|^rr/get_job_listReply$'],  # zenoh 0.7.2, internet
            # cmd=['ros2', 'run', 'zenoh_bridge_dds', 'zenoh_bridge_dds', '-d', '0', '-a',
            #      'rt/image_stream/h264|rt/pump_crane/angle|rt/pump_crane/angle_cmd|rt/pump_crane/movement_dir_cmd|rt/pump_crane/pump|rt/pump_crane/pump_cmd|'
            #      'rq/control_playbackRequest|rr/control_playbackReply|rq/edit_bagRequest|rr/edit_bagReply|rq/edit_jobRequest|rr/edit_jobReply|'
            #      'rq/enable_livestreamRequest|rr/enable_livestreamReply|rq/get_bag_listRequest|rr/get_bag_listReply|rq/get_job_listRequest|rr/get_job_listReply'],  # zenoh 0.5.0, LAN/localhost
            cmd=['ros2', 'run', 'zenoh_bridge_dds', 'zenoh_bridge_dds', '-m', 'client', '-e', 'tcp/archeryarena.org:7447', '-d', '0', '-a',
                 'rt/image_stream/h264|rt/pump_crane/angle|rt/pump_crane/angle_cmd|rt/pump_crane/movement_dir_cmd|rt/pump_crane/pump|rt/pump_crane/pump_cmd|'
                 'rq/control_playbackRequest|rr/control_playbackReply|rq/edit_bagRequest|rr/edit_bagReply|rq/edit_jobRequest|rr/edit_jobReply|'
                 'rq/enable_livestreamRequest|rr/enable_livestreamReply|rq/get_bag_listRequest|rr/get_bag_listReply|rq/get_job_listRequest|rr/get_job_listReply'],  # zenoh 0.5.0, internet
            output='screen',
        ),
    ])
