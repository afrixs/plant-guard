from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('plantguard_bringup')

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument(
            'raspberry_pi', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # Node(
        #     package='usb_cam',
        #     executable='usb_cam_node_exe',
        #     name='usb_cam',
        #     output='screen',
        #     # prefix='gnome-terminal -- gdb -ex run --args'
        # ),

        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[
                os.path.join(pkg_dir, 'params', 'camera.yaml'),
                {'use_rpicam_encoder': LaunchConfiguration('raspberry_pi')},
            ],
            # prefix='gnome-terminal -- gdb -ex run --args'
        ),

        # Node(
        #     package='pg_pump_crane',
        #     executable='pump_crane_control',
        #     name='pump_crane_control',
        #     namespace='pump_crane',
        #     output='screen',
        # ),

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
            # cmd=['ros2', 'run', 'zenoh_bridge_dds', 'zenoh_bridge_dds', '-d', '23', '-a',
            #      '^rt/image_stream/ffmpeg$|'
            #      '^rt/moisture_sensor/moistures_stream$|^rt/moisture_sensor/config_stamped_stream$|'
            #      '^rt/pump_crane/angle_stream$|^rt/pump_crane/angle_cmd$|^rt/pump_crane/movement_dir_cmd$|^rt/pump_crane/pump_stream$|^rt/pump_crane/pump_cmd$|'
            #      '^rq/control_playbackRequest$|^rr/control_playbackReply$|^rq/edit_bagRequest$|^rr/edit_bagReply$|'
            #      '^rq/enable_livestreamRequest$|^rr/enable_livestreamReply$|^rq/get_bag_listRequest$|^rr/get_bag_listReply$|'
            #      '^rq/edit_jobRequest$|^rr/edit_jobReply$|^rq/get_job_listRequest$|^rr/get_job_listReply$|'
            #      '^rq/edit_deviceRequest$|^rr/edit_deviceReply$|^rq/get_device_listRequest$|^rr/get_device_listReply$|'
            #      '^rt/play_bag/_action/feedback$|^rt/play_bag/_action/status$|'
            #      '^rq/play_bag/_action/cancel_goalRequest$|^rr/play_bag/_action/cancel_goalReply$|'
            #      '^rq/play_bag/_action/get_resultRequest$|^rr/play_bag/_action/get_resultReply$|'
            #      '^rq/play_bag/_action/send_goalRequest$|^rr/play_bag/_action/send_goalReply$|'
            #      '$|'],  # zenoh 0.7.2, LAN/localhost
            # cmd=['ros2', 'run', 'zenoh_bridge_dds', 'zenoh_bridge_dds', '-m', 'client', '-e', 'tcp/archeryarena.org:7447', '-d', '23', '-a',
            #      '^rt/image_stream/ffmpeg$|'
            #      '^rt/moisture_sensor/moistures_stream$|^rt/moisture_sensor/config_stamped_stream$|'
            #      '^rt/pump_crane/angle_stream$|^rt/pump_crane/angle_cmd$|^rt/pump_crane/movement_dir_cmd$|^rt/pump_crane/pump_stream$|^rt/pump_crane/pump_cmd$|'
            #      '^rq/control_playbackRequest$|^rr/control_playbackReply$|^rq/edit_bagRequest$|^rr/edit_bagReply$|'
            #      '^rq/enable_livestreamRequest$|^rr/enable_livestreamReply$|^rq/get_bag_listRequest$|^rr/get_bag_listReply$|'
            #      '^rq/edit_jobRequest$|^rr/edit_jobReply$|^rq/get_job_listRequest$|^rr/get_job_listReply$|'
            #      '^rq/edit_deviceRequest$|^rr/edit_deviceReply$|^rq/get_device_listRequest$|^rr/get_device_listReply$|'
            #      '^rt/play_bag/_action/feedback$|^rt/play_bag/_action/status$|'
            #      '^rq/play_bag/_action/cancel_goalRequest$|^rr/play_bag/_action/cancel_goalReply$|'
            #      '^rq/play_bag/_action/get_resultRequest$|^rr/play_bag/_action/get_resultReply$|'
            #      '^rq/play_bag/_action/send_goalRequest$|^rr/play_bag/_action/send_goalReply$|'
            #      '$|'],  # zenoh 0.7.2, internet
            cmd=['ros2', 'run', 'zenoh_bridge_dds', 'zenoh_bridge_dds', '-d', '23', '-a',
                 'rt/image_stream/ffmpeg|'
                 'rt/moisture_sensor/moistures_stream|rt/moisture_sensor/config_stamped_stream|'
                 'rt/pump_crane/angle_stream|rt/pump_crane/angle_cmd|rt/pump_crane/movement_dir_cmd|rt/pump_crane/pump_stream|rt/pump_crane/pump_cmd|'
                 'rq/control_playbackRequest|rr/control_playbackReply|rq/edit_bagRequest|rr/edit_bagReply|'
                 'rq/enable_livestreamRequest|rr/enable_livestreamReply|rq/get_bag_listRequest|rr/get_bag_listReply|'
                 'rq/edit_jobRequest|rr/edit_jobReply|rq/get_job_listRequest|rr/get_job_listReply|'
                 'rq/edit_deviceRequest|rr/edit_deviceReply|rq/get_device_listRequest|rr/get_device_listReply|'
                 'rt/play_bag/_action/feedback|rt/play_bag/_action/status|'
                 'rq/play_bag/_action/cancel_goalRequest|rr/play_bag/_action/cancel_goalReply|'
                 'rq/play_bag/_action/get_resultRequest|rr/play_bag/_action/get_resultReply|'
                 'rq/play_bag/_action/send_goalRequest|rr/play_bag/_action/send_goalReply'
                 ],  # zenoh 0.5.0, LAN/localhost
            # cmd=['ros2', 'run', 'zenoh_bridge_dds', 'zenoh_bridge_dds', '-m', 'client', '-e', 'tcp/archeryarena.org:7447', '-d', '23', '-a',
            #      'rt/image_stream/ffmpeg|'
            #      'rt/moisture_sensor/moistures_stream|rt/moisture_sensor/config_stamped_stream|'
            #      'rt/pump_crane/angle_stream|rt/pump_crane/angle_cmd|rt/pump_crane/movement_dir_cmd|rt/pump_crane/pump_stream|rt/pump_crane/pump_cmd|'
            #      'rq/control_playbackRequest|rr/control_playbackReply|rq/edit_bagRequest|rr/edit_bagReply|'
            #      'rq/enable_livestreamRequest|rr/enable_livestreamReply|rq/get_bag_listRequest|rr/get_bag_listReply|'
            #      'rq/edit_jobRequest|rr/edit_jobReply|rq/get_job_listRequest|rr/get_job_listReply|'
            #      'rq/edit_deviceRequest|rr/edit_deviceReply|rq/get_device_listRequest|rr/get_device_listReply|'
            #      'rt/play_bag/_action/feedback|rt/play_bag/_action/status|'
            #      'rq/play_bag/_action/cancel_goalRequest|rr/play_bag/_action/cancel_goalReply|'
            #      'rq/play_bag/_action/get_resultRequest|rr/play_bag/_action/get_resultReply|'
            #      'rq/play_bag/_action/send_goalRequest|rr/play_bag/_action/send_goalReply'
            #      ],  # zenoh 0.5.0, internet
            respawn=True,  # try to reconnect if zenoh router is down
            output='screen',
        ),
    ])
