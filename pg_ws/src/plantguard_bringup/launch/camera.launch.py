from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='h264_decoder',
            arguments=['h264', 'raw'],
            remappings=[
                ('out', 'image_uncompressed'),
            ],
        ),

        # TODO: For debugging purposes (should be launched at the client side instead)
        Node(
            package='image_transport',
            executable='republish',
            name='h264_decoder',
            arguments=['h264', 'raw'],
            remappings=[
                ('in/h264', 'image_raw/h264'),
                ('out', 'image_uncompressed'),
            ],
        )
    ])
