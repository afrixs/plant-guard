from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker',
            output='screen',
        ),

        ExecuteProcess(
            # cmd=['ros2', 'run', 'zenoh_bridge_dds', 'zenoh_bridge_dds', '-d', '23', '-a', '^rt/chatter$'],  # zenoh 0.7.2, LAN/localhost
            # cmd=['ros2', 'run', 'zenoh_bridge_dds', 'zenoh_bridge_dds', '-m', 'client', '-e', 'tcp/archeryarena.org:7447', '-d', '23', '-a', '^rt/chatter$'],  # zenoh 0.7.2, internet
            # cmd=['ros2', 'run', 'zenoh_bridge_dds', 'zenoh_bridge_dds', '-d', '23', '-a', 'rt/chatter'],  # zenoh 0.5.0, LAN/localhost
            cmd=['ros2', 'run', 'zenoh_bridge_dds', 'zenoh_bridge_dds', '-m', 'client', '-e', 'tcp/archeryarena.org:7447', '-d', '23', '-a', 'rt/chatter'],  # zenoh 0.5.0, internet
            output='screen',
        ),
    ])
