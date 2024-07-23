from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='listener',
            output='screen',
        ),

        ExecuteProcess(
            # cmd=['ros2', 'run', 'zenoh_bridge_dds', 'zenoh_bridge_dds', '-d', '0', '-l', 'tcp/172.17.0.1:7447', '-a', '^rt/chatter$'],
            cmd=['ros2', 'run', 'zenoh_bridge_dds', 'zenoh_bridge_dds', '-m', 'client', '-e', 'tcp/archeryarena.org:7447', '-d', '0', '-l', 'tcp/172.17.0.1:7447', '-a', '^rt/chatter$'],
            output='screen',
        ),
    ])
