from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_rt_eval',
            namespace='node1',
            executable='rt_cli_node',
            name='cli1',
            arguments=['100000', '100000', '1' , './results_test/multi_srv']

        ),
        Node(
            package='ros2_rt_eval',
            namespace='node2',
            executable='rt_cli_node',
            name='cli2',
            arguments=['100000', '100000', '1' , './results_test/multi_srv']

        ),
        Node(
            package='ros2_rt_eval',
            namespace='node3',
            executable='rt_cli_node',
            name='cli3',
            arguments=['100000', '100000', '1' , './results_test/multi_srv']

        ),
        Node(
            package='ros2_rt_eval',
            namespace='node4',
            executable='rt_cli_node',
            name='cli4',
            arguments=['100000', '100000', '1' , './results_test/multi_srv']

        ),
        Node(
            package='ros2_rt_eval',
            namespace='node5',
            executable='rt_cli_node',
            name='cli5',
            arguments=['100000', '100000', '1' , './results_test/multi_srv']

        ),
        Node(
            package='ros2_rt_eval',
            namespace='node1',
            executable='rt_srv_node',
            name='srv1',
            # arguments=['1']
        ),
        Node(
            package='ros2_rt_eval',
            namespace='node2',
            executable='rt_srv_node',
            name='srv2',
            # arguments=['1']
        ),
        Node(
            package='ros2_rt_eval',
            namespace='node3',
            executable='rt_srv_node',
            name='srv3',
            # arguments=['1']
        ),
        Node(
            package='ros2_rt_eval',
            namespace='node4',
            executable='rt_srv_node',
            name='srv4',
            # arguments=['1']
        ),
        Node(
            package='ros2_rt_eval',
            namespace='node5',
            executable='rt_srv_node',
            name='srv5',
            # arguments=['1']
        ),
        # ?
    ])