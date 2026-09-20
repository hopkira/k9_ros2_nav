"""K9 LD06 commissioning: raw scans only; robot description owns sensor TF."""
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='ld06_container', namespace='',
            package='rclcpp_components', executable='component_container_isolated',
            output='screen',
            composable_node_descriptions=[ComposableNode(
                package='ldlidar_component', plugin='ldlidar::LdLidarComponent',
                name='ldlidar_node',
                parameters=[str(Path(__file__).with_name('ld06.yaml'))],
                remappings=[('/ldlidar_node/scan', '/scan_raw')],
                extra_arguments=[{'use_intra_process_comms': True}],
            )],
        ),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='ld06_lifecycle_manager', output='screen',
             parameters=[{'autostart': True, 'node_names': ['ldlidar_node'],
                          'bond_timeout': 5.0}]),
    ])
