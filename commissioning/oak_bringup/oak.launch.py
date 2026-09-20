"""OAK depth and XYZ cloud; shared robot description owns oakd_link mount."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    driver = Path(get_package_share_directory('depthai_ros_driver_v3'))
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(driver / 'launch/driver.launch.py')),
            launch_arguments={'name': 'oak', 'parent_frame': 'oakd_link',
                              'params_file': str(Path(__file__).with_name('oak.yaml')),
                              'use_rviz': 'false'}.items()),
        LoadComposableNodes(target_container='oak_container',
            composable_node_descriptions=[ComposableNode(
                package='depth_image_proc', plugin='depth_image_proc::PointCloudXyzNode',
                name='oak_pointcloud',
                remappings=[('image_rect', '/oak/stereo/image_raw'),
                            ('camera_info', '/oak/stereo/camera_info'),
                            ('points', '/oak/points')],
                extra_arguments=[{'use_intra_process_comms': True}])]),
    ])
