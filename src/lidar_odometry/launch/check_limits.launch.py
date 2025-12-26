from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='lidar_odometry',
                plugin='lidar_odometry::LidarOdometryNode',
                name='odom_node',
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='lidar_odometry',
                plugin='lidar_odometry::OdometryAnalyzer',
                name='odom_analyzer',
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen',
    )
    return LaunchDescription([container])