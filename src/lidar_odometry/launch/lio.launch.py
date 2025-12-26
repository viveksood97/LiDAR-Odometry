from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


PACKAGE_NAME = 'lidar_odometry'

def generate_launch_description():
    csv_filepath = LaunchConfiguration('csv_filepath')
    declare_csv_filepath_cmd = DeclareLaunchArgument(
        'csv_filepath',
        description='CSV file path for trajectory data'
    )
    container = ComposableNodeContainer(
        name=PACKAGE_NAME,
        namespace='',
        package='rclcpp_components',
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package='lidar_odometry',
                plugin='lidar_odometry::LidarOdometryNode',
                name='odom_node',
                parameters=[{'use_sim_time': True}],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='lidar_odometry',
                plugin='lidar_odometry::OdometryAnalyzer',
                name='odom_analyzer',
                parameters=[
                    {'use_sim_time': True},
                    {'csv_filepath': csv_filepath}
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen',
    )
    ld = LaunchDescription()
    ld.add_action(declare_csv_filepath_cmd)
    ld.add_action(container)
    return ld