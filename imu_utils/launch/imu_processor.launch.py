from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import ComposableNodeContainer, SetParameter
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    imu_utils_package_dir = get_package_share_directory("imu_utils")
    print(imu_utils_package_dir)
    imu_params = PathJoinSubstitution([
                    imu_utils_package_dir, 'config', 'imu_processor.yaml'
                ])

    log_level = LaunchConfiguration('log_level')
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='DEBUG'
    )

    container_sensor_utils = ComposableNodeContainer(
        name='container_sensor_utils',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        # prefix=["gdbserver localhost:3000"],
        emulate_tty=True,
        composable_node_descriptions=[

            ComposableNode(
                package='imu_utils',
                plugin='imu_utils::ImuProcessor',
                name='imu_processor',
                extra_arguments=[{'use_intra_process_comms': True}],
                parameters=[imu_params],
                remappings=[
                    ('imu_in', LaunchConfiguration("topic_in")),
                    ('imu_out', 'imu_processed'),
                ],
            ),
            ComposableNode(
                package="logging_demo",
                plugin='logging_demo::LoggerConfig',
                name='logger_config',
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ],
        output='both',
    )

    return LaunchDescription([
        DeclareLaunchArgument("topic_in", default_value="/imu/data"),
        SetParameter(name='use_sim_time', value=True),
        declare_log_level,
        container_sensor_utils,
        ])

