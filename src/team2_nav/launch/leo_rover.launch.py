import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch_ros.actions import SetParameter, Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    ld = LaunchDescription()
    pkg_name = 'team2_nav'

    # Use xacro to process the files for simulation
    xacro_file_sim = os.path.join(get_package_share_directory('team2_nav'), 'urdf/leo.urdf')
    robot_description_raw_sim = xacro.process_file(xacro_file_sim).toxml()

    # Launch lidar package
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('rplidar_ros'), 'launch/rplidar_a2m12_launch.py')])
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_raw_sim}],
    )

    # Launch rviz2 with the nav2 configuration
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory(pkg_name), 'rviz', 'nav2.rviz')],
    )

    # Filter imu data
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory(pkg_name), 'config/imu_filter_params.yaml')
        ],
    )

    # Filter odometry data, uses imu data
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory(pkg_name), 'config/localization_params.yaml'),
        ],
    )

    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=False))
    ld.add_action(node_rviz)
    ld.add_action(imu_filter_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(robot_localization_node)
    ld.add_action(rplidar_launch)

    return ld