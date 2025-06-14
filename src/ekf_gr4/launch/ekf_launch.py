import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the config directory
    ekf_config_path = os.path.join(
        get_package_share_directory('ekf_gr4'),
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_ue4',
            output='screen',
            parameters=[ekf_config_path,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}],  # Use the absolute path
            remappings=[
     		   ('/odometry/filtered', '/odometry/filtered_gr4')
            ]
        ),
        Node(
            package='ekf_gr4',      # Change to your actual package name
            executable='recorder',  # Matches what you defined in CMakeLists.txt
            name='trajectory_recorder',
            output='screen'
        )
    ])

