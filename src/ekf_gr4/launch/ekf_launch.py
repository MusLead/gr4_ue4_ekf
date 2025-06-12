import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the config directory
    ekf_config_path = os.path.join(
        get_package_share_directory('ekf_gr4'),
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_ue4',
            output='screen',
            parameters=[ekf_config_path],  # Use the absolute path
            remappings=[
     		   ('/odometry/filtered', '/odometry/filtered_gr4')
	    ]
        )
    ])

