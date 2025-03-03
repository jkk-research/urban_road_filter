from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('urban_road_filter')

    # Path to the YAML parameters file
    params_file = os.path.join(pkg_share, 'config', 'urban_road_filter_params.yaml')

    # Path to the RViz config file
    rviz_config_file = os.path.join(pkg_share, 'config', 'demo1.rviz')

    return LaunchDescription([
        # RQT Reconfigure GUI (only useful if using dynamic_reconfigure, remove if not needed)
        Node(
            package='rqt_reconfigure',
            executable='rqt_reconfigure',
            name='rqt_conf_gui',
            output='screen'
        ),

        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_config',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),

        # Urban Road Filter Node with parameters loaded from YAML
        Node(
            package='urban_road_filter',
            executable='urban_road_filter',
            namespace='main',
            name='urban_road_filter',
            output='screen',
            parameters=[params_file]  
        )
    ])
