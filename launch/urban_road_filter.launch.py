from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('urban_road_filter')
    
    # Path to the parameters.yaml file
    config_file_path = os.path.join(pkg_dir, 'config', 'parameters.yaml')
    
    # Create the default rviz config path
    default_rviz_config_path = os.path.join(pkg_dir, 'config', 'urban_road_filter.rviz')
    
    # Create launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')  # Changed to true for bag playback
    rviz_config_file = LaunchConfiguration('rviz_config_file', default=default_rviz_config_path)
    
    # Declare the launch arguments  
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',  # Changed to true for bag playback
        description='Use simulation (Gazebo) clock if true')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')
    
    # Start the urban_road_filter node
    urban_road_filter_cmd = Node(
        package='urban_road_filter',
        executable='urban_road_filter',
        name='urban_road_filter',
        output='screen',
        parameters=[config_file_path]
    )
    
    # Start RViz
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Make sure the rviz configuration directory exists
    os.makedirs(os.path.join(pkg_dir, 'config'), exist_ok=True)
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    
    # Add nodes to the launch description
    ld.add_action(urban_road_filter_cmd)
    ld.add_action(rviz_cmd)
    
    return ld