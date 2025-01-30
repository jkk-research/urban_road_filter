from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rqt_reconfigure',
            executable='rqt_reconfigure',
            name='rqt_conf_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_config',
            arguments=['-d', '$(find urban_road_filter)/config/demo1.rviz'],
            output='screen'
        ),
        Node(
            package='urban_road_filter',
            executable='lidar_road',
            namespace='urban_road_filter',
            name='urban_road_filter',
            output='screen'
        )
    ])
