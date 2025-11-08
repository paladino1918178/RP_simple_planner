from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Pubblica la mappa
       
         Node(
            package='simple_planner',
            executable='map_publisher',
            name='map_publisher',
            output='screen'
        ),
        # Lancia RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/gerardo/ws/src/simple_planner/launch/map_config.rviz']
        ),
        # Esempio nodo static_transform_publisher (map -> base_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
        ),
        # Esempio nodo simple_planner_node
        Node(
            package='simple_planner',
            executable='simple_planner_node',
            name='simple_planner',
            output='screen'
        )
    ])

