from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    num_open_doors_arg = DeclareLaunchArgument(
        'num_open_doors', default_value='10',
        description='Numero di porte aperte contemporaneamente'
    )

    num_open_doors = LaunchConfiguration('num_open_doors')

    return LaunchDescription([
        num_open_doors_arg,

        Node(
            package='simple_planner',
            executable='map_publisher',
            name='map_publisher',
            output='screen',
            parameters=[{'num_open_doors': num_open_doors}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/gerardo/ws/src/simple_planner/launch/map_config.rviz']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
        ),

        Node(
            package='simple_planner',
            executable='simple_planner_node',
            name='simple_planner',
            output='screen'
        )
    ])
