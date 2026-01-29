#en rviz2, revisar que fixed frame sea map, y en map que durability policy sea transient local. 
#Si no sale el lidar, add -> topic -> scan_raw -> LaserScan
#es posible que la imagen se demore en aparecer


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    bringup_pkg = FindPackageShare('jaime_bringup')
    description_pkg = FindPackageShare('jaime_description')
    tablet_pkg = FindPackageShare('jaime_tablet')

    display_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                description_pkg, 'launch', 'display.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'rviz': 'true'
        }.items()
    )

    basic_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                bringup_pkg, 'launch', 'robot', 'jaime_basic.launch.py'
            ])
        )
    )

    localization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                bringup_pkg, 'launch', 'localization', 'localization_launch.py'
            ])
        )
    )

    navigation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                bringup_pkg, 'launch', 'navigation', 'navigation_launch.py'
            ])
        )
    )

#tablet    
    tablet_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                tablet_pkg, 'tablet_launch.py'
            ])
        )
    )

    return LaunchDescription([
        display_node,
        basic_node,
        localization_node,
        navigation_node,
        tablet_node,
    ])

