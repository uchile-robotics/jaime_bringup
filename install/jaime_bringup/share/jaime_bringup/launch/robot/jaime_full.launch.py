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
        )
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
                tablet_pkg, 'launch', 'tablet_launch.py'
            ])
        )
    )
#para el urdf

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command([
                'xacro ',
                PathJoinSubstitution([description_pkg, 'urdf', 'jaime.xacro'])
                
            ])
        }]
    )
        # Timer para lanzar localización después de 5s
    launch_localization = TimerAction(
        period=5.0,
        actions=[localization_node]
    )

    # Timer para lanzar navegación después de 10s
    launch_navigation = TimerAction(
        period=10.0,
        actions=[navigation_node]
    )

    return LaunchDescription([
        display_node,
        basic_node,
        launch_localization,
        launch_navigation,
        robot_state_publisher_node,
    ])
