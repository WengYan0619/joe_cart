import os
import xacro
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node



def generate_launch_description():

    package_name='joe_cart' 

    map_yaml_path = os.path.join(get_package_share_directory(package_name), 'maps', 'block_e_lab.yaml')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'yaml_filename': map_yaml_path}]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'set_initial_pose': True},]
    )

    # Lifecycle management for AMCL and map_server
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]
    )

    # Delayed start for lifecycle manager
    lifecycle_manager_timer = TimerAction(
        period=5.0,
        actions=[lifecycle_manager]
    )

    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_server',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    rviz_config = os.path.join(get_package_share_directory(package_name), 'config', 'remote_pc.rviz')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}]
    )

    nav2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','navigation_launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    joy_params = os.path.join(get_package_share_directory('joe_cart'),'config','joystick.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': False}],
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': False}],
            remappings=[('/cmd_vel','/cmd_vel_joy')]
         )

    start_other_nodes = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=rviz2,
            on_start=[
                map_server, 
                amcl,
                lifecycle_manager_timer,
                rosbridge_server,
                nav2,
                joy_node,
                teleop_node
            ]
        )
    )

    # Launch them all!
    return LaunchDescription([
        rviz2,
        start_other_nodes
    ])
