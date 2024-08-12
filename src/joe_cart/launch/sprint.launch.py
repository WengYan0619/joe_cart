import os
import xacro
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='joe_cart' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory(package_name), 'worlds', 'joe_joe_mart.world'),
        description='Full path to the world file to load'
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={
                        'world': LaunchConfiguration('world'),
                        'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
                    }.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')

        
    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    # controller_manager = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[controller_params_file],
    #     remappings=[
    #         ("~/robot_description", "/robot_description"),
    #     ],
    # )
    # delayed_controller_manager = TimerAction(period=1.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "-c", "/controller_manager"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "-c", "/controller_manager"],
    )

    # Launch Nav2 Map Server
    map_yaml_path = os.path.join(get_package_share_directory(package_name), 'joe_mart.yaml')
    
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'yaml_filename': 'joe_mart.yaml'}]
    )

    # Launch AMCL
    amcl_params = os.path.join(get_package_share_directory(package_name), 'config', 'amcl_params.yaml')
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params,
                    {'use_sim_time': True},
                    {'set_initial_pose': True},
                    {'initial_pose.x': 3.385},
                    {'initial_pose.y': -1.367}]
    )

    # Delayed start for AMCL lifecycle bringup
    delayed_amcl = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['amcl']}
                ]
            )
        ]
    )

    # Delayed start for map_server lifecycle bringup
    delayed_map_server = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='map_server',
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}
                ]
            )
        ]
    )

    # Launch ROS Bridge Server
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_server',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Launch RViz2 with custom configuration
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'configs', 'sprint.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/zawl/joe_cart/src/joe_cart/config/sprint.rviz'],
        parameters=[{'use_sim_time': True}]
    )

    # Add rosbridge_server to the final LaunchDescription return

    # Make sure to add map_server to the final LaunchDescription return

    \
    # Code for delaying a node (I haven't tested how effective it is)
    # 
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )
    #
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner



    # Launch them all!
    return LaunchDescription([
        world_file_arg,
        rsp,
        joystick,
        twist_mux,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        map_server,
        delayed_map_server,
        amcl,
        delayed_amcl,
        rosbridge_server,
        rviz2
    ])