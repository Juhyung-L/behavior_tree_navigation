import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('simulation_launch')
    
    rviz_config_file_path = os.path.join(pkg_share, 'config', 'urdf_config.rviz')
    ekf_config_file_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    world_file_path = os.path.join(pkg_share, 'worlds', 'house.world')
    robot_model_file_path = os.path.join(pkg_share, 'urdf', 'kiwi_drive', 'mobile_bot.urdf')
    map_yaml_file_path = os.path.join(pkg_share, 'map', 'map.yaml')
    params_file_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # launch configuration variables
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_spawn = LaunchConfiguration('x_spawn')
    y_spawn = LaunchConfiguration('y_spawn')
    yaw_spawn = LaunchConfiguration('yaw_spawn')
    world_file = LaunchConfiguration('world_file')
    robot_model_file = LaunchConfiguration('robot_model_file')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    params_file = LaunchConfiguration('params_file')

    # declare launch arguments
    delcare_rviz_config_file = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=rviz_config_file_path,
        description='Path to RViz config file'
    )
    delcare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_x_spawn = DeclareLaunchArgument(
        name='x_spawn', 
        default_value='-1.0',
        description='x position of robot at spawn'
    )
    declare_y_spawn = DeclareLaunchArgument(
        name='y_spawn', 
        default_value='1.0',
        description='y position of robot at spawn'
    )
    declare_yaw_spawn = DeclareLaunchArgument(
        name='yaw_spawn',
        default_value='0.0',
        description='yaw orientation of robot at spawn'
    )
    declare_world_file = DeclareLaunchArgument(
        name='world_file',
        default_value=world_file_path,
        description='Path to world file'
    )
    declare_robot_model_file = DeclareLaunchArgument(
        name='robot_model_file',
        default_value=robot_model_file_path,
        description='Path to robot model file'
    )
    declare_map_yaml_file = DeclareLaunchArgument(
        name='map_yaml_file',
        default_value=map_yaml_file_path,
        description='Path to map.yaml file for nav2_map_server'
    )
    declare_params_file = DeclareLaunchArgument(
        name='params_file',
        default_value=params_file_path,
        description='Path to nav2 parameters file'
    )

    # declare nodes to launch
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file_path, 
                    {'use_sim_time': use_sim_time}]
    )
    keyboard_telelop_node = Node(
        package='keyboard_teleop',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        output='screen',
        parameters=[params_file_path]
    )

    # launch nav2 localization
    nav2_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file,
            'initial_pose.x': x_spawn,
            'initial_pose.y': y_spawn,
            'initial_pose.yaw': yaw_spawn
        }.items()
    )

    # launch other launch files
    robot_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'world.launch.py')
        ),
        launch_arguments={
            'x_spawn': x_spawn,
            'y_spawn': y_spawn,
            'yaw_spawn': yaw_spawn,
            'use_sim_time': use_sim_time,
            'robot_model_file': robot_model_file,
            'world_file': world_file
        }.items()
    )

    return LaunchDescription([
        delcare_rviz_config_file,
        delcare_use_sim_time,
        declare_x_spawn,
        declare_y_spawn,
        declare_yaw_spawn,
        declare_world_file,
        declare_robot_model_file,
        declare_map_yaml_file,
        declare_params_file,
        
        robot_localization_node,
        rviz_node,
        keyboard_telelop_node,
        nav2_localization_launch,
        robot_world_launch,
    ])
