from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_model_file = LaunchConfiguration('robot_model_file')
    x_spawn = LaunchConfiguration('x_spawn')
    y_spawn = LaunchConfiguration('y_spawn')
    yaw_spawn = LaunchConfiguration('yaw_spawn')

    declare_robot_model_file = DeclareLaunchArgument(
        name='robot_model_file',
        default_value='',
        description='Path to robot model file'
    )
    declare_x_spawn = DeclareLaunchArgument(
        name='x_spawn', 
        default_value='0.0',
        description='x position of robot at spawn'
    )
    declare_y_spawn = DeclareLaunchArgument(
        name='y_spawn', 
        default_value='0.0',
        description='y position of robot at spawn'
    )
    declare_yaw_spawn = DeclareLaunchArgument(
        name='yaw_spawn',
        default_value='0.0',
        description='yaw orientation of robot at spawn'
    )

    start_gazebo_ros_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mobile_bot',
            '-file', robot_model_file,
            '-x', x_spawn,
            '-y', y_spawn,
            '-Y', yaw_spawn
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_x_spawn,
        declare_y_spawn,
        declare_yaw_spawn,
        declare_robot_model_file,
        start_gazebo_ros_spawner
    ])