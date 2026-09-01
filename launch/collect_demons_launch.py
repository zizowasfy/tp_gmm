import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    task_arg = DeclareLaunchArgument(
        'task',
        default_value='franka_pick_cube',
        description='Task name to record demonstrations for (e.g. franka_pick_cube, pick, place)'
    )

    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='franka_panda',
        description='Robot model: franka_panda, kinova_gen3, ur10'
    )

    num_demons_arg = DeclareLaunchArgument(
        'num_demons',
        default_value='10',
        description='Number of demonstrations to record'
    )

    auto_arg = DeclareLaunchArgument(
        'auto',
        default_value='false',
        description='Automatically save valid demonstrations without prompting'
    )

    gazebo_arg = DeclareLaunchArgument(
        'gazebo',
        default_value='true',
        description='Update Gazebo simulation object poses (true/false)'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='ROS namespace'
    )

    save_dir_arg = DeclareLaunchArgument(
        'save_dir',
        default_value='',
        description='Custom root directory to save demonstrations to'
    )

    # Launch Demonstration Collector Node
    # Note: collect_trajectories.py reads CLI args like --task, --robot, --num-demons
    collector_node = Node(
        package='tp_gmm',
        executable='collect_trajectories.py',
        name='demonstration_collector',
        output='screen',
        arguments=[
            '--task', LaunchConfiguration('task'),
            '--robot', LaunchConfiguration('robot'),
            '--num-demons', LaunchConfiguration('num_demons'),
            '--namespace', LaunchConfiguration('namespace'),
        ],
        prefix=['xterm -e'] if False else [],  # Can run directly in screen output
    )

    return LaunchDescription([
        task_arg,
        robot_arg,
        num_demons_arg,
        auto_arg,
        gazebo_arg,
        namespace_arg,
        save_dir_arg,
        collector_node
    ])
