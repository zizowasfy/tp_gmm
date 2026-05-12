from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    task_arg = DeclareLaunchArgument(
        'task',
        default_value='Rbolts',
        description='Task argument'
    )
    
    subtask_arg = DeclareLaunchArgument(
        'subtask',
        default_value='action',
        description='Subtask argument'
    )


    tp_gmm_node = Node(
        package='tp_gmm',
        executable='tp_gmm_node.py',
        name='tp_gmm',
        output='screen'
    )

    gmm_rviz_converter_node = Node(
        package='tp_gmm',
        executable='gmm_rviz_converter_node',
        name='gmm_rviz_converter_node',
        output='screen'
    )

    gmm_moveit_node = Node(
        package='tp_gmm',
        executable='gmm_moveit_node',
        name='gmm_moveit_node',
        output='screen'
    )


    return LaunchDescription([
        task_arg,
        subtask_arg,
        tp_gmm_node,
        gmm_rviz_converter_node,
        gmm_moveit_node
    ])
