from google.protobuf.internal import python_message
from google.protobuf.internal import python_message
from google.protobuf.internal import python_message
from google.protobuf.internal import python_message
from google.protobuf.internal import python_message
from launch.actions import pop_launch_configurations
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    geometry = {
        'cutoff': ParameterValue(LaunchConfiguration('gmm_cutoff'), value_type=float),
        'covariance_floor': ParameterValue(LaunchConfiguration('gmm_covariance_floor'), value_type=float),
        'legacy_weighted_scale': False,
    }
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
        parameters=[{
            'policy_ckpt_path': LaunchConfiguration('policy_ckpt_path')
            }],
        output='screen'
    )

    gmm_rviz_converter_node = Node(
        package='tp_gmm',
        executable='gmm_rviz_converter_node',
        name='gmm_rviz_converter_node',
        parameters=[geometry],
        output='screen'
    )

    gmm_moveit_node = Node(
        package='tp_gmm',
        executable='gmm_moveit_node',
        name='gmm_moveit_node',
        output='screen'
    )

    deformed_gmm_rviz_converter_node = Node(
        package='tp_gmm',
        executable='gmm_rviz_converter_node',
        name='deformed_gmm_rviz_converter_node',
        parameters=[{
            **geometry,
            'input_topic': '/gmm/deformed_cartesian_space',
            'output_topic': 'deformed_gmm_rviz_converter_output'
        }],
        output='screen'
    )

    deformed_gmm_moveit_node = Node(
        package='tp_gmm',
        executable='gmm_moveit_node',
        name='deformed_gmm_moveit_node',
        remappings=[
            ('gmm_rviz_converter_output', 'deformed_gmm_rviz_converter_output'),
            ('gmm_moveit', 'deformed_gmm_moveit'),
            ('solid_primitives_viz', 'deformed_solid_primitives_viz')
        ],
        output='screen'
    )


    return LaunchDescription([
        DeclareLaunchArgument('policy_ckpt_path', default_value='/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-09-11_21-20-28_ppo_torch/checkpoints/best_agent.pt'),
        DeclareLaunchArgument('gmm_cutoff', default_value='2.0'),
        DeclareLaunchArgument('gmm_covariance_floor', default_value='1e-8'),
        task_arg,
        subtask_arg,
        tp_gmm_node,
        gmm_rviz_converter_node,
        gmm_moveit_node,
        deformed_gmm_rviz_converter_node,
        deformed_gmm_moveit_node
    ])
