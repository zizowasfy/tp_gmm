from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Match the covariance corridor used by the retrained model experiments.
    cutoff_arg = DeclareLaunchArgument('gmm_cutoff', default_value='2.0')
    floor_arg = DeclareLaunchArgument('gmm_covariance_floor', default_value='1e-8')
    gmm_geometry = {
        'cutoff': ParameterValue(LaunchConfiguration('gmm_cutoff'), value_type=float),
        'covariance_floor': ParameterValue(LaunchConfiguration('gmm_covariance_floor'), value_type=float),
        'legacy_weighted_scale': ParameterValue(LaunchConfiguration('gmm_legacy_weighted_scale'), value_type=bool),
        'scale': ParameterValue(LaunchConfiguration('gmm_corridor_scale'), value_type=float),
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
        parameters=[
            {
                'policy_ckpt_path': '/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-09-11_21-20-28_ppo_torch/checkpoints/best_agent.pt',
            }
        ],
        output='screen'
    )

    gmm_rviz_converter_node = Node(
        package='tp_gmm',
        executable='gmm_rviz_converter_node',
        name='gmm_rviz_converter_node',
        parameters=[gmm_geometry],
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
            **gmm_geometry,
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
        DeclareLaunchArgument('gmm_legacy_weighted_scale', default_value='false'),
        DeclareLaunchArgument('gmm_corridor_scale', default_value='10.0'),
        cutoff_arg,
        floor_arg,
        task_arg,
        subtask_arg,
        tp_gmm_node,
        gmm_rviz_converter_node,
        gmm_moveit_node,
        deformed_gmm_rviz_converter_node,
        deformed_gmm_moveit_node
    ])
