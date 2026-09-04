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
        parameters=[
            {
                # 'policy_ckpt_path': '/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-06-15_17-18-11_ppo_torch/checkpoints/best_agent.pt'
                # 'policy_ckpt_path': '/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-06-04_16-49-13_ppo_torch_envs=32/checkpoints/best_agent.pt'
                # 'policy_ckpt_path': '/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-08-05_21-41-30_ppo_torch/checkpoints/best_agent.pt'
                # 'policy_ckpt_path': '/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-08-21_23-14-27_ppo_torch/checkpoints/best_agent.pt'
                # 'policy_ckpt_path': '/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-08-25_22-25-09_ppo_torch/checkpoints/best_agent.pt'
                # 'policy_ckpt_path': '/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-08-25_23-08-51_ppo_torch/checkpoints/best_agent.pt'
                # 'policy_ckpt_path': '/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-08-26_13-09-35_ppo_torch/checkpoints/best_agent.pt',
                # 'policy_ckpt_path': '/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-08-26_14-02-37_ppo_torch/checkpoints/best_agent.pt'
                # 'policy_ckpt_path': '/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-08-26_19-15-12_ppo_torch/checkpoints/best_agent.pt',
                # 'policy_ckpt_path': '/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-08-26_21-56-10_ppo_torch/checkpoints/best_agent.pt',
                # 'policy_ckpt_path': '/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-08-31_10-49-08_ppo_torch/checkpoints/best_agent.pt',
                # 'policy_ckpt_path': '/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-08-31_11-36-52_ppo_torch/checkpoints/best_agent.pt',
                # 'policy_ckpt_path': '/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-08-31_21-09-18_ppo_torch/checkpoints/best_agent.pt',
                # 'policy_ckpt_path': '/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-08-31_21-55-45_ppo_torch/checkpoints/best_agent.pt',
                # 'policy_ckpt_path': '/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-08-31_22-40-43_ppo_torch/checkpoints/best_agent.pt',
                # 'policy_ckpt_path': '/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-09-01_01-05-26_ppo_torch/checkpoints/best_agent.pt',
                'policy_ckpt_path': '/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-09-03_20-31-51_ppo_torch/checkpoints/best_agent.pt',

            }
        ],
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

    deformed_gmm_rviz_converter_node = Node(
        package='tp_gmm',
        executable='gmm_rviz_converter_node',
        name='deformed_gmm_rviz_converter_node',
        parameters=[{
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
        task_arg,
        subtask_arg,
        tp_gmm_node,
        gmm_rviz_converter_node,
        gmm_moveit_node,
        deformed_gmm_rviz_converter_node,
        deformed_gmm_moveit_node
    ])
