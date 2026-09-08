"""Plan-only Panda sampler laboratory: no controllers, hardware connection or execution."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config = (MoveItConfigsBuilder('moveit_resources_panda')
              .robot_description(file_path='config/panda.urdf.xacro')
              .robot_description_semantic(file_path='config/panda.srdf')
              .planning_pipelines(pipelines=['ompl'])
              .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
              .to_moveit_configs())
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('lfd', default_value='true'),
        DeclareLaunchArgument('policy_ckpt_path', default_value='', description='RL checkpoint; required for deformed comparison'),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=[config.to_dict(), {'allow_trajectory_execution': False,
                          'constraint_samplers': 'tp_gmm/GMMConstraintSamplerAllocator'}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[config.robot_description]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[config.robot_description, {'zeros.panda_joint2': -0.785,
                          'zeros.panda_joint4': -2.356, 'zeros.panda_joint6': 1.571,
                          'zeros.panda_joint7': 0.785, 'zeros.panda_finger_joint1': 0.04}]),
        Node(package='tp_gmm', executable='tp_gmm_node.py', output='screen',
             condition=IfCondition(LaunchConfiguration('lfd')),
             parameters=[{'policy_ckpt_path': LaunchConfiguration('policy_ckpt_path')}]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', PathJoinSubstitution([FindPackageShare('moveit_resources_panda_moveit_config'),
                                                   'launch', 'sampling.rviz'])],
             parameters=[config.robot_description, config.robot_description_semantic,
                         config.robot_description_kinematics, config.planning_pipelines]),
    ])
