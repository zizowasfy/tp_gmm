from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # We declare an argument to allow overriding the config package
    moveit_config_pkg_arg = DeclareLaunchArgument(
        'moveit_config_pkg',
        default_value='kinova_gen3_7dof_robotiq_2f_85_moveit_config',
        description='MoveIt config package for the Kinova Gen3'
    )
    
    # Generate MoveIt config dictionaries dynamically
    # This automatically loads robot_description, SRDF, OMPL pipeline, kinematics, etc.
    # from the specified moveit_config package.
    moveit_config = (
        MoveItConfigsBuilder("gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config")
        .planning_pipelines("ompl")
        .to_dict()
    )

    collector_node = Node(
        package="tp_gmm",
        executable="collect_trajectories_gen3_ros2.py",
        output="screen",
        parameters=[moveit_config]
    )

    return LaunchDescription([
        moveit_config_pkg_arg,
        collector_node
    ])
