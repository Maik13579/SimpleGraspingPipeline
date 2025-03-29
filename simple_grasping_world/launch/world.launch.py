from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    default_params_path = os.path.join(
        get_package_share_directory('simple_grasping_world'),
        'params',
        'params.yaml'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_path,
        description='YAML file with parameters for the world component'
    )

    # Declare launch argument to remap the input cloud topic
    input_cloud_arg = DeclareLaunchArgument(
        'input_cloud_topic',
        default_value='~/input_cloud',
        description='Topic to subscribe for input cloud'
    )

    params_file = LaunchConfiguration('params_file')

    # Launch your node as a component in a container
    container = ComposableNodeContainer(
        name='component_manager',
        namespace='/simple_world',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                namespace='/simple_world',
                package='simple_grasping_world',
                plugin='SimpleGraspingWorldNode',
                name='simple_grasping_world',
                parameters=[params_file],
                remappings=[('~/input_cloud', LaunchConfiguration('input_cloud_topic'))]
            )
        ]
    )

    return LaunchDescription([
        params_file_arg,
        input_cloud_arg,
        container
    ])
