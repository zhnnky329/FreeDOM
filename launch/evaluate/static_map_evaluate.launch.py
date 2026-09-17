from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('freedom')
    ground_truth_path = PathJoinSubstitution([pkg_share, 'generated_pcd', 'ground_truth_voxel.pcd'])
    static_map_path = PathJoinSubstitution([pkg_share, 'generated_pcd', 'static_map_voxel.pcd'])

    evaluate_node = Node(
        package='freedom',
        executable='static_map_evaluate',
        name='static_map_evaluate',
        output='screen',
        parameters=[{
            # voxel_size = 0.2 for outdoor datasets and 0.1 for indoor datasets
            'voxel_size': 0.2,
            'ground_truth_path': ground_truth_path,
            'static_map_path': static_map_path,
        }],
    )

    return LaunchDescription([evaluate_node])
