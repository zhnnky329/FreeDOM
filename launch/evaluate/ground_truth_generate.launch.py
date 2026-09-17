from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('freedom')
    save_map_path = PathJoinSubstitution([pkg_share, 'generated_pcd/'])
    save_frame_path = PathJoinSubstitution([pkg_share, 'generated_pcd/frame/'])

    ground_truth_node = Node(
        package='freedom',
        executable='ground_truth_generate',
        name='ground_truth_generate',
        output='screen',
        parameters=[{
            'sensor_tf_frame': 'velodyne',
            'map_tf_frame': 'map',
            'pointcloud_topic': '/velodyne_points',
            'save_map_topic': '/save_map',
            # voxel_size = 0.2 for outdoor datasets and 0.1 for indoor datasets
            'voxel_size': 0.2,
            # min_range = 2.7 for outdoor datasets and 0.0 for indoor datasets
            'min_range': 2.7,
            'max_range': 1000.0,
            'save_map_path': save_map_path,
            'enable_save_frame': False,
            'save_frame_path': save_frame_path,
        }],
    )

    return LaunchDescription([ground_truth_node])
