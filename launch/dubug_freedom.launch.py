from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('freedom')

    # 配置文件路径
    config_file = PathJoinSubstitution([pkg_share, 'config', 'kitti_velodyne.yaml'])

    pcd_save_path = PathJoinSubstitution([pkg_share, 'generated_pcd/'])
    fov_mask_path = PathJoinSubstitution([pkg_share, 'config/'])

    freedom_node = Node(
        package='freedom',
        executable='freedom_node',
        name='freedom',
        output='screen',
        prefix='gdb -ex run --args',  # 使用GDB调试
        parameters=[
            # 加载YAML配置文件
            config_file,
            # 添加其他参数
            {
                'pointcloud_topic': '/unilidar/cloud',
                'map_tf_frame': 'unilidar_lidar',
                'sensor_tf_frame': 'unilidar_lidar',
                'save_map_topic': '/save_map',
                'save_map_path': pcd_save_path,
                'raycast_enhancement.fov_mask_path': fov_mask_path,
                'raycast_enhancement.learn_fov': False,
                'enable_visualization': True,
            }
        ],
    )

    return LaunchDescription([
        freedom_node,
    ])
