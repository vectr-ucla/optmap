import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    current_pkg = FindPackageShare('optmap')
    nn_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'nn.yaml'])
    optmap_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'optmap.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('optmap_namespace', default_value='optmap', description='OptMap Node Namespace'),
        DeclareLaunchArgument('pointcloud_topic', default_value='dliom/odom_node/pointcloud', description='PointCloud Topic'),
        DeclareLaunchArgument('pose_topic', default_value='dliom/odom_node/pose_optmap', description='Pose Topic'),
        DeclareLaunchArgument('pose_update_topic', default_value='/temp', description='Pose Updates from Loop Closures'),

        # Point Cloud Descriptor Node
        Node(
            package='optmap',
            executable='pc_descriptor_node.py',
            name='descriptor_node',
            namespace=LaunchConfiguration('optmap_namespace'),
            output='screen',
            emulate_tty=True,
            parameters=[nn_yaml_path],
            remappings=[('pointcloud', LaunchConfiguration('pointcloud_topic')),
                        ('descriptor', 'pc_descriptor_node/descriptor')],
        ),
        
        # OptMap Node
        Node(
            package='optmap',
            executable='optmap_node',
            name='optmap_node',
            namespace=LaunchConfiguration('optmap_namespace'),
            output='screen',
            emulate_tty=True,
            parameters=[optmap_yaml_path],
            remappings=[('descriptor', 'pc_descriptor_node/descriptor'),
                        ('pointcloud', LaunchConfiguration('pointcloud_topic')),
                        ('pose', LaunchConfiguration('pose_topic')),
                        ('pose_update', LaunchConfiguration('pose_update_topic')),
                        ('map', 'optmap/map'),
                        ('map_scans', 'optmap/map_scans'),
                        ('poses', 'optmap/poses'),
                        ('markers', 'optmap/markers')],
        ),
    ])