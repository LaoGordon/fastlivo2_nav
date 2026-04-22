from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fastlivo_odom_topic = LaunchConfiguration('fastlivo_odom_topic')
    fastlivo_cloud_topic = LaunchConfiguration('fastlivo_cloud_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    obstacle_topic = LaunchConfiguration('obstacle_topic')
    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    publish_identity_map_to_odom = LaunchConfiguration('publish_identity_map_to_odom')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fastlivo_odom_topic',
            default_value='/aft_mapped_to_init',
            description='FAST-LIVO2 odometry topic'),
        DeclareLaunchArgument(
            'fastlivo_cloud_topic',
            default_value='/cloud_registered_lidar',
            description='FAST-LIVO2 registered cloud topic'),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Output nav odometry topic'),
        DeclareLaunchArgument(
            'obstacle_topic',
            default_value='/obstacle_points',
            description='Output obstacle pointcloud topic'),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Global map frame'),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='Odometry frame'),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base',
            description='Robot base frame'),
        DeclareLaunchArgument(
            'publish_identity_map_to_odom',
            default_value='true',
            description='Publish identity map->odom TF in bridge'),
        Node(
            package='fastlivo_nav_bridge',
            executable='fastlivo_nav_bridge_node',
            name='fastlivo_nav_bridge',
            output='screen',
            parameters=[
                {
                    'fastlivo_odom_topic': fastlivo_odom_topic,
                    'fastlivo_cloud_topic': fastlivo_cloud_topic,
                    'odom_topic': odom_topic,
                    'obstacle_topic': obstacle_topic,
                    'map_frame': map_frame,
                    'odom_frame': odom_frame,
                    'base_frame': base_frame,
                    'publish_identity_map_to_odom': publish_identity_map_to_odom,
                }
            ],
        )
    ])
