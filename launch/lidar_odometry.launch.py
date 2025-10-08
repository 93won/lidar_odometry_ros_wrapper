#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        description='Path to configuration YAML file (REQUIRED)'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base frame ID'
    )
    
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame', 
        default_value='odom',
        description='Odometry frame ID'
    )
    
    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map', 
        description='Map frame ID'
    )
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Whether to publish TF transforms'
    )
    
    enable_viewer_arg = DeclareLaunchArgument(
        'enable_viewer',
        default_value='false',
        description='Whether to enable Pangolin viewer'
    )
    
    publish_features_arg = DeclareLaunchArgument(
        'publish_features',
        default_value='false',
        description='Whether to publish feature points'
    )
    
    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='100.0',
        description='Maximum range for point cloud filtering'
    )
    
    min_range_arg = DeclareLaunchArgument(
        'min_range',
        default_value='1.0',
        description='Minimum range for point cloud filtering'
    )
    
    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/velodyne_points',
        description='Input point cloud topic name'
    )
    
    # RViz argument
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    # Simulation time argument for rosbag playback
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (needed for rosbag playback)'
    )
    
    # LiDAR Odometry Node
    lidar_odometry_node = Node(
        package='lidar_odometry_ros',
        executable='lidar_odometry_node',
        name='simple_lidar_odometry',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('velodyne_points', LaunchConfiguration('pointcloud_topic')),
        ]
    )
    
    # RViz node
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('lidar_odometry_ros'),
        'rviz',
        'lidar_odometry.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'FATAL'],
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        additional_env={'RCUTILS_LOGGING_SEVERITY_THRESHOLD': 'FATAL'},
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )
    
    return LaunchDescription([
        config_file_arg,
        base_frame_arg,
        odom_frame_arg,
        map_frame_arg,
        publish_tf_arg,
        enable_viewer_arg,
        publish_features_arg,
        max_range_arg,
        min_range_arg,
        pointcloud_topic_arg,
        enable_rviz_arg,
        use_sim_time_arg,
        LogInfo(msg="Starting LiDAR Odometry ROS Node..."),
        lidar_odometry_node,
        rviz_node,
    ])
