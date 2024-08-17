from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher2',
            output='screen',
            arguments=['0.36377', '-0.02093', '-0.08490', '1.5708', '0', '1.5708', 'body', 'imu_frame'],
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher3',
            output='screen',
            arguments=['-0.4145', '0', '0.11', '3.14159', '0', '2.35619', 'body', 'dvl_frame'],
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher1',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'body', 'ground_truth_frame'],
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='kalman_filters',
            executable='imu_dvl_publisher',
            name='imu_dvl_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '~/colcon_ws/src/kalman_filters/rviz/imu_dvl_config.rviz'],
            parameters=[{'use_sim_time': True}],    
        ),
    ])
