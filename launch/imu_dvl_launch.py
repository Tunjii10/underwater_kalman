from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kalman_filters',
            executable='imu_dvl_publisher',
            name='imu_dvl_publisher',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '~/colcon_ws/src/kalman_filters/rviz/imu_dvl_config.rviz']
        ),
    ])
