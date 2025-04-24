from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port for the IMU device'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Baud rate for the IMU device'
        ),
        Node(
            package='imu_driver',
            executable='driver',
            name='imu_driver',
            output='screen',
            parameters=[
                {'port': LaunchConfiguration('port')},
                
            ]
        )
    ])

