from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='radio_nav',
            executable='gps_node',
            name='gps_node',
            output='screen',
            parameters=[
                {'port': '/dev/ttyACM0'},
                {'baud_rate': 9600}
            ]
        ),
        Node(
            package='radio_nav',
            executable='uwb_node',
            name='uwb_node',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB0'},
                {'baud_rate': 115200}
            ]
        ),
        Node(
            package='radio_nav',
            executable='gps_kalman_filter',
            name='gps_kalman_filter',
            output='screen',
            parameters=[
                {'process_noise_lat': 1e-5},
                {'process_noise_lon': 1e-5},
                {'measurement_noise_lat': 1e-3},
                {'measurement_noise_lon': 1e-3}
            ]
        ),
        Node(
            package='radio_nav',
            executable='beacon_estimator_kalman',
            name='beacon_estimator',
            output='screen'
        ),
        Node(
            package='radio_nav',
            executable='display_nav',
            name='display_nav',
            output='screen'
        )
    ])