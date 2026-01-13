from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='s500_ros2_driver',
            executable='s500_publisher_node',
            name='s500_distance2_publisher_node',
            output='screen',
            respawn=True,
            parameters=[{
                # connection parameters, changes after launch will be ignored
                'connection_type': 'udp', # uncomment 'udp_' or 'serial_' params as appropriate
                'udp_ip': '192.168.0.20',
                'udp_port': 51200,
                # 'serial_port', '/dev/ttyUSB0',
                # 'baud_rate', 115200,
                
                # device parameters, changes after launch will be ignored
                'start_mm': 0,
                'length_mm': 20000,
                'gain_index': -1,
                'ping_interval_ms': 100, # 10hz, 25hz is upper limit
                'packet_type': 'distance2',
                'chirp_enable': True,
                'decimation': 0,
                'frame_id': "Sounder_S500_Distance2",
                
                # dynamic parameter(s), s500 will be updated when changed after launch
                'sos_mm_per_sec': 1500000
            }]
        )
    ])
