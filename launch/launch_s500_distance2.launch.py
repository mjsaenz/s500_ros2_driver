from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    connection_type = LaunchConfiguration('connection_type', default='udp')
    udp_ip = LaunchConfiguration('udp_ip', default='192.168.0.20')
    udp_port = LaunchConfiguration('udp_port', default='51200')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    baud_rate = LaunchConfiguration('baud_rate', default='115200')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'connection_type',
            default_value='udp',
            description='Connection type to use. Valid parameters are "udp" or "serial".'
        ),
        DeclareLaunchArgument(
            'udp_ip',
            default_value='192.168.0.20',
            description='IP of S500 PCB. Ignored if connection_type parameter value is "serial".'
        ),
        DeclareLaunchArgument(
            'udp_port',
            default_value='51200',
            description='UDP port of S500 PCB. Ignored if connection_type parameter value is "serial".'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port of S500 PCB. Ignored if connection_type parameter value is "udp".'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate of serial connection to S500 PCB. Ignored if connection_type parameter value is "udp".'
        ),
        Node(
            package='s500_ros2_driver',
            executable='s500_publisher_node',
            name='s500_distance2_publisher_node',
            output='screen',
            respawn=True,
            parameters=[{
                'use_sim_time': use_sim_time,
                # connection parameters, changes after launch will be ignored
                'connection_type': connection_type,
                'udp_ip': udp_ip,
                'udp_port': udp_port,
                'serial_port': serial_port,
                'baud_rate': baud_rate,
                
                # device parameters, changes after launch will be ignored
                'start_mm': 0,
                'length_mm': 20000,
                'gain_index': -1,
                'ping_interval_ms': 100, # 10hz, 25hz is upper limit reported by ros2 topic hz, but manual timing suggests true upper limit could be higher
                'packet_type': 'distance2',
                'chirp_enable': True,
                'decimation': 0,
                'frame_id': "Sounder_S500_Distance2",
                
                # dynamic parameter(s), s500 will be updated when changed after launch
                'sos_mm_per_sec': 1500000
            }]
        )
    ])
