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
    start_mm = LaunchConfiguration('start_mm', default='0')
    length_mm = LaunchConfiguration('length_mm', default='20000')
    gain_index = LaunchConfiguration('gain_index', default='-1')
    ping_interval_ms = LaunchConfiguration('ping_interval_ms', default='100')
    packet_type = LaunchConfiguration('packet_type', default='profile6_t')
    chirp_enable = LaunchConfiguration('chirp_enable', default='true')
    decimation = LaunchConfiguration('decimation', default='0')
    frame_id = LaunchConfiguration('frame_id', default='Sounder_S500_Profile6_t')
    sos_mm_per_sec = LaunchConfiguration('sos_mm_per_sec', default='1500000')
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
        DeclareLaunchArgument(
            'start_mm',
            default_value='0',
            description='Start of ping range in mm.'
        ),
        DeclareLaunchArgument(
            'length_mm',
            default_value='20000',
            description='Length of the returned range in mm.'
        ),
        DeclareLaunchArgument(
            'gain_index',
            default_value='-1',
            description='Gain index; -1 enables automatic gain.'
        ),
        DeclareLaunchArgument(
            'ping_interval_ms',
            default_value='100',
            description='Minimum interval between pings in milliseconds.'
        ),
        DeclareLaunchArgument(
            'packet_type',
            default_value='profile6_t',
            description='Packet type to request from the S500.'
        ),
        DeclareLaunchArgument(
            'chirp_enable',
            default_value='true',
            description='Enable CHIRP when true.'
        ),
        DeclareLaunchArgument(
            'decimation',
            default_value='0',
            description='Signal downsampling factor; 0 selects automatic range resolution in CHIRP mode.'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='Sounder_S500_Profile6_t',
            description='Frame ID for published message headers.'
        ),
        DeclareLaunchArgument(
            'sos_mm_per_sec',
            default_value='1500000',
            description='Speed of sound in the sounding medium in mm/s.'
        ),
        Node(
            package='s500_ros2_driver',
            executable='s500_publisher_node',
            name='s500_profile6_t_publisher_node',
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
                'start_mm': start_mm,
                'length_mm': length_mm,
                'gain_index': gain_index,
                'ping_interval_ms': ping_interval_ms, # 10hz, s500 operational upper limit
                'packet_type': packet_type,
                'chirp_enable': chirp_enable,
                'decimation': decimation,
                'frame_id': frame_id,
                
                # dynamic parameter(s), s500 will be updated when changed after launch
                'sos_mm_per_sec': sos_mm_per_sec
            }]
        )
    ])
