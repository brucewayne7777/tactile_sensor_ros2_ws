from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino connection'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )
    
    threshold_arg = DeclareLaunchArgument(
        'touch_threshold',
        default_value='400',
        description='Threshold for binary touch detection (0-1023). Touch detected when value < threshold'
    )
    
    # Create the tactile publisher node
    tactile_publisher_node = Node(
        package='tactile_sensor_pkg',
        executable='tactile_publisher',
        name='tactile_publisher',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'touch_threshold': LaunchConfiguration('touch_threshold')
        }],
        output='screen'
    )
    
    # Create the tactile visualizer node
    tactile_visualizer_node = Node(
        package='tactile_sensor_pkg',
        executable='tactile_visualizer',
        name='tactile_visualizer',
        output='screen'
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        threshold_arg,
        tactile_publisher_node,
        tactile_visualizer_node
    ])
