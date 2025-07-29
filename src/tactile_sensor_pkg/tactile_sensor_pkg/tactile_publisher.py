# Improved tactile_publisher.py

import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import Int32MultiArray

class TactileSensorPublisher(Node):
    def __init__(self):
        super().__init__('tactile_sensor_publisher')
        
        # Flag to indicate if the serial port was connected successfully
        self.is_connected = False
        
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        self.publisher_ = self.create_publisher(Int32MultiArray, 'tactile_values', 10)
        
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=1.0)
            time.sleep(2)
            self.get_logger().info(f'Successfully connected to serial port: {serial_port}')
            self.is_connected = True # Set flag on success
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port {serial_port}. Error: {e}')
            return # Exit __init__ if connection fails

        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').rstrip()
                values = [int(v) for v in line.split(',')]
                
                if len(values) == 4:
                    msg = Int32MultiArray()
                    msg.data = values
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Publishing: {msg.data}', throttle_duration_sec=1)
                else:
                    self.get_logger().warn(f'Received incomplete data: {line}')

            except (UnicodeDecodeError, ValueError) as e:
                self.get_logger().warn(f'Could not parse serial data: {e}')
            except Exception as e:
                self.get_logger().error(f'An unexpected error occurred: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TactileSensorPublisher()
    
    # Only spin the node if the serial connection was successful
    if not node.is_connected:
        node.get_logger().info("Shutting down due to serial connection failure in __init__.")
    else:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            # Cleanly close the serial port and shutdown
            node.ser.close()
            node.get_logger().info('Serial port closed.')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
