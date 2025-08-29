import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import Int32MultiArray, Bool

class TactileSensorPublisher(Node):
    def __init__(self):
        super().__init__('tactile_sensor_publisher')
        
        # Declare parameters for serial port, baud rate, and threshold
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('calibration_samples', 100) # Number of samples to average for baseline
        self.declare_parameter('touch_threshold_percentage', 15.0) # Percentage drop to detect touch

        # Get parameters
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.calibration_samples_count = self.get_parameter('calibration_samples').get_parameter_value().integer_value
        
        # Initialize baseline and calibration data structures
        self.baseline_values = None
        self._calibration_data = []
        
        # Create publishers for both raw data and binary touch data
        self.raw_publisher_ = self.create_publisher(Int32MultiArray, 'tactile_raw_values', 10)
        self.touch_publisher_ = self.create_publisher(Bool, 'tactile_touch_detected', 10)
        self.binary_array_publisher_ = self.create_publisher(Int32MultiArray, 'tactile_binary_array', 10)
        
        # Set up the serial connection
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=1.0)
            time.sleep(2) # Wait for the connection to establish
            self.get_logger().info(f'Successfully connected to serial port: {serial_port}')
            self.get_logger().info(f'Starting calibration with {self.calibration_samples_count} samples...')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port {serial_port}. Error: {e}')
            # Shutdown the node if the serial port is not available.
            self.destroy_node()
            rclpy.shutdown()
            return

        # Create a timer to read and publish data every 20ms (50Hz) for lower latency
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').rstrip()
                values = [int(v) for v in line.split(',')]

                if len(values) != 4:
                    self.get_logger().warn(f'Received incomplete data: {line} (expected 4 values)')
                    return

                # --- Calibration Phase ---
                if self.baseline_values is None:
                    self._calibration_data.append(values)
                    if len(self._calibration_data) >= self.calibration_samples_count:
                        # Calculate the average for each sensor
                        num_sensors = len(self._calibration_data[0])
                        avg_baseline = [0] * num_sensors
                        for i in range(num_sensors):
                            sensor_sum = sum(sample[i] for sample in self._calibration_data)
                            avg_baseline[i] = sensor_sum / len(self._calibration_data)
                        
                        self.baseline_values = avg_baseline
                        self.get_logger().info(f'Calibration complete. Baseline values set to: {self.baseline_values}')
                    else:
                        self.get_logger().info(f'Calibrating... {len(self._calibration_data)}/{self.calibration_samples_count}')
                    return # Skip publishing during calibration

                # --- Publishing Phase ---
                # Publish raw values
                raw_msg = Int32MultiArray()
                raw_msg.data = values
                self.raw_publisher_.publish(raw_msg)
                
                # Dynamically get the latest touch threshold and calculate the multiplier
                touch_threshold_percentage = self.get_parameter('touch_threshold_percentage').get_parameter_value().double_value
                touch_multiplier = 1.0 - (touch_threshold_percentage / 100.0)

                # Process binary touch detection based on percentage drop from baseline
                binary_values = [1 if val < (base * touch_multiplier) else 0 for val, base in zip(values, self.baseline_values)]
                any_touch = any(binary_values)
                
                # Publish binary touch state (True if any sensor is touched)
                touch_msg = Bool()
                touch_msg.data = any_touch
                self.touch_publisher_.publish(touch_msg)
                
                # Publish binary array (individual sensor states)
                binary_array_msg = Int32MultiArray()
                binary_array_msg.data = binary_values
                self.binary_array_publisher_.publish(binary_array_msg)
                
                # Log the data (only when touch is detected to reduce log spam)
                if any_touch:
                    self.get_logger().info(f'TOUCH DETECTED! Raw: {values} | Binary: {binary_values}')
                else:
                    self.get_logger().debug(f'Raw: {values} | Binary: {binary_values}')

            except (UnicodeDecodeError, ValueError) as e:
                self.get_logger().warn(f'Could not parse serial data: {e}')
            except Exception as e:
                self.get_logger().error(f'An unexpected error occurred: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TactileSensorPublisher()
    if node.ser: # Only spin if serial connection was successful
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            # Cleanly close the serial port and shutdown
            node.ser.close()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
