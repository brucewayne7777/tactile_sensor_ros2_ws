import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self.declare_parameter('serial_port', '/dev/ttyACM0')  # Change to your port
        self.declare_parameter('baud_rate', 9600)

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
            self.get_logger().info(f"Serial port {serial_port} opened")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port {serial_port}: {e}")
            self.ser = None

        self.subscriber = self.create_subscription(
            String,
            'servo_command',
            self.command_callback,
            10
        )
        self.get_logger().info("Servo controller node started, listening for 'servo_command' topic")

    def command_callback(self, msg):
        cmd = msg.data.strip()
        self.get_logger().info(f"Received command: {cmd}")

        if self.ser and self.ser.is_open:
            try:
                # Send command followed by newline
                self.ser.write((cmd + '\n').encode('utf-8'))
                self.get_logger().info(f"Sent to Arduino: {cmd}")
                # Optionally, read back response
                response = self.ser.readline().decode('utf-8').strip()
                if response:
                    self.get_logger().info(f"Arduino response: {response}")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial write failed: {e}")
        else:
            self.get_logger().error("Serial port not available")

def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
