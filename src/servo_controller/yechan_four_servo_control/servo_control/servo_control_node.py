import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Initialize serial connection to Arduino
        self.create_subscription(String, 'servo_command', self.listener_callback, 10)  # Subscribe to topic

    def listener_callback(self, msg):
        command = msg.data

        # Parse command: first char is servo number (1-4), rest is action (pull, push, neutral)
        if len(command) >= 2 and command[0] in ['1', '2', '3', '4']:
            servo_num = int(command[0])
            action = command[1:].lower()  # Extract action and convert to lowercase

            if action == 'pull':
                self.ser.write(f'{servo_num}p'.encode())  # Send 'p' to Arduino for pull action
                self.get_logger().info(f'Servo {servo_num} set to Pull (20 degrees)')
            elif action == 'push':
                self.ser.write(f'{servo_num}u'.encode())  # Send 'u' to Arduino for push action
                self.get_logger().info(f'Servo {servo_num} set to Push (155 degrees)')
            elif action == 'neutral':
                self.ser.write(f'{servo_num}n'.encode())  # Send 'n' to Arduino for neutral action
                self.get_logger().info(f'Servo {servo_num} set to Neutral (110 degrees)')
            else:
                self.get_logger().warn(f'Invalid action for Servo {servo_num}: {action}')  # Warn if action is invalid
        else:
            self.get_logger().warn(f'Invalid command format: {command} (e.g., "1pull", "2push")')  # Warn if command format is wrong

def main(args=None):
    rclpy.init(args=args)
    node = ServoControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
