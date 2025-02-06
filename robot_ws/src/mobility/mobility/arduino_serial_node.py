import rclpy
from rclpy.node import Node
import serial
from robot_msgs.msg import MotorCommand
import time

class ArduinoSerialNode(Node):
    
    def __init__(self):
        super().__init__('arduino_serial_node')
     
        self.motor_command_subscription = self.create_subscription(MotorCommand, '/motor_command', self.motor_command_callback, 10)
        
        serial_port = '/dev/ttyACM0'
        baud_rate = 115200

        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=None)
            self.get_logger().info(f'Connected to Arduino on {serial_port} at {baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')
            self.ser = None

        if self.ser.is_open:
            self.get_logger().info('Arduino serial port is open')
        else:
            self.get_logger().error('Arduino serial port is not open')

    def motor_command_callback(self, msg):
        
        if self.ser is not None:
            command = str(msg.left_forward) + ',' + str(msg.left_reverse) + ',' + str(msg.right_forward) + ',' + str(msg.right_reverse) + '\n'
            try:
                self.ser.reset_input_buffer()
                self.ser.write(command.encode('utf-8'))
                self.ser.flush()
                # time.sleep(0.05)
                self.get_logger().info(f'Sent command to Arduino: {command}')
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to send command: {e}')
        else:
            self.get_logger().warn('Serial connection is not established.')

    def destroy_node(self):
        if self.ser is not None:
            self.ser.close()
        super().destroy_node()
 
def main(args=None):
    rclpy.init(args=args)

    node = ArduinoSerialNode()
    try:    
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
        rclpy.shutdown()
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        node.destroy_node()
        

if __name__ == '__main__':
    main()