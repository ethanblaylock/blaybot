import rclpy
from rclpy.node import Node

from robot_msgs.msg import Xbox
from robot_msgs.msg import MotorCommand

from mobility import parameters as p

class DriveNode(Node):
    
    def __init__(self):
        super().__init__('drive_node')
        self.xbox_subscription = self.create_subscription(Xbox, '/xbox', self.xbox_callback, 10)

        self.motor_command_publisher = self.create_publisher(MotorCommand, '/motor_command', 10)

    def xbox_callback(self, msg):
        
        motor_command_msg = MotorCommand()
        if msg.l_stick_ud > 0:
            motor_command_msg.left_forward = int(msg.l_stick_ud*p.MAX_PWM_COUNTS)
            motor_command_msg.left_reverse= 0
        else:
            motor_command_msg.left_forward = 0
            motor_command_msg.left_reverse = int(-msg.l_stick_ud*p.MAX_PWM_COUNTS)
        
        if msg.r_stick_ud > 0:
            motor_command_msg.right_forward = int(msg.r_stick_ud*p.MAX_PWM_COUNTS)
            motor_command_msg.right_reverse = 0
        else:
            motor_command_msg.right_forward = 0
            motor_command_msg.right_reverse = int(-msg.r_stick_ud*p.MAX_PWM_COUNTS)
        
        self.motor_command_publisher.publish(motor_command_msg)

def main(args=None):
    rclpy.init(args=args)

    node = DriveNode()
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