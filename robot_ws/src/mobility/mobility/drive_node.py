import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from robot_msgs.msg import Xbox
from robot_msgs.msg import MotorCommand

from mobility import parameters as p

class DriveNode(Node):
    
    def __init__(self):
        super().__init__('drive_node')
        self.xbox_subscription = self.create_subscription(Xbox, '/xbox', self.xbox_callback, 10)

        self.motor_command_publisher = self.create_publisher(MotorCommand, '/motor_command', 10)

        self.speed = 3

        self.lb_debounce = True
        self.rb_debounce = True

    def xbox_callback(self, msg):
        if msg.l_bumper == 1 and self.speed > 0 and self.lb_debounce:
            self.speed -= 1
            self.lb_debounce = False
        if msg.l_bumper == 0:
            self.lb_debounce = True
        if msg.r_bumper == 1 and self.speed < 3 and self.rb_debounce:
            self.speed += 1
            self.rb_debounce = False
        if msg.r_bumper == 0:
            self.rb_debounce = True

        motor_command_msg = MotorCommand()
        if msg.l_stick_ud > 0:
            motor_command_msg.left_forward = int(msg.l_stick_ud*p.MAX_PWM_COUNTS*p.DRIVE_SPEEDS[self.speed])
            motor_command_msg.left_reverse= 0
        else:
            motor_command_msg.left_forward = 0
            motor_command_msg.left_reverse = int(-msg.l_stick_ud*p.MAX_PWM_COUNTS*p.DRIVE_SPEEDS[self.speed])
        
        if msg.r_stick_ud > 0:
            motor_command_msg.right_forward = int(msg.r_stick_ud*p.MAX_PWM_COUNTS*p.DRIVE_SPEEDS[self.speed])
            motor_command_msg.right_reverse = 0
        else:
            motor_command_msg.right_forward = 0
            motor_command_msg.right_reverse = int(-msg.r_stick_ud*p.MAX_PWM_COUNTS*p.DRIVE_SPEEDS[self.speed])
        self.motor_command_publisher.publish(motor_command_msg)

def main(args=None):
    rclpy.init(args=args)

    node = DriveNode()
    try:    
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
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