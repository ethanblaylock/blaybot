import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from robot_msgs.msg import Xbox

class JoystickNode(Node):

    def __init__(self):
        super().__init__('joystick_node')
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.xbox_publisher = self.create_publisher(Xbox, '/xbox', 10)

    def joy_callback(self, msg):
        self.get_logger().info('Received Joy message: %s' % msg)

        # Create Xbox message
        xbox_msg = Xbox()

        # Assign values from Joy axes to Xbox message
        xbox_msg.L_STICK_LR = msg.axes[0]
        xbox_msg.L_STICK_UD = msg.axes[1]
        xbox_msg.L_TRIGGER = msg.axes[2]
        xbox_msg.R_STICK_LR = msg.axes[3]
        xbox_msg.R_STICK_UD = msg.axes[4]
        xbox_msg.R_TRIGGER = msg.axes[5]
        xbox_msg.D_PAD_LR = msg.axes[6]
        xbox_msg.D_PAD_UD = msg.axes[7]

        # Assign values from Joy buttons to Xbox message
        xbox_msg.A = msg.buttons[0]
        xbox_msg.B = msg.buttons[1]
        xbox_msg.X = msg.buttons[2]
        xbox_msg.Y = msg.buttons[3]
        xbox_msg.L_BUMPER = msg.buttons[4]
        xbox_msg.R_BUMPER = msg.buttons[5]
        xbox_msg.BACK = msg.buttons[6]
        xbox_msg.START = msg.buttons[7]
        xbox_msg.XBOX = msg.buttons[8]
        xbox_msg.L_STICK_PRESS = msg.buttons[9]
        xbox_msg.R_STICK_PRESS = msg.buttons[10]

        # Publish Xbox message
        self.xbox_publisher.publish(xbox_msg)



def main(args=None):
    rclpy.init(args=args)

    node = JoystickNode()
    try:    
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except:
        rclpy.shutdown()
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        node.destroy_node()
        

if __name__ == '__main__':
    main()