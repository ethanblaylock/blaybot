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

        # Create Xbox message
        xbox_msg = Xbox()

        # Assign values from Joy axes to Xbox message
        xbox_msg.l_stick_lr= msg.axes[0]
        xbox_msg.l_stick_ud = msg.axes[1]
        xbox_msg.r_stick_lr = msg.axes[2]
        xbox_msg.r_stick_ud = msg.axes[3]
        xbox_msg.r_trigger = msg.axes[4]
        xbox_msg.l_trigger = msg.axes[5]
        xbox_msg.d_pad_lr = msg.axes[6]
        xbox_msg.d_pad_ud = msg.axes[7]

        # Assign values from Joy buttons to Xbox message
        xbox_msg.a = msg.buttons[0]
        xbox_msg.b = msg.buttons[1]
        xbox_msg.x = msg.buttons[3]
        xbox_msg.y = msg.buttons[4]
        xbox_msg.l_bumper = msg.buttons[6]
        xbox_msg.r_bumper = msg.buttons[7]
        xbox_msg.view = msg.buttons[10]
        xbox_msg.menu = msg.buttons[11]
        xbox_msg.xbox= msg.buttons[12]
        xbox_msg.l_stick_press = msg.buttons[13]
        xbox_msg.r_stick_press = msg.buttons[14]
        xbox_msg.share = msg.buttons[15]

        # Publish Xbox message
        self.xbox_publisher.publish(xbox_msg)



def main(args=None):
    rclpy.init(args=args)

    node = JoystickNode()
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