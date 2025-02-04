import rclpy
from rclpy.node import Node

from robot_msgs.msg import Xbox


class DriveNode(Node):

    def __init__(self):
        super().__init__('drive_node')
        self.xbox_subscription = self.create_subscription(Xbox, '/xbox', self.xbox_callback, 10)


    def xbox_callback(self, msg):
        self.get_logger().info('Received Xbox message: %s' % msg)


def main(args=None):
    rclpy.init(args=args)

    node = DriveNode()
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