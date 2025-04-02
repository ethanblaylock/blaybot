import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from robot_msgs.msg import Xbox
from robot_msgs.msg import Mode

from mobility import parameters as p

class ModeManagerNode(Node):
    
    def __init__(self):
        super().__init__('mode_manager_node')
        self.xbox_subscription = self.create_subscription(Xbox, '/xbox', self.xbox_callback, 10)

        self.mode_publisher = self.create_publisher(Mode, '/mode', 10)

        self.xbox_debounce = True

        self.current_mode = Mode.DRIVE
        self.get_logger().info("Mode is now: " + "Drive")

    def xbox_callback(self, msg):
        if msg.xbox == 1 and self.xbox_debounce:
            mode_msg = Mode()
            mode_msg = self.cycle_mode(mode_msg)
            self.xbox_debounce = False
        if msg.xbox == 0:
            self.xbox_debounce = True
        
    def cycle_mode(self, mode_msg):
        if self.current_mode == Mode.DRIVE:
            mode_msg.mode = Mode.ARM
            self.current_mode = Mode.ARM
            self.get_logger().info("Mode is now: " + "Arm")
        else:
            mode_msg.mode = Mode.DRIVE
            self.current_mode = Mode.DRIVE
            self.get_logger().info("Mode is now: " + "Drive")
        self.mode_publisher.publish(mode_msg)
      

def main(args=None):
    rclpy.init(args=args)

    node = ModeManagerNode()
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