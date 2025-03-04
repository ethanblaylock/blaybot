import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from robot_msgs.msg import Xbox
from robot_msgs.msg import ArmCommand

from mobility import parameters as p

class ArmNode(Node):
    
    def __init__(self):
        super().__init__('arm_node')
        self.xbox_subscription = self.create_subscription(Xbox, '/xbox', self.xbox_callback, 10)

        self.arm_command_publisher = self.create_publisher(ArmCommand, '/arm_command', 10)

        self.current_joint1 = p.INIT_JOINT1
        self.current_joint2 = p.INIT_JOINT2
        self.current_joint3 = p.INIT_JOINT3
        self.current_joint4 = p.INIT_JOINT4
        self.current_joint5 = p.INIT_JOINT5
        self.current_joint6 = p.INIT_JOINT6


    def xbox_callback(self, msg):




        arm_command_msg = ArmCommand()
        arm_command_msg.joint1 = self.current_joint1
        arm_command_msg.joint2 = self.current_joint2
        arm_command_msg.joint3 = self.current_joint3
        arm_command_msg.joint4 = self.current_joint4
        arm_command_msg.joint5 = self.current_joint5
        arm_command_msg.joint6 = self.current_joint6

        self.arm_command_publisher.publish(arm_command_msg)

def main(args=None):
    rclpy.init(args=args)

    node = ArmNode()
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