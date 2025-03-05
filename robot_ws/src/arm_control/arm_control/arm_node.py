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

        self.speed = 3

        self.a_debounce = True
        self.b_debounce = True

        
    def xbox_callback(self, msg):
        if msg.a == 1 and self.speed < 3 and self.a_debounce:
            self.speed += 1
            self.a_debounce = False
        if msg.a == 0:
            self.a_debounce = True
        if msg.b == 1 and self.speed > 0 and self.b_debounce:
            self.speed -= 1
            self.b_debounce = False
        if msg.r_bumper == 0:
            self.b_debounce = True

        # Set joint angles
        self.current_joint1 += msg.l_stick_lr*p.MAX_ARM_SPEED*p.ARM_SPEEDS[self.speed]
        self.current_joint2 += msg.l_stick_ud*p.MAX_ARM_SPEED*p.ARM_SPEEDS[self.speed]
        self.current_joint3 += msg.r_stick_ud*p.MAX_ARM_SPEED*p.ARM_SPEEDS[self.speed]
        self.current_joint4 += msg.d_pad_ud*p.MAX_ARM_SPEED*p.ARM_SPEEDS[self.speed]

        if msg.r_trigger > 0:
            self.current_joint5 += msg.r_trigger*p.MAX_ARM_SPEED*p.ARM_SPEEDS[self.speed]
        elif msg.l_trigger > 0:
            self.current_joint5 -= msg.l_trigger*p.MAX_ARM_SPEED*p.ARM_SPEEDS[self.speed]

        if msg.r_bumper > 0:
            self.current_joint6 += msg.r_bumper*p.MAX_ARM_SPEED*p.ARM_SPEEDS[self.speed]
        elif msg.l_bumper > 0:
            self.current_joint6 -= msg.l_bumper*p.MAX_ARM_SPEED*p.ARM_SPEEDS[self.speed]


        # Limit joint angles
        self.current_joint1 = float(max(min(self.current_joint1, p.JOINT1_LIMITS[1]), p.JOINT1_LIMITS[0]))
        self.current_joint2 = float(max(min(self.current_joint2, p.JOINT2_LIMITS[1]), p.JOINT2_LIMITS[0]))
        self.current_joint3 = float(max(min(self.current_joint3, p.JOINT3_LIMITS[1]), p.JOINT3_LIMITS[0]))
        self.current_joint4 = float(max(min(self.current_joint4, p.JOINT4_LIMITS[1]), p.JOINT4_LIMITS[0]))
        self.current_joint5 = float(max(min(self.current_joint5, p.JOINT5_LIMITS[1]), p.JOINT5_LIMITS[0]))
        self.current_joint6 = float(max(min(self.current_joint6, p.JOINT6_LIMITS[1]), p.JOINT6_LIMITS[0]))

        # Create and publish ArmCommand message
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