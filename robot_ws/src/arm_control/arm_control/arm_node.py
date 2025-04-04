import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from robot_msgs.msg import Xbox
from robot_msgs.msg import ArmCommand, Mode
from tf2_msgs.msg import TFMessage
from roboticstoolbox import DHRobot
from mobility import parameters as p
import numpy as np

from arm_control.visual_servoing import VisualServoing
from arm_control.apriltag_client import ApriltagClient


class ArmNode(Node):
    
    def __init__(self):
        super().__init__('arm_node')
        self.xbox_subscription = self.create_subscription(Xbox, '/xbox', self.xbox_callback, 10)

        self.arm_command_publisher = self.create_publisher(ArmCommand, '/arm_command', 10)

        self.mode_subscription = self.create_subscription(Mode, '/mode', self.mode_callback, 10)

        

        self.arm_dh_model = DHRobot(p.dh_params, name='arm')
        self.arm_dh_model.q = p.INIT_Q

        self.current_joint1 = p.TUCK_JOINT1
        self.current_joint2 = p.TUCK_JOINT2
        self.current_joint3 = p.TUCK_JOINT3
        self.current_joint4 = p.TUCK_JOINT4
        self.current_joint5 = p.TUCK_JOINT5
        self.current_joint6 = p.TUCK_JOINT6
        self.move_to_joint_angles(p.INIT_JOINT1, p.INIT_JOINT2, p.INIT_JOINT3, p.INIT_JOINT4, p.INIT_JOINT5, p.INIT_JOINT6)

        self.speed = 3

        self.a_debounce = True
        self.b_debounce = True

        self.arm_enable = False

        self.apriltag_client = ApriltagClient()
        self.visual_servoing = VisualServoing()

        self.start_visual_servo = False
        self.visual_servo_ready = False

        self.tf_subscription = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        
    def tf_callback(self, msg):
        # Get best marker for gripper pose
        transforms = msg.transforms

        # check is empty
        if len(transforms) == 0:
            if self.visual_servo_ready and not self.start_visual_servo:
                self.get_logger().info('Move closer to the AprilTag to Visual Servo')
            self.visual_servo_ready = False
            return
            

        if not self.visual_servo_ready and not self.start_visual_servo:
            self.get_logger().info('Ready to Visual Servo')
            self.visual_servo_ready = True
        
        self.apriltag_client.process_detection(transforms)  


    def xbox_callback(self, msg):
        if not self.arm_enable:
            return
        if self.start_visual_servo:
            self.visual_servo()

        if msg.share == 1:
            self.get_logger().info('Tucking arm')
            self.tuck()
            self.get_logger().info('Tucking complete')
            return

        if msg.view == 1:
            self.get_logger().info('Untucking arm')
            self.untuck()
            self.get_logger().info('Untucking complete')
            return

        if msg.menu == 1:
            self.visual_servoing.error = 1
            self.open_gripper()
            self.visual_servo()
    
        if msg.y == 1:
            self.start_visual_servo = False

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
        # self.current_joint1 += msg.l_stick_lr*p.MAX_ARM_SPEED*p.ARM_SPEEDS[self.speed]
        # self.current_joint2 += msg.l_stick_ud*p.MAX_ARM_SPEED*p.ARM_SPEEDS[self.speed]
        # self.current_joint3 -= msg.r_stick_ud*p.MAX_ARM_SPEED*p.ARM_SPEEDS[self.speed]
        # self.current_joint4 += msg.d_pad_ud*p.MAX_ARM_SPEED*p.ARM_SPEEDS[self.speed]

        if msg.r_trigger > 0:
            self.current_joint5 += msg.r_trigger*p.MAX_ARM_SPEED*p.ARM_SPEEDS[self.speed]
        elif msg.l_trigger > 0:
            self.current_joint5 -= msg.l_trigger*p.MAX_ARM_SPEED*p.ARM_SPEEDS[self.speed]

        if msg.r_bumper > 0:
            self.current_joint6 += msg.r_bumper*p.MAX_ARM_SPEED*p.ARM_SPEEDS[self.speed]
        elif msg.l_bumper > 0:
            self.current_joint6 -= msg.l_bumper*p.MAX_ARM_SPEED*p.ARM_SPEEDS[self.speed]

        des_x_vel_ik = msg.l_stick_ud*p.MAX_ARM_SPEED*p.ARM_SPEEDS[self.speed]
        des_y_vel_ik = msg.l_stick_lr*p.MAX_ARM_SPEED*p.ARM_SPEEDS[self.speed]
        des_z_vel_ik = msg.r_stick_ud*p.MAX_ARM_SPEED*p.ARM_SPEEDS[self.speed]

        des_y_rot_ik = msg.d_pad_ud*p.MAX_ARM_SPEED*p.ARM_SPEEDS[self.speed]
        des_ee_twist = np.array([des_x_vel_ik, des_y_vel_ik, des_z_vel_ik, 0, des_y_rot_ik, 0])
        q = self.arm_dh_model.q
        J = self.arm_dh_model.jacob0(q)
        J_dagger = J.T @ np.linalg.inv(J @ J.T + p.KD**2 * np.eye(len(J)))
        q_dot = J_dagger @ des_ee_twist
        q_dot = np.clip(q_dot, -p.MAX_ARM_SPEED, p.MAX_ARM_SPEED)
        self.current_joint1 += q_dot[0]
        self.current_joint2 += q_dot[1]
        self.current_joint3 += q_dot[2]
        self.current_joint4 += q_dot[3]
        self.current_joint5 += q_dot[4]

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

        self.update_dh_model()
    
    def mode_callback(self, mode_msg):
        if mode_msg.mode == Mode.ARM:
            self.arm_enable = True
        else:
            self.arm_enable = False
    def tuck(self):
        self.move_to_joint_angles(p.TUCK_JOINT1, p.TUCK_JOINT2, p.TUCK_JOINT3, p.TUCK_JOINT4, p.TUCK_JOINT5, self.current_joint6)

    def untuck(self):
        self.move_to_joint_angles(p.INIT_JOINT1, p.INIT_JOINT2, p.INIT_JOINT3, p.INIT_JOINT4, p.INIT_JOINT5, self.current_joint6)

    def open_gripper(self):
        self.move_to_joint_angles(self.current_joint1, self.current_joint2, self.current_joint3, self.current_joint4, self.current_joint5, p.OPEN_GRIPPER)

    def close_gripper(self):
        self.move_to_joint_angles(self.current_joint1, self.current_joint2, self.current_joint3, self.current_joint4, self.current_joint5, p.CLOSE_GRIPPER)
    
    def update_dh_model(self):
        angle1 = (self.current_joint1 - 3000) / 848.826363 + 0.58905
        angle2 = (self.current_joint2 - 3000) / 848.826363 + 0.78972
        angle3 = (self.current_joint3 - 3000) / 848.826363 + 1.64933
        angle4 = (self.current_joint4 - 3000) / 848.826363 - 0.82467
        angle5 = (self.current_joint5 - 3000) / 848.826363 + 0.11781
        self.arm_dh_model.q = [angle1, angle2, angle3, angle4, angle5]

    def move_to_joint_angles(self, joint1, joint2, joint3, joint4, joint5, joint6):
        while True:
            if self.current_joint1 < joint1:
                self.current_joint1 += 1
            elif self.current_joint1 > joint1:
                self.current_joint1 -= 1
            if self.current_joint2 < joint2:
                self.current_joint2 += 1
            elif self.current_joint2 > joint2:
                self.current_joint2 -= 1
            if self.current_joint3 < joint3:
                self.current_joint3 += 1
            elif self.current_joint3 > joint3:
                self.current_joint3 -= 1
            if self.current_joint4 < joint4:
                self.current_joint4 += 1
            elif self.current_joint4 > joint4:
                self.current_joint4 -= 1
            if self.current_joint5 < joint5:
                self.current_joint5 += 1
            elif self.current_joint5 > joint5:
                self.current_joint5 -= 1
            if self.current_joint6 < joint6:
                self.current_joint6 += 1
            elif self.current_joint6 > joint6:
                self.current_joint6 -= 1
            
            # Limit joint angles
            self.current_joint1 = float(max(min(self.current_joint1, p.JOINT1_LIMITS[1]), p.JOINT1_LIMITS[0]))
            self.current_joint2 = float(max(min(self.current_joint2, p.JOINT2_LIMITS[1]), p.JOINT2_LIMITS[0]))
            self.current_joint3 = float(max(min(self.current_joint3, p.JOINT3_LIMITS[1]), p.JOINT3_LIMITS[0]))
            self.current_joint4 = float(max(min(self.current_joint4, p.JOINT4_LIMITS[1]), p.JOINT4_LIMITS[0]))
            self.current_joint5 = float(max(min(self.current_joint5, p.JOINT5_LIMITS[1]), p.JOINT5_LIMITS[0]))
            self.current_joint6 = float(max(min(self.current_joint6, p.JOINT6_LIMITS[1]), p.JOINT6_LIMITS[0]))

            self.update_dh_model()

            arm_command_msg = ArmCommand()
            arm_command_msg.joint1 = self.current_joint1
            arm_command_msg.joint2 = self.current_joint2
            arm_command_msg.joint3 = self.current_joint3
            arm_command_msg.joint4 = self.current_joint4
            arm_command_msg.joint5 = self.current_joint5
            arm_command_msg.joint6 = self.current_joint6
            self.arm_command_publisher.publish(arm_command_msg)
            if abs(self.current_joint1 - joint1) < 1 and abs(self.current_joint2 - joint2) < 1 and abs(self.current_joint3 - joint3) < 1 and abs(self.current_joint4 - joint4) < 1 and abs(self.current_joint5 - joint5) < 1 and abs(self.current_joint6 - joint6) < 1:
                break
            for i in range(50000):
                pass

    def visual_servo(self):
        if not self.start_visual_servo:
            self.get_logger().info('starting servoing')
            final_camera_depth = 4

            desired_corners = self.get_target_corners(final_camera_depth, 0.025)

            ideal_cam_pose = np.array([0,0,final_camera_depth])
            self.visual_servoing.set_target(ideal_cam_pose,None,ideal_corners=desired_corners)
            self.get_logger().info('target set')
            self.start_visual_servo = True
        
            
        if self.apriltag_client.corners is None:
            # self.get_logger().info('no corners')
            return
        
        marker_corners = self.apriltag_client.corners
        if marker_corners is None:
            self.get_logger().info('no marker corners')
            return
        
        # Don't move if the target hasn't been set
        if not self.visual_servoing._target_set:
            self.get_logger().info('target not set')
            return
        
        if np.linalg.norm(self.visual_servoing.error) < 0.1:
            self.get_logger().info('target reached')
            self.start_visual_servo = False
            self.close_gripper()
            self.untuck()
            return
        self.get_logger().info(f'error: {np.linalg.norm(self.visual_servoing.error)}')
        # Get control law velocity and transform to body frame, then send to robot
        twist = self.visual_servoing.get_next_vel(corners=marker_corners, depths=self.apriltag_client.depths)
        self.apriltag_client.corners = None
        
        new_twist = np.zeros(6)
        new_twist[0] = twist[1] # x is the y
        new_twist[1] = -twist[0] # y is negative x
        new_twist[2] = twist[2] # z is negative y
        new_twist[3] = twist[4]
        new_twist[4] = -twist[3]
        new_twist[5] = twist[5]
        twist = new_twist

        transform_matrix = self.arm_dh_model.fkine(self.arm_dh_model.q)
        rotation_matrix = transform_matrix.R
        Z_FN_to_FBase = np.vstack([np.hstack([rotation_matrix, np.zeros((3,3))]), np.hstack([np.zeros((3,3)), rotation_matrix])])
        
        R_Cam = p.CAMERA_FRAME[0:-1, 0:-1]
        P_Cam = p.CAMERA_FRAME[0:-1, -1]
        skewSymMatrix = self.skew(P_Cam)
        Z_P_FCam_to_FN = np.vstack([np.hstack([np.eye(R_Cam.shape[0]), -skewSymMatrix]), np.hstack([np.zeros((3,3)), np.eye(R_Cam.shape[0])])])
        Z_R_FCam_to_FN = np.vstack([np.hstack([R_Cam, np.zeros((3,3))]), np.hstack([np.zeros((3,3)), R_Cam])])
        
        Z_FCam_to_FN = Z_P_FCam_to_FN @ Z_R_FCam_to_FN

        Z_shift = Z_FN_to_FBase @ Z_FCam_to_FN

        twist = Z_shift @ twist
        q = self.arm_dh_model.q
        J = self.arm_dh_model.jacob0(q)
        # J[:, 4] = J[:, 4] * 5
        J_dagger = J.T @ np.linalg.inv(J @ J.T + p.KD**2 * np.eye(len(J)))
        q_dot = J_dagger @ twist
        
        q_dot = np.clip(q_dot, -p.MAX_ARM_SPEED, p.MAX_ARM_SPEED)
        # self.get_logger().info(f'q_dot: {q_dot}')
        self.current_joint1 += q_dot[0]
        self.current_joint2 += q_dot[1]
        self.current_joint3 += q_dot[2]
        self.current_joint4 += q_dot[3]
        self.current_joint5 += (q_dot[4])

        self.current_joint1 = float(max(min(self.current_joint1, p.JOINT1_LIMITS[1]), p.JOINT1_LIMITS[0]))
        self.current_joint2 = float(max(min(self.current_joint2, p.JOINT2_LIMITS[1]), p.JOINT2_LIMITS[0]))
        self.current_joint3 = float(max(min(self.current_joint3, p.JOINT3_LIMITS[1]), p.JOINT3_LIMITS[0]))
        self.current_joint4 = float(max(min(self.current_joint4, p.JOINT4_LIMITS[1]), p.JOINT4_LIMITS[0]))
        self.current_joint5 = float(max(min(self.current_joint5, p.JOINT5_LIMITS[1]), p.JOINT5_LIMITS[0]))

        self.update_dh_model()
        arm_command_msg = ArmCommand()
        arm_command_msg.joint1 = self.current_joint1
        arm_command_msg.joint2 = self.current_joint2
        arm_command_msg.joint3 = self.current_joint3
        arm_command_msg.joint4 = self.current_joint4
        arm_command_msg.joint5 = self.current_joint5
        arm_command_msg.joint6 = self.current_joint6
        self.arm_command_publisher.publish(arm_command_msg)

    def get_target_corners(self, final_camera_depth, size):
        corner = size/2
        corner = corner/final_camera_depth
        return np.array([-corner, corner, corner, corner, corner, -corner, -corner, -corner])

    def skew(self, v):
        return np.array([
            [0., -v[2], v[1]],
            [v[2], 0., -v[0]],
            [-v[1], v[0], 0.]
        ])
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