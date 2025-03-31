#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from robot_msgs.msg import Xbox
from robot_msgs.msg import ArmCommand, Mode
from roboticstoolbox import DHRobot
from mobility import parameters as p
import numpy as np

class ApriltagClient(object):
    
    def __init__(self):
        self.tf_subscription = self.create_subscription(Mode, '/tf', self.process_detection, 10)

        self.corners = None
        self.depths = None
        

    def process_detection(self,apriltag_detections):
        """
        Process income apriltag detections message, store pose and corners if detections were found.
        """        

        # Get best marker for gripper pose
        (my_marker,marker_pos)=self.__get_marker(apriltag_detections.detections)

        if my_marker is None:
            return

        size = apriltag_detections.detections[marker_pos].size
        
        corners, depths = self.compute_tag_corners(apriltag_detections.detections[marker_pos].pose.pose, size)
        self.corners = corners
        self.depths = depths

    def compute_tag_corners(self, tag_pose, tag_size):
        """
        Compute the corners of the AprilTag in the camera frame and project them onto the image plane.

        Args:
        - tag_pose: Pose of the tag in the camera frame (geometry_msgs/Pose).
        - tag_size: Size of the tag (length of one side).
        - camera_matrix: The intrinsic camera matrix.

        Returns:
        - A list of projected 2D corners of the tag in the image frame.
        """
        # Tag corners in the tag's local frame (3D coordinates)
        half_size = tag_size / 2.0
        tag_corners_local = np.array([
            [-half_size, -half_size, 0],  # corner 0 (top-left)
            [half_size, -half_size, 0],   # corner 1 (top-right)
            [half_size, half_size, 0],    # corner 2 (bottom-right)
            [-half_size, half_size, 0],   # corner 3 (bottom-left)
        ])
        #print(tag_corners_local)
        # Convert the tag's rotation quaternion into a rotation matrix
        q = [tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z, tag_pose.orientation.w]
        rotation_matrix = np.quaternion_matrix(q)[:3, :3]  # 3x3 rotation matrix
        
        # Tag's translation vector (position in camera frame)
        translation = np.array([tag_pose.position.x, tag_pose.position.y, tag_pose.position.z])
        
        # Compute the corners in the camera frame
        tag_corners_camera = []
        corners_2d = np.zeros(8)
        depths = np.zeros(4)
        counter = 0
        index = 0
        for corner in tag_corners_local:
            # Apply the rotation and translation to the local corners
            corner_camera = np.dot(rotation_matrix, corner) + translation
            tag_corners_camera.append(corner_camera)
            
            
            corners_2d[counter] = corner_camera[0]/corner_camera[2]
            corners_2d[counter+1] = corner_camera[1]/corner_camera[2]
            
            depths[index] = corner_camera[2]
            counter += 2
            index += 1
        
        # Project the corners onto the image plane
        #corners_2d = self.project_to_image_plane(tag_corners_camera, camera_matrix)
        
        return corners_2d, depths
