#!/usr/bin/env python
"""
General visual servoing class to perform either image based visual servoing (ibvs) or 
pose based visual servoing (pbvs). Currently only eye in hand (eih) methods are supported,
although eye to hand (eth) methods are easily applied by applying the transformation from 
the camera (eye) to the hand to the velocity twist vector.
"""
import numpy as np
from mobility import parameters as p

class VisualServoing(object):
    """
    General visual servoing class to perform either image based visual servoing (ibvs) or 
    pose based visual servoing (pbvs). Currently only eye in hand (eih) methods are supported,
    although eye to hand (eth) methods are easily applied by applying the transformation from 
    the camera (eye) to the hand to the velocity twist vector.
    """
    def __init__(self):
        self._translation_only=False

        self._L=np.zeros((2*4,6))
        self._ideal_feature=np.zeros((4*2,1))

        self.error = 1
        # Gain on controller, essentially sets arm speed, although too high of a value will cause the
        # function to diverge.
        self._lambda=3

        self._target_set=False
        
    def set_target(self,ideal_cam_pose=None, ideal_cam_rot=None,ideal_corners=None):
        """
        Sets the target position for the visual servoing law. The pose inputs are in homogeneous coordinates.
        While the corner positions aren't necessary for pbvs, they are still used to draw the desired position
        in the image.
        """
        self._ideal_cam_pose=ideal_cam_pose
        self._ideal_cam_rot = ideal_cam_rot
        if ideal_corners is not None:
            self._ideal_corners = ideal_corners

        self._eih_initialize_target_feature()
        self._target_set=True
 

    def _eih_initialize_target_feature(self):
        """
        In the event of ibvs eih servoing, initialize the interaction matrix (L) based on
        the desired position. The same L matrix will be used as an approximation to the true
        L throughout the servoing process (so that we don't have to reestimate the depth Z
        at each step). While estimating the depth is possible with the tags, it is useful to
        experiment with a constant interaction matrix regardless.
        """
        for i in range(0,4):
            x=self._ideal_corners[i*2]
            y=self._ideal_corners[i*2+1]                     
            self._ideal_feature[i*2,0]=x
            self._ideal_feature[i*2+1,0]=y
            p = self._ideal_cam_pose
            Z=p[2]
            self._L[i*2:i*2+2,:]=np.matrix([[-1/Z,0,x/Z,x*y,-(1+x*x),y],[0,-1/Z,y/Z,1+y*y,-x*y,-x]])

    def _calc_L(self, corners, depths):
        L=np.zeros((2*4,6))
        for i in range(0,4):
            x=corners[i*2]
            y=corners[i*2+1]                     
            Z=depths[i]
            L[i*2:i*2+2,:]=np.matrix([[-1/Z,0,x/Z,x*y,-(1+x*x),y],[0,-1/Z,y/Z,1+y*y,-x*y,-x]])
        return L
    
        
    def get_next_vel(self,t=None,R=None,corners=None, depths=None):
        """
        Computes the servo law mandated velocity given a current pose or set of image coordinates.
        At least one of either t and R or corners must be input.
        """
        if (t is None or R is None) and corners is None:
            return
        
        target_feature = corners.flatten()
        target_feature = target_feature[:,None]
        L = self._calc_L(corners, depths)
    
        error = target_feature - self._ideal_feature
        
        self.error = error
        new_L = L + self._L
        vel=-self._lambda*np.matmul((np.linalg.inv(new_L.T @ new_L)@new_L.T)/2,self.error)
        
        return vel