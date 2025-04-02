from roboticstoolbox import RevoluteDH, DHRobot
import numpy as np
import matplotlib.pyplot as plt

MAX_PWM_COUNTS = 255
DRIVE_SPEEDS = [0.25,0.5,0.75,1]
MAX_ARM_SPEED = 10.0
ARM_SPEEDS = [0.25,0.5,0.75,1]


# joint1: 2508.1162109375
# joint2: 3663.258544921875
# joint3: 3438.89208984375
# joint4: 2900.0
# joint5: 2896.42041015625
# joint6: 3020.0


# Intial joint angles
INIT_JOINT1 = 2500
INIT_JOINT2 = 3660
INIT_JOINT3 = 3430
INIT_JOINT4 = 2900
INIT_JOINT5 = 2900
INIT_JOINT6 = 3500
# Joint limits
JOINT1_LIMITS = [1000, 5000]
JOINT2_LIMITS = [2800, 4900]
JOINT3_LIMITS = [1000, 4000]
JOINT4_LIMITS = [1000, 5000]
JOINT5_LIMITS = [1000, 5000]
JOINT6_LIMITS = [3020, 4000]


# Tuck joint angles
TUCK_JOINT1 = 2500
TUCK_JOINT2 = 4900
TUCK_JOINT3 = 4000
TUCK_JOINT4 = 3300
TUCK_JOINT5 = 3000
TUCK_JOINT6 = 3500



# Define the robot
d0 = 0.0635
alpha0 = np.pi/2


a1 = 0.2287
alpha1 = np.pi

a2 = 0.2033
alpha2 = np.pi

theta3 = -np.pi/2
d3 = 0.01918
alpha3 = -np.pi/2



d4 = 0.1397 + 0.055



dh_params = [
    RevoluteDH(d=d0, alpha=alpha0),
    RevoluteDH(a=a1, alpha=alpha1),
    RevoluteDH(a=a2, alpha=alpha2),
    RevoluteDH(offset=theta3, d=d3, alpha=alpha3),
    RevoluteDH(d=d4)
]

INIT_Q = [0,np.pi/2,3*np.pi/4,-np.pi/4,0]


KD = 0.1

# K = [1213.69694,    0.  
#    ,  739.09721,
#             0.     , 1211.13729,  622.62561,
#             0.     ,    0.     ,    1.     ]

CAMERA_FRAME = np.array([
    [1., 0., 0., 0.],
    [0., 1., 0., 0.],
    [0., 0., 1., 0.],
    [0., 0., 0., 1.]
])
if __name__ == '__main__':
    robot = DHRobot(dh_params)
    print(robot)
    robot.plot(INIT_Q)
    plt.show()
    input("Press Enter to close the plot...")
    print(robot.fkine([0,0,0,0,0]))