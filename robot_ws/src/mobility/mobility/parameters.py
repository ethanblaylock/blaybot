from roboticstoolbox import RevoluteDH, DHRobot
import numpy as np
import matplotlib.pyplot as plt

MAX_PWM_COUNTS = 255
DRIVE_SPEEDS = [0.25,0.5,0.75,1]
MAX_ARM_SPEED = 10.0
ARM_SPEEDS = [0.25,0.5,0.75,1]




# Intial joint angles
INIT_JOINT1 = 2500
INIT_JOINT2 = 3000
INIT_JOINT3 = 3000
INIT_JOINT4 = 3100
INIT_JOINT5 = 3000
INIT_JOINT6 = 3500
# Joint limits
JOINT1_LIMITS = [1000, 5000]
JOINT2_LIMITS = [2800, 4700]
JOINT3_LIMITS = [1000, 4000]
JOINT4_LIMITS = [1000, 5000]
JOINT5_LIMITS = [1000, 5000]
JOINT6_LIMITS = [3020, 4000]


# Tuck joint angles
TUCK_JOINT1 = 2500
TUCK_JOINT2 = 4800
TUCK_JOINT3 = 3900
TUCK_JOINT4 = 3230
TUCK_JOINT5 = 3000
TUCK_JOINT6 = 3500



# Define the robot
d0 = -0.0635
alpha0 = np.pi/2


a1 = 0.2287
alpha1 = np.pi

a2 = 0.2033
alpha2 = np.pi

theta3 = -np.pi/2
d3 = -0.01918
alpha3 = np.pi/2



d4 = -0.1397 - 0.055



dh_params = [
    RevoluteDH(d=d0, alpha=alpha0),
    RevoluteDH(a=a1, alpha=alpha1),
    RevoluteDH(a=a2, alpha=alpha2),
    RevoluteDH(offset=theta3, d=d3, alpha=alpha3),
    RevoluteDH(d=d4)
]

INIT_Q = [0,-np.pi/2,-3*np.pi/4,np.pi/4,0]

if __name__ == '__main__':
    robot = DHRobot(dh_params)
    print(robot)
    robot.plot([0,-np.pi/2,-3*np.pi/4,np.pi/4,0])
    plt.show()
    input("Press Enter to close the plot...")
    print(robot.fkine([0,0,0,0,0]))