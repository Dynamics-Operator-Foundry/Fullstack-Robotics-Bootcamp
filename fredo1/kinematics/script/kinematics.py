from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt

import sys
import os
import numpy as np

from sympy import *

class utils:    
    def __init__(self):
        pass
    
    def rot3D(self, phi, theta, psi):
        R_x = np.array([
            [1, 0, 0],
            [0, cos(phi), -sin(phi)],
            [0, sin(phi), cos(phi)]
        ])
    
        R_y = np.array([
            [cos(theta), 0, sin(theta)],
            [0, 1, 0],
            [-sin(theta), 0, cos(theta)]
        ])
    
        R_z = np.array([
            [cos(psi), -sin(psi), 0],
            [sin(psi), cos(psi), 0],
            [0, 0, 1]
        ])
    
        return R_z @ R_y @ R_x
    
    def homo3D(self, phi, theta, psi, translation=np.array([0,0,0])):
        R = self.rot3D(phi, theta, psi)
        T_SE3 = np.eye(4)
        T_SE3[:3, :3] = R
        T_SE3[:3, 3] = translation
        
        return T_SE3

class kinematics(utils):
    def __init__(self):
        super().__init__()
        
    def forward(self, q=np.array([0,0,0])):
        q1 = q[0]
        q2 = q[1]
        q3 = q[2]
        
        TSE3_l1_2_I = self.homo3D(
            phi=0,
            theta=0,
            psi=0,
            translation=np.array([0,0,0])
        )
        
        TSE3_l2_2_l1 = self.homo3D(
            phi=0,
            theta=0,
            psi=q1,
            translation=np.array([0.01916, -0.05981, 0.06072])
        )
        
        TSE3_l3_2_l2 = self.homo3D(
            phi=q2,
            theta=0,
            psi=0,
            translation=np.array([-0.0299, -0.0105, 0.0335])
        )
        
        TSE3_l4_2_l3 = self.homo3D(
            phi=q3,
            theta=0,
            psi=0,
            translation=np.array([-0.0014, -0.01042, 0.096,])
        )                        
        
        return TSE3_l1_2_I @ TSE3_l2_2_l1 @ TSE3_l3_2_l2 @ TSE3_l4_2_l3 @ np.array([0.02849, 0.00007, 0.1100, 1])

kine = kinematics()

print(kine.forward(q=np.array([0, 1.57, 0])))

# if __name__ == "__main__":
#     forward = 0