from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt

import sys
import os
import numpy as np

from sympy import *

class utils:    
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

T_I20 = np.array([[1, 2], [3, 4]])
lala = np.array([[1], [2]])

print(T_I20 @ lala)

print(T_I20)

if __name__ == "__main__":
    forward = 0