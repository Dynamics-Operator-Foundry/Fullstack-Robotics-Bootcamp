from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import sympy as sp

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
    
    def homo3D(self, phi, theta, psi, translation=np.array([0,0,0]), symb=False):
        R = self.rot3D(phi, theta, psi)
        
        if not symb:
            T_SE3 = np.eye(4)
            T_SE3[:3, :3] = R
            T_SE3[:3, 3] = translation
        else:
            T_SE3 = sp.Matrix.eye(4)
            T_SE3[:3, :3] = R
            T_SE3[:3, 3] = sp.Matrix(translation)
        
        
        return T_SE3

util = utils()

# get homo-transformation from l1 to I
TSE3_l1_2_I = util.homo3D(
    phi=0,
    theta=0,
    psi=0,
    translation=np.array([0,0,0])
)
print(TSE3_l1_2_I)

# express l1 end in I
l1end_in_I = TSE3_l1_2_I @ np.array([0.01916, -0.05981, 0.06072, 1])
print(l1end_in_I)

# get homo-transformation from l2 to l1
TSE3_l2_2_l1 = util.homo3D(
    phi=0,
    theta=0,
    psi=0,
    translation=np.array([0.01916, -0.05981, 0.06072])
)
print(TSE3_l2_2_l1)

# express l2 end in I
l2end_in_I = TSE3_l1_2_I @ TSE3_l2_2_l1 @ np.array([-0.0299, -0.0105, 0.0335, 1])
print(l2end_in_I)

# get homo-transformation from l3 to l2
TSE3_l3_2_l2 = util.homo3D(
    phi=0,
    theta=0,
    psi=0,
    translation=np.array([-0.0299, -0.0105, 0.0335])
)
print(TSE3_l3_2_l2)

# express l3 end in I
l3end_in_I = TSE3_l1_2_I @ TSE3_l2_2_l1 @ TSE3_l3_2_l2 @ np.array([-0.0014, -0.01042, 0.096, 1])
print(l3end_in_I)

# get homo-transformation from l4 to l3
TSE3_l4_2_l3 = util.homo3D(
    phi=0,
    theta=0,
    psi=0,
    translation=np.array([-0.0014, -0.01042, 0.096])
)
print(TSE3_l4_2_l3)

# express l4 end in I
l4end_in_I = TSE3_l1_2_I @ TSE3_l2_2_l1 @ TSE3_l3_2_l2 @ TSE3_l4_2_l3 @ np.array([0.02849, 0.00007, 0.0400, 1])
print(l4end_in_I)


############################################################
# derive Jacobian
print("=======================================================")
q1, q2, q3 = sp.symbols('q1, q2, q3', real=True)

 
# derive Jacobians here
TSE3_l1_2_I = util.homo3D(
    phi=0,
    theta=0,
    psi=0,
    translation=np.array([0,0,0])
)

TSE3_l2_2_l1 = util.homo3D(
    phi=0,
    theta=0,
    psi=q1,
    translation=np.array([0.01916, -0.05981, 0.06072]),
    symb=True
)


TSE3_l3_2_l2 = util.homo3D(
    phi=q2,
    theta=0,
    psi=0,
    translation=np.array([-0.0299, -0.0105, 0.0335]),
    symb=True
)

TSE3_l4_2_l3 = util.homo3D(
    phi=q3,
    theta=0,
    psi=0,
    translation=np.array([-0.0014, -0.01042, 0.096]),
    symb=True
)

r_E = TSE3_l1_2_I @ TSE3_l2_2_l1 @ TSE3_l3_2_l2 @ TSE3_l4_2_l3 @ np.array([0.02849, 0.00007, 0.1100, 1])
r_E = sp.Matrix(r_E[0:3])

# print(r_E.rows)
# print(r_E.cols)
q = [q1, q2, q3]
J = sp.simplify(r_E.jacobian(q))
print(J.rows)
print(J.cols)

print()
print(J)
