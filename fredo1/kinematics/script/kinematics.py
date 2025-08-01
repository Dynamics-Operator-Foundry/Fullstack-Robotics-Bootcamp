import numpy as np
import sympy as sp
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
    
    # def dh_homo(self, )

class kinematics(utils):
    def __init__(self):
        super().__init__()
        return
        
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
        
        return (TSE3_l1_2_I @ TSE3_l2_2_l1 @ TSE3_l3_2_l2 @ TSE3_l4_2_l3 @ np.array([0.02849, 0.00007, 0.1100, 1]))[:3]
    
    def inverse(self, q0, r_ref):
        q_k = q0
        dq_norm = np.inf
        
        # dr = J dq
        while dq_norm > 1e-6:
            
            J = self.get_Jacobian(q_k)
            
            dr = r_ref - self.forward(q_k)
            # print(np.linalg.norm(dr))
            # print(dq_norm)
            # print()
                    
            # print(J)
            J = np.array(J, dtype=np.float64)
            dq = np.linalg.pinv(J) @ dr
            q_k = q_k + dq
            # save_for_viz(q_k)
                    
            dq_norm = np.linalg.norm(dq)
            
            print(dq_norm)
            
        # return q_k
        return (q_k + np.pi) % (2 * np.pi) - np.pi

    def get_Jacobian(self, q=np.array([0,0,0])):
        q1 = q[0]
        q2 = q[1]
        q3 = q[2]
        
        return np.array([[0.00281*sin(q1) + 0.096*sin(q2)*cos(q1) + 0.11*sin(q2 + q3)*cos(q1) + 0.01042*cos(q1)*cos(q2) - 7.0e-5*cos(q1)*cos(q2 + q3) + 0.0105*cos(q1), (-0.01042*sin(q2) + 7.0e-5*sin(q2 + q3) + 0.096*cos(q2) + 0.11*cos(q2 + q3))*sin(q1), (7.0e-5*sin(q2 + q3) + 0.11*cos(q2 + q3))*sin(q1)], [0.096*sin(q1)*sin(q2) + 0.11*sin(q1)*sin(q2 + q3) + 0.01042*sin(q1)*cos(q2) - 7.0e-5*sin(q1)*cos(q2 + q3) + 0.0105*sin(q1) - 0.00281*cos(q1), (0.01042*sin(q2) - 7.0e-5*sin(q2 + q3) - 0.096*cos(q2) - 0.11*cos(q2 + q3))*cos(q1), -(7.0e-5*sin(q2 + q3) + 0.11*cos(q2 + q3))*cos(q1)], [0, -0.096*sin(q2) - 0.11*sin(q2 + q3) - 0.01042*cos(q2) + 7.0e-5*cos(q2 + q3), -0.11*sin(q2 + q3) + 7.0e-5*cos(q2 + q3)]])
    
    def dh_forward(self):
        
        return                            