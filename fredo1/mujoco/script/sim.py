import mujoco
import mujoco.viewer
import numpy as np
import time
import socket
import struct
import select
import csv
import os
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
            translation=np.array([-0.0014, -0.01042, 0.096])
        )                        
        
        return TSE3_l1_2_I @ TSE3_l2_2_l1 @ TSE3_l3_2_l2 @ TSE3_l4_2_l3 @ np.array([0.02849, 0.00007, 0.110, 1])


PORT = 60003
BUFFER_SIZE = 1024
fmt = 'dddd'  # 4 doubles

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', PORT))  # bind to all interfaces

model = mujoco.MjModel.from_xml_path("../fredo1.xml")
data = mujoco.MjData(model)
ee_site_id = model.site("ee_marker").id


sock.setblocking(False)

kine = kinematics()

csv_filename = "../log/joint_data_log.csv"
if os.path.exists(csv_filename):
    os.remove(csv_filename)
    
# with open(csv_filename, mode='w', newline='') as csvfile:
#     csv_writer = csv.writer(csvfile)
#     csv_writer.writerow(["time", "joint1_deg", "joint2_deg", "joint3_deg", "ee_x", "ee_y", "ee_z"])  # header

def set_joint_angles(time, data, angles, csv_writer, csvfile, joint_names=["joint_1", "joint_2", "joint_3"]):
    for joint_name, angle in zip(joint_names, angles):
        joint_id = model.joint(joint_name).qposadr
        data.qpos[joint_id] = angle
    mujoco.mj_forward(model, data)

    r_e = kine.forward(q=np.array([angles[0], angles[1], angles[2]]))
    
    data.site_xpos[ee_site_id] = r_e[:3]
    
    # csv recoding
    csv_writer.writerow([time, angles[0], angles[1], angles[2], r_e[0], r_e[2], r_e[2]])
    csvfile.flush() 

def sim():
    with open(csv_filename, mode='w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(["time", "joint1_deg", "joint2_deg", "joint3_deg", "ee_x", "ee_y", "ee_z"])  # header

        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.lookat[:] = np.array([0.04, -0.04, 0.08])     
            viewer.cam.distance = 0.64
            viewer.cam.azimuth = -40
            viewer.cam.elevation = -25

            
            while viewer.is_running():
                got_data = False
                while True:
                    ready = select.select([sock], [], [], 0.001)
                    if ready[0]:
                        got_data = True
                        try:
                            data_lala, _ = sock.recvfrom(1024)
                        except BlockingIOError:
                            break
                    else:
                        break  

                if not got_data:
                    continue

                if len(data_lala) == 32:      
                    timelala, joint1, joint2, joint3 = struct.unpack(fmt, data_lala)
                    print(f"Time: {timelala:.3f}, Joint1: {joint1:.3f}, Joint2: {joint2:.3f}, Joint3: {joint3:.3f}")                                                
                    
                    angles = [joint1 / 180 * np.pi, joint2 / 180 * np.pi, joint3 / 180 * np.pi]
                    # time, data, angles, csv_writer, csvfile, joint_names
                    set_joint_angles(timelala, data, angles, csv_writer, csvfile)
                    viewer.sync()
                    time.sleep(0.01)
                else:
                    print(f"Unexpected data size: {len(data_lala)} bytes")
    

if __name__ == "__main__":
    sim()