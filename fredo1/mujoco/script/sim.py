import mujoco
import mujoco.viewer
import numpy as np
import time
import socket
import struct
import select
import csv
import os

PORT = 60003
BUFFER_SIZE = 1024
fmt = 'dddd'  # 4 doubles

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', PORT))  # bind to all interfaces

model = mujoco.MjModel.from_xml_path("../fredo1.xml")
data = mujoco.MjData(model)

sock.setblocking(False)

def set_joint_angles(data, angles, joint_names=["joint_1", "joint_2", "joint_3"]):
    for joint_name, angle in zip(joint_names, angles):
        joint_id = model.joint(joint_name).qposadr
        data.qpos[joint_id] = angle
    mujoco.mj_forward(model, data)


csv_filename = "../log/joint_data_log.csv"
if os.path.exists(csv_filename):
    os.remove(csv_filename)

def sim():
    with open(csv_filename, mode='w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(["time", "joint1_deg", "joint2_deg", "joint3_deg"])  # header

        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.lookat[:] = np.array([0.04, -0.04, 0.08])     
            viewer.cam.distance = 0.64
            viewer.cam.azimuth = -40
            viewer.cam.elevation = -25

            try:
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
                        
                        # csv recoding
                        # csv_writer.writerow([timelala, joint1, joint2, joint3])
                        # csvfile.flush()  # Optional: ensures data is written immediately
                        
                        angles = [joint1 / 180 * np.pi, joint2 / 180 * np.pi, joint3 / 180 * np.pi]
                        set_joint_angles(data, angles)
                        viewer.sync()
                        time.sleep(0.01)
                    else:
                        print(f"Unexpected data size: {len(data_lala)} bytes")
            finally:
                print("EXIT")

if __name__ == "__main__":
    sim()