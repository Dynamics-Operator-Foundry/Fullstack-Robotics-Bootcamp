import mujoco
import mujoco.viewer
import numpy as np
import time
import socket
import struct
import select

PORT = 60003
BUFFER_SIZE = 1024
fmt = 'dddd'  # 4 doubles

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', PORT))  # bind to all interfaces

# load model here
model = mujoco.MjModel.from_xml_path("../fredo1.xml")
data = mujoco.MjData(model)

sock.setblocking(False)

# viz here
def set_joint_angles(data, angles, joint_names=["joint_1", "joint_2", "joint_3"]):
    for joint_name, angle in zip(joint_names, angles):
        joint_id = model.joint(joint_name).qposadr # q-positions address
        data.qpos[joint_id] = angle # q-positions
        
    mujoco.mj_forward(model, data)  # forward kinematics -> not dynamics forwarding

# viewer here
with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.lookat[:] = np.array([0.04, -0.04, 0.08])     
    viewer.cam.distance = 0.64                       
    viewer.cam.azimuth = -40                        
    viewer.cam.elevation = -25         
    
    
    latest_data = None
    while True:
        ready = select.select([sock], [], [], 0)
        if ready[0]:
            try:
                latest_data, _ = sock.recvfrom(1024)
            except BlockingIOError:
                break
        else:
            break             
    
    while viewer.is_running():
        got_data = False
        while True:
            ready = select.select([sock], [], [], 0)
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
        
        # data_lala, addr = sock.recvfrom(1024)  
        if len(data_lala) == 32:      
            timelala, joint1, joint2, joint3 = struct.unpack(fmt, data_lala)
            print(f"Time: {timelala:.3f}, Joint1: {joint1:.3f}, Joint2: {joint2:.3f}, Joint3: {joint3:.3f}")
        else:
            print(f"Unexpected data size: {len(data_lala)} bytes")
        
        angles = [joint1 / 180 * np.pi, joint2 / 180 * np.pi, joint3 / 180 * np.pi]
        
        set_joint_angles(data, angles)
        viewer.sync()
        
        time.sleep(0.1)  