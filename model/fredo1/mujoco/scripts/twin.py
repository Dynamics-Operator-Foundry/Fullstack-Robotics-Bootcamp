import mujoco
import mujoco.viewer
import numpy as np
import time

# load model here
model = mujoco.MjModel.from_xml_path("fredo1.xml")
data = mujoco.MjData(model)

# viz here
def set_joint_angles(data, angles, joint_names=["joint_1", "joint_2", "joint_3"]):
    for joint_name, angle in zip(joint_names, angles):
        joint_id = model.joint(joint_name).qposadr # q-positions address
        data.qpos[joint_id] = angle # q-positions
        
    mujoco.mj_forward(model, data)  # forward kinematics -> not dynamics forwarding

# viewer here
with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.lookat[:] = np.array([0.04, 0, 0.06])     # Look at origin
    viewer.cam.distance = 0.64                       # Distance from origin
    viewer.cam.azimuth = 60                        # Rotate around y-axis (horizontal)
    viewer.cam.elevation = -25                      # Tilt down 30 degrees
    
    while viewer.is_running():
        t = time.time()
        angles = [-1.57, -1.57, 0]
        set_joint_angles(data, angles)

        viewer.sync()
        time.sleep(0.01)  # Control the refresh rate
