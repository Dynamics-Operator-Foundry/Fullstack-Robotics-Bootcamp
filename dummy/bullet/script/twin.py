import pybullet as p
import pybullet_data
import time
import math

# Start physics simulation
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("../urdf/plane.urdf")
robot_id = p.loadURDF("../urdf/dummy.urdf", useFixedBase=True)

# Get joint info to check index
num_joints = p.getNumJoints(robot_id)
for i in range(num_joints):
    print(p.getJointInfo(robot_id, i))

# Assuming joint_1 is index 0 — replace if needed
joint_index = 2

angle_range = range(0, 10000, 1)  # degrees

while True:
    for deg in list(angle_range):
        # Your rotation logic here
        # print(deg)
        
        deg = 0
        rad = 0
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=0,
            controlMode=p.VELOCITY_CONTROL,
            force=0.0
        )
        
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=1,
            controlMode=p.VELOCITY_CONTROL,
            force=0.0
        )
        
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=2,
            controlMode=p.VELOCITY_CONTROL,
            force=0.0
        )
        p.stepSimulation()
        time.sleep(0.02)
        print("hi")

