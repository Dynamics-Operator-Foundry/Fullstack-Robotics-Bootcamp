import pybullet as p
import pybullet_data
import time
import math

status = p.connect(p.GUI)

p.setGravity(0, 0, -9.81)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setTimeStep(1.0 / 100.0)

p.loadURDF("../urdf/plane.urdf")
robot_id = p.loadURDF("../urdf/fredo1.urdf", useFixedBase=True)

num_joints = p.getNumJoints(robot_id)
for i in range(num_joints):
    print(p.getJointInfo(robot_id, i))

joint_index = 2

angle_range = range(0, 10000, 1) 

while True:
    for deg in list(angle_range):
        
        deg = 0
        rad = 0
        for joint_id in range(p.getNumJoints(robot_id)):
            p.setJointMotorControl2(
                robot_id, joint_id,
                controlMode=p.VELOCITY_CONTROL,
                force=0  # This disables motor control
            )
        p.stepSimulation()
        time.sleep(0.01)
        print("hi")

