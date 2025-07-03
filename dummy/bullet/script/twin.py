import pybullet as p
import pybullet_data
import time

# Start physics simulation
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")
robot_id = p.loadURDF("../urdf/dummy.urdf", useFixedBase=True)

# Print joint info
for i in range(p.getNumJoints(robot_id)):
    print(p.getJointInfo(robot_id, i))

# Main loop
while True:
    # Set velocity for joint 0
    p.setJointMotorControl2(
        bodyUniqueId=robot_id,
        jointIndex=0,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=4.0,
        force=0.1
    )
    # Set velocity for joint 1
    p.setJointMotorControl2(
        bodyUniqueId=robot_id,
        jointIndex=1,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=-1.0,
        force=0.1
    )

    p.stepSimulation()
    time.sleep(0.02)
