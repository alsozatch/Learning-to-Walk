import pybullet as pb
import pybullet_data as pb_data
from time import sleep

connectionId = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pb_data.getDataPath())
planeId = pb.loadURDF("plane.urdf")
pb.setGravity(0,0,-9.81)

walkbotId = pb.loadURDF("walkbot.urdf")

pb.setJointMotorControl2(walkbotId, 1, pb.VELOCITY_CONTROL, targetVelocity=-0.4)
for _ in range(400):
    print(pb.getBasePositionAndOrientation(walkbotId)[0])
    pb.stepSimulation()

pb.setJointMotorControl2(walkbotId, 0, pb.VELOCITY_CONTROL, targetVelocity=0.4)
pb.setJointMotorControl2(walkbotId, 1, pb.VELOCITY_CONTROL, targetVelocity=-0.1)
for _ in range(1000):
    print(pb.getBasePositionAndOrientation(walkbotId)[0])
    pb.stepSimulation()
