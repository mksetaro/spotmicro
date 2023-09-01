"""
Simulation of SpotMicroAI and it's Kinematics 

"""
import time
import simulation.spotmicroai as spotmicro
import os

urdfPath = os.path.dirname(__file__) + "/robot_model/spot.xml"
print(urdfPath)
robot=spotmicro.Robot(urdf=urdfPath)

for i in range (10000):
    robot.step()
    time.sleep(1./240.)
# physicsClient = p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.setGravity(0,0,-10)
# planeId = p.loadURDF("plane.urdf")
# cubeStartPos = [0,0,1]
# cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
# boxId = p.loadURDF("robot-model/spot.urdf.xml",cubeStartPos, cubeStartOrientation)
# print(p.getJointInfo(boxId, 0))
# for i in range (10000):
#     p.stepSimulation()
#     time.sleep(1./240.)
# cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
# print(cubePos,cubeOrn)

# p.disconnect()