import pybullet as p
import pybullet_data

client = p.connect(p.GUI)
p.setGravity(0,0,-10, physicsClientId=client)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeID = p.loadURDF("plane.urdf")
carID = p.loadURDF("racecar/racecar.urdf", basePosition=[0,0,1])
# cubeID = p.loadURDF("cube_collisionfilter.urdf", [0,0,3], useMaximalCoordinates=False)

p.setCollisionFilterGroupMask(carID, -1, 0,0 )

enableCol = 1
p.setCollisionFilterPair(planeID, carID, -1, -1, enableCol)

# pos, orien = p.getBasePositionAndOrientation(carID)

for _ in range(3000):
    pos, orien = p.getBasePositionAndOrientation(carID)
    p.applyExternalForce(carID, 0, [40,0,0], pos, p.WORLD_FRAME)
    p.stepSimulation()