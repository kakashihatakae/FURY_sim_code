import pybullet as p
import time 
import math
import fury
import numpy as np

client = p.connect(p.GUI)
p.setGravity(0,0,-10, physicsClientId=client)
# p.resetSimulation() #?

# p.resetDebugVisualizerCamera(15, -346, -16, [0, -15, 1]) #?
# 
# p.resetDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

phi = (1 + math.sqrt(5)) / 2.0

icosahedron_vertices = np.array([[-1.0, 0.0, phi],
                                     [0.0, phi, 1.0],
                                     [1.0, 0.0, phi],
                                     [-phi, 1.0, 0.0],
                                     [0.0, phi, -1.0],
                                     [phi, 1.0, 0.0],
                                     [-phi, -1.0, 0.0],
                                     [0.0, -phi, 1.0],
                                     [phi, -1.0, 0.0],
                                     [-1.0, 0.0, -phi],
                                     [0.0, -phi, -1.0],
                                     [1.0, 0.0, -phi]])

icosahedron_mesh = np.array([[1, 0, 2],
                                 [2, 5, 1],
                                 [5, 4, 1],
                                 [3, 1, 4],
                                 [0, 1, 3],
                                 [0, 6, 3],
                                 [9, 3, 6],
                                 [8, 2, 7],
                                 [2, 0, 7],
                                 [0, 7, 6],
                                 [5, 2, 8],
                                 [11, 5, 8],
                                 [11, 4, 5],
                                 [9, 11, 4],
                                 [4, 3, 9],
                                 [11, 10, 8],
                                 [8, 10, 7],
                                 [6, 7, 10],
                                 [10, 9, 6],
                                 [9, 10, 11]], dtype='i8')

pyramid_vert = np.array([[0.5, 0.5, 0.5],
                         [0.5, -0.5, -0.5],
                         [-0.5, 0.5, -0.5],
                         [-0.5, -0.5, 0.5]])

pyramid_triag = np.array([[0,1,2],
                         [3, 0, 2],
                         [0, 3, 1],
                         [1, 2, 3]],dtype='i8')
                         
pyramid_color = [[ 19 ,20 ,96,0.5],
                 [ 21 ,20 ,96, 0.5],
                 [ 20 ,19 ,200, 0.5],
                 [ 20 ,19 ,200, 0.5]]

# objcollision = p.createCollisionShape(p.GEOM_MESH, vertices=pyramid_vert, indices=pyramid_triag.flatten()) #change vert, trian
# objvisual = p.createVisualShape(p.GEOM_MESH, 
#                                     vertices=pyramid_vert, 
#                                     indices=pyramid_triag.flatten(),
#                                     rgbaColor=[0.1, 0.2, 0.6, 0.9])
# obj = p.createMultiBody(baseMass=0.5,baseCollisionShapeIndex=objcollision,
#                   baseVisualShapeIndex=objvisual, 
#                   basePosition=[0, 0, 5],baseOrientation = [ -0.4044981, -0.8089962, -0.4044981, 0.1352322 ])

boxHalfLength = 0.1
boxHalfWidth = 5
boxHalfHeight = 5
wallcollision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[boxHalfLength, boxHalfWidth, boxHalfHeight])
wallvisual = p.createVisualShape(p.GEOM_BOX, halfExtents=[boxHalfLength, boxHalfWidth, boxHalfHeight])
wall = p.createMultiBody(baseCollisionShapeIndex=wallcollision,
                  baseVisualShapeIndex=wallvisual, 
                  basePosition=[-4, 0, 4])

boxHalfLength_1 = 5
boxHalfWidth_1 = 5
boxHalfHeight_1 = 0.1
wallcollision_1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[boxHalfLength_1, boxHalfWidth_1, boxHalfHeight_1])
wallvisual_1 = p.createVisualShape(p.GEOM_BOX, halfExtents=[boxHalfLength_1, boxHalfWidth_1, boxHalfHeight_1])
wall_1 = p.createMultiBody(baseCollisionShapeIndex=wallcollision_1,
                  baseVisualShapeIndex=wallvisual_1, 
                  basePosition=[0, 0, 0])

boxHalfLength_ = 0.1
boxHalfWidth_ = 0.1
boxHalfHeight_ = 0.5
objcollision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[boxHalfLength_, boxHalfWidth_, boxHalfHeight_])
objvisual = p.createVisualShape(p.GEOM_BOX, 
                                halfExtents=[boxHalfLength_, boxHalfWidth_, boxHalfHeight_], 
                                rgbaColor=[0.1, 0.2, 0.6, 1])
obj = p.createMultiBody(baseMass = 0.5,
                  baseCollisionShapeIndex=objcollision,
                  baseVisualShapeIndex=objvisual, 
                  basePosition=[0, 0, 4],
                  baseOrientation = [ -0.4044981, -0.8089962, -0.4044981, 0.1352322 ])

p.changeDynamics(obj, -1, lateralFriction=0.5)
p.changeDynamics(obj, -1, restitution=0.6)
# p.changeDynamics(pyramid, -1, mass=0.5)

# p.changeDynamics(pyramid, -1, lateralFriction=0.5)
# p.changeDynamics(pyramid, -1, restitution=0.5)
# p.changeDynamics(pyramid, -1, mass=0.5)

p.changeDynamics(wall, -1, lateralFriction=0.3)
p.changeDynamics(wall, -1, restitution=0.5)
# p.changeDynamics(wall, -1)


p.changeDynamics(wall_1, -1, lateralFriction=0.4)
p.changeDynamics(wall_1, -1, restitution=0.6)
# p.changeDynamics(wall, -1)

# p.setCollisionFilterGroupMask(pyramid, -1, 0,0 )

enableCol = 1
p.setCollisionFilterPair(obj, wall, -1, -1, enableCol)
p.setCollisionFilterPair(obj, wall_1, -1, -1, enableCol)
# p.setRealTimeSimulation(1)

f = 1
for i in range(3000):
    pos, orn = p.getBasePositionAndOrientation(obj)
    print('degree: ',[180*element/math.pi for element in p.getEulerFromQuaternion(orn)])
    # pos1, orn1 = p.getBasePositionAndOrientation(wall)
    
    if f:
        time.sleep(3)
        for j in range(5):
            p.applyExternalForce(obj, -1, forceObj=[-1000,0,0], posObj=pos, flags=p.WORLD_FRAME)
            f=0
    print("obj : ",pos, orn)
    # print("wall : ", pos1, orn1)
    time.sleep(0.01)
    p.stepSimulation()