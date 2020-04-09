import numpy as np
from fury import window, actor, ui
import itertools
import pybullet as p
import time 
import math


client = p.connect(p.DIRECT)
p.setGravity(0,0,-10, physicsClientId=client)


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
# wallvisual = p.createVisualShape(p.GEOM_BOX, halfExtents=[boxHalfLength, boxHalfWidth, boxHalfHeight])
wall = p.createMultiBody(baseCollisionShapeIndex=wallcollision, 
                  basePosition=[-4, 0, 4])

boxHalfLength_1 = 5
boxHalfWidth_1 = 5
boxHalfHeight_1 = 0.1
wallcollision_1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[boxHalfLength_1, boxHalfWidth_1, boxHalfHeight_1])
# wallvisual_1 = p.createVisualShape(p.GEOM_BOX, halfExtents=[boxHalfLength_1, boxHalfWidth_1, boxHalfHeight_1])
wall_1 = p.createMultiBody(baseCollisionShapeIndex=wallcollision_1,
                  basePosition=[0, 0, 0])

boxHalfLength_ = 0.1
boxHalfWidth_ = 0.1
boxHalfHeight_ = 0.5
objcollision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[boxHalfLength_, boxHalfWidth_, boxHalfHeight_])
# objvisual = p.createVisualShape(p.GEOM_BOX, 
#                                 halfExtents=[boxHalfLength_, boxHalfWidth_, boxHalfHeight_], 
#                                 rgbaColor=[0.1, 0.2, 0.6, 1])
obj = p.createMultiBody(baseMass = 0.5,
                  baseCollisionShapeIndex=objcollision,
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



xyz = np.array([[0,0,0]])
colors = np.array([[0.7, 0.5, 0.5, 1]])
radii = 0.5

scene = window.Scene()

sphere_actor = actor.sphere(centers=xyz,
                            colors=colors,
                            radii=radii)

cuboid_actor = actor.box(centers=xyz,
                         directions = np.array(p.getEulerFromQuaternion([ -0.4044981, -0.8089962, -0.4044981, 0.1352322 ])),
                         size = (0.2, 0.2, 1),
                         colors=(0,1,1))

wall_actor_1 = actor.box(centers=np.array([[-4, 0, 4]]),
                         directions=np.array([[1.57, 0, 0]]),
                         size=(0.2, 10, 10),
                         colors=(1,1,1))

wall_actor_2 = actor.box(centers=np.array([[0, 0, 0]]),
                         directions=np.array([[-1.57, 0 , 0]]),
                         size=(10, 10, 0.2),
                         colors=(1,1,1))

scene.add(wall_actor_1)
scene.add(wall_actor_2)
scene.add(cuboid_actor)

showm = window.ShowManager(scene,
                           size=(900, 768), reset_camera=False,
                           order_transparent=True)

showm.initialize()
counter = itertools.count()

class storage:
    f = 1
    orn_prev = ( -0.4044981, -0.8089962, -0.4044981, 0.1352322 )


def timer_callback(_obj, _event):
    cnt = next(counter)
    # tb.message = "Let's count up to 100 and exit :" + str(cnt)
    # showm.scene.azimuth(0.05 * cnt)
    # sphere_actor.GetProperty().SetOpacity(cnt/100.)
    showm.render()
    # print(sphere_actor.GetPosition())    
    pos, orn = p.getBasePositionAndOrientation(obj)
    
    if storage.f:
        # time.sleep(3)
        print("entered")
        for j in range(5):
            p.applyExternalForce(obj, -1, forceObj=[-1000,0,0], posObj=pos, flags=p.WORLD_FRAME)
            _, orn = p.getBasePositionAndOrientation(obj)
            storage.f=0
    cuboid_actor.SetPosition(pos[0], pos[1], pos[2])
    orn_deg = np.degrees(p.getEulerFromQuaternion(orn))
    cuboid_actor.SetOrientation(orn_deg[0], orn_deg[1], orn_deg[2])
    # cuboid_actor.RotateWXYZ(orn[1], orn[2], orn[3], orn[0])
    p.stepSimulation()
    print(orn)
    storage.orn_prev = orn
    
    if cnt == 2000:
        
        showm.exit()
# 

showm.add_timer_callback(True, 10, timer_callback)

showm.start()


window.record(showm.scene, size=(900, 768), out_path="viz_timer.png")



