import numpy as n
from fury import window,  actor, ui


cord = n.array([0,0,0]).reshape(1,3)
colors = n.random.rand(1, 4)
radii = 2

scene = window.Scene()
sphere_actor = actor.sphere(centers=cord, 
                            colors=colors, 
                            radii=radii)
scene.add(sphere_actor)

showm = window.ShowManager(scene=scene, size=(900, 768), reset_camera=False, order_transparent=True)
showm.initialize()

global count 
count = 0

def timer_callback(_obj, _event):
    global count
    count += 1
    print(count)
    print()
    if count == 100:
        showm.exit()

showm.add_timer_callback(True, 1000, timer_callback)
showm.start()

window.record(showm.scene, size=(900, 768), out_path="viz_timer.png")