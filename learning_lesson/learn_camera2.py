## put them in isaacsim scripts
from isaacsim.sensors.camera import Camera

c = Camera(prim_path = "/World/GR1_T2/camera2", name="camera2", )
# c.set_local_pose(translation=[0,0,0])
print("camera2 pose world", c.get_local_pose('world'))
print("camera2 pose ros", c.get_local_pose('ros'))
print("camera2 pose usd", c.get_local_pose('usd'))

c = Camera(prim_path = "/World/GR1_T2/camera3", name="camera3", )
c.set_world_pose(position=[0,0,0])

