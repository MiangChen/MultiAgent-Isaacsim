from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.sensors.camera import Camera
from isaacsim.core.api import World
import isaacsim.core.utils.numpy.rotations as rot_utils
import numpy as np
import matplotlib.pyplot as plt

import isaacsim.core.utils.prims as prims_utils
my_world = World(stage_units_in_meters=1.0)
from isaacsim.core.utils.semantics import  add_update_semantics
from pxr import Usd, UsdGeom, Gf

from isaacsim.core.utils.numpy import rotations
from isaacsim.core.utils.prims import define_prim, get_prim_at_path


prim_path = '/new_cube_2'
cube_2 = my_world.scene.add(
    DynamicCuboid(
        prim_path=prim_path,
        name="cube_1",
        position=np.array([5.0, 3, 1.0]),
        scale=np.array([0.6, 0.5, 0.2]),
        size=1.0,
        color=np.array([255, 0, 0]),
    )
)
prim = prims_utils.get_prim_at_path(prim_path)

semantic_label = 'house'
type_label = 'class'
add_update_semantics(prim, semantic_label, type_label, suffix=type_label)


cube_3 = my_world.scene.add(
    DynamicCuboid(
        prim_path="/new_cube_3",
        name="cube_2",
        position=np.array([-5, 1, 3.0]),
        scale=np.array([0.1, 0.1, 0.1]),
        size=1.0,
        color=np.array([0, 0, 255]),
        linear_velocity=np.array([0, 0, 0.4]),
    )
)

camera_axes = 'usd'
camera = Camera(
    prim_path="/World/camera",
    position=np.array([0.0, 0.0, 25.0]),
    frequency=20,
    resolution=(256, 256),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True), ## 角度到4元数
    # camera_axes = camera_axes,
    # orientation=[0.5, 0.5, -0.5, -0.5]# rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True), ## 角度到4元数
)

camera.set_local_pose(translation=np.array([0.0, 0.0, 0.0]), orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True), camera_axes= camera_axes )

my_world.scene.add_default_ground_plane()
my_world.reset()
camera.initialize()

i = 0
# camera.add_motion_vectors_to_frame()

# camera.add_distance_to_camera_to_frame()
# camera.add_distance_to_image_plane_to_frame()
camera.add_bounding_box_2d_loose_to_frame()
print(camera.get_local_pose())

while simulation_app.is_running():

    # 获取值（默认时间或指定时间）
    timecode = Usd.TimeCode.Default()
    prim = get_prim_at_path("/World/camera")
    translate_attr = prim.GetAttribute('xformOp:translate')
    if not translate_attr:
        print("Prim 未定义 xformOp:translate 属性")
    else:  # 静态用默认时间，动态用 Usd.TimeCode(frame)
        translate_value: Gf.Vec3d = translate_attr.Get(timecode)
        # print(f"平移值: {tra/slate_value}")  # 例如 (1.0, 2.0, 3.0)
        position = list(translate_value)
    print("translate", translate_value)
    quat_attr = prim.GetAttribute('xformOp:orient')
    if not quat_attr:
        print("Prim 未定义 xformOp:orient 属性")
    else:
        quat_value = quat_attr.Get(timecode)
        quat = [quat_value.real] + list(quat_value.imaginary)
        euler_degree = None
        print("quat", quat)

    # camera.set_world_pose(orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True)), ## 角度到4元数
    print(rot_utils.euler_angles_to_quats(np.array([0, 90, 0])))
    print(camera.get_local_pose(camera_axes='world'))
    print(camera.get_local_pose(camera_axes='ros'))
    print(camera.get_local_pose(camera_axes='usd'))
    # print(camera.get_world_pose(camera_axes=camera_axes))
    print('*' * 100)

    my_world.step(render=True)
    # print(camera.get_current_frame())
    if i == 100:
        import os
        # 在保存前确保目录存在
        os.makedirs("output", exist_ok=True)
        points_2d = camera.get_image_coords_from_world_points(
            np.array([cube_3.get_world_pose()[0], cube_2.get_world_pose()[0]])
        )
        points_3d = camera.get_world_points_from_image_coords(points_2d, np.array([24.94, 24.9]))  # 反推的时候, 关于深度的信息不会准确
        print("points_2d", points_2d)
        print("points_3d", points_3d)
        # imgplot = plt.imshow(camera.get_rgba()[:, :, :3])
        # 在update()函数中添加以下代码
        rgba_img = camera.get_rgba()[:, :, :3]  # 提取RGB通道
        imgplot = plt.imshow(rgba_img)
        plt.savefig(f"output/frame_{i}.png")  # 保存为PNG
        plt.close()  # 关闭当前图像避免内存泄漏

        plt.show()
        print(camera.get_current_frame().keys())  # dict_keys(['rendering_time', 'rendering_frame', 'rgba', 'motion_vectors', 'distance_to_image_plane'])
        print(camera.get_current_frame()['bounding_box_2d_loose'])
        # print(np.array_equal(camera.get_rgba(), camera.get_current_frame()['rgba']))  # True

        # print(camera.get_current_frame()["motion_vectors"])
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
    i += 1


simulation_app.close()
