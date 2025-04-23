# 首先确保导入所需的库和函数
from pydantic import BaseModel, Field
from typing import List
from omni.kit.commands import execute
from pxr import Gf

# 定义 AddSphereInput 模型
class AddSphereInput(BaseModel):
    prim_path: str = Field(..., description="Path where the sphere will be created.")
    radius: float = Field(default=50.0, description="Radius of the sphere.")
    position: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0],
                                  description="Position of the sphere [x, y, z].")

# 定义 add_sphere 函数
def add_sphere(inputs: AddSphereInput) -> str:
    from omni.kit.commands import execute
    try:
        result, prim_path = execute(
            "CreateMeshPrim",
            prim_type="Sphere",
            object_origin=Gf.Vec3f(*inputs.position),
            half_scale=inputs.radius
        )
        if result:
            return f"[Add Sphere] Created at {prim_path}, radius {inputs.radius}, position {inputs.position}"
        else:
            return "[Add Sphere] Command failed to create sphere."
    except Exception as e:
        return f"[Add Sphere] Error: {str(e)}"

# 现在调用 add_sphere 函数
input_data = AddSphereInput(prim_path="/World/Sphere", radius=10.0, position=[1.0, 2.0, 3.0])

# 调用 add_sphere 并输出结果
result = add_sphere(input_data)
print(result)
