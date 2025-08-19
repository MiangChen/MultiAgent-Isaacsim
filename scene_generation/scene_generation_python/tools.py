import omni.usd
import omni.kit.commands
from pxr import Gf, UsdGeom
from typing import List
from pydantic import BaseModel, Field
from langchain_core.tools import StructuredTool
import asyncio


# ---------- 输入模型定义 ----------

class DeletePrimInput(BaseModel):
    prim_path: str = Field(..., description="The full path of the prim to delete.")

class AddSphereInput(BaseModel):
    prim_path: str = Field(..., description="Path where the sphere will be created.")
    radius: float = Field(default=50.0, description="Radius of the sphere.")
    position: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0],
                                  description="Position of the sphere [x, y, z].")

class AddCubeInput(BaseModel):
    prim_path: str = Field(..., description="Path where the cube will be created.")
    size: float = Field(default=100.0, description="Size of the cube.")
    position: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0],
                                  description="Position of the cube [x, y, z].")


# ---------- 同步占位符函数 ----------

# 这些同步函数只是为了占位和确保工具的正确注册，它们本身不执行任何操作
def Add_sphere(inputs: AddSphereInput) -> str:
    return "Sync: Placeholder for add_sphere"

def Add_cube(inputs: AddCubeInput) -> str:
    return "Sync: Placeholder for add_cube"

def Delete_prim(inputs: DeletePrimInput) -> str:
    return "Sync: Placeholder for delete_prim"


# ---------- 工具函数封装类 ----------

class ToolFunctions:
    def __init__(self):
        self.tools = [
            StructuredTool.from_function(
                name="add_cube",
                description="Add a cube to the stage at a specific path, size, and position.",
                func=Add_cube,  # 同步占位符
                coroutine=self.add_cube
            ),
            StructuredTool.from_function(
                name="add_sphere",
                description="Add a sphere to the stage at a specific path, radius, and position.",
                func=Add_sphere,  # 同步占位符
                coroutine=self.add_sphere
            ),
            StructuredTool.from_function(
                name="delete_prim",
                description="Delete a prim from the stage by path.",
                func=Delete_prim,  # 同步占位符
                coroutine=self.delete_prim
            ),
        ]

    # ---------- 异步工具函数 ----------

    async def delete_prim(self, inputs: DeletePrimInput) -> str:
        await omni.kit.app.get_app().next_update_async()
        try:
            stage = omni.usd.get_context().get_stage()
            if not stage:
                return "[Delete Prim] No stage loaded."
            prim = stage.GetPrimAtPath(inputs.prim_path)
            if not prim or not prim.IsValid():
                return f"[Delete Prim] Invalid path: {inputs.prim_path}"
            stage.RemovePrim(inputs.prim_path)
            return f"[Delete Prim] Deleted {inputs.prim_path}"
        except Exception as e:
            return f"[Delete Prim] Error: {str(e)}"

    async def add_sphere(self, inputs: AddSphereInput) -> str:
        await omni.kit.app.get_app().next_update_async()
        try:
            result, prim_path = omni.kit.commands.execute(
                "CreateMeshPrim",
                prim_type="Sphere",
                path=inputs.prim_path,
                object_origin=Gf.Vec3f(*inputs.position),
                half_scale=inputs.radius
            )
            if result:
                return f"[Add Sphere] Created at {prim_path}, radius {inputs.radius}, position {inputs.position}"
            else:
                return "[Add Sphere] Command failed to create sphere."
        except Exception as e:
            return f"[Add Sphere] Error: {str(e)}"

    async def add_cube(self, inputs: AddCubeInput) -> str:
        # await omni.kit.app.get_app().next_update_async()  # 在主循环开始前，await next_update_async() 会导致死锁。
        try:
            result, prim_path = omni.kit.commands.execute(
                "CreateMeshPrim",
                prim_type="Cube",
                path=inputs.prim_path,
                object_origin=Gf.Vec3f(*inputs.position),
                half_scale=inputs.size / 2.0  # Convert full size → half_scale
            )
            if result:
                return f"[Add Cube] Created at {prim_path}, size {inputs.size}, position {inputs.position}"
            else:
                return "[Add Cube] Command failed to create cube."
        except Exception as e:
            return f"[Add Cube] Error: {str(e)}"

    # ---------- 可选：直接设置位置信息 ----------
    def set_prim_position(self, prim_path: str, position: List[float]):
        try:
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            xform = UsdGeom.Xformable(prim)
            xform.AddTranslateOp().Set(Gf.Vec3f(*position))
        except Exception as e:
            print(f"[Set Position] Error setting position for {prim_path}: {e}")
