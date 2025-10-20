# =============================================================================
# MCP Extension - Model Context Protocol Extension for Isaac Sim
# =============================================================================
#
# This module implements an Omniverse Kit extension that provides MCP (Model
# Context Protocol) functionality for Isaac Sim, enabling external tools to
# interact with the simulation environment through a standardized protocol.
#
# =============================================================================

# Standard library imports
import gc
import json
import math
import os
import socket
import sys
import threading
import time
import traceback
from pathlib import Path
from typing import Dict, Any, List, Optional, Union

# Third-party library imports
import numpy as np
import carb
import omni

current_dir = os.path.dirname(os.path.abspath(__file__))

files_parent_dir = os.path.abspath(os.path.join(current_dir, "../../../"))
sys.path.append(files_parent_dir)  # 将项目根目录添加到 sys.path 中

# Local project imports
from config.variables import PATH_PROJECT, PATH_ISAACSIM_ASSETS, ASSET_PATH
from physics_engine.isaacsim_utils import (
    add_reference_to_stage, get_stage_units, Articulation, World, 
    DynamicCuboid, DynamicSphere, create_prim
)
from physics_engine.pxr_utils import UsdGeom, Gf, Usd, UsdPhysics, PhysxSchema, UsdShade



# Extension Methods required by Omniverse Kit
# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class MCPExtension(omni.ext.IExt):
    _instance = None

    @classmethod
    def get_instance(cls):
        """
        A class method to get the singleton instance of the running extension.
        This is the main way other internal scripts will access the service.
        """
        return cls._instance

    def __init__(self) -> None:
        """Initialize the extension."""
        super().__init__()
        MCPExtension._instance = self  # Set the instance here as a fallback
        self.ext_id = None
        self.running = False
        self.host = None
        self.port = None
        self.socket = None
        self.assert_repository_path = None
        self.server_thread = None
        self._usd_context = None
        self._physx_interface = None  # not used
        self._timeline = None  # not used
        self._window = None  # not used
        self._status_label = None  # not used
        self._server_thread = None  # not used
        self._models = None
        self._settings = carb.settings.get_settings()
        self._image_url_cache = {}  # cache for image url
        self._text_prompt_cache = {}  # cache for text prompt
        self.prim_info = []

    def on_startup(self, ext_id: str):
        """Initialize extension and UI elements"""
        MCPExtension._instance = self
        print("trigger  on_startup for: ", ext_id)
        print("settings: ", self._settings.get("/exts/omni.kit.pipapi"))
        self._settings.set(
            "/persistent/isaac/asset_root/default",
            f"{PATH_ISAACSIM_ASSETS}/Assets/Isaac/4.5",
        )
        self.assert_repository_path = (
                self._settings.get("/exts/isaac.sim.mcp/repository")
                or f"{PATH_PROJECT}/scene"
        )
        self.port = self._settings.get("/exts/isaac.sim.mcp/server, port") or 8766
        self.host = self._settings.get("/exts/isaac.sim.mcp/server.host") or "localhost"
        if not hasattr(self, "running"):
            self.running = False

        self.ext_id = ext_id
        self._usd_context = omni.usd.get_context()
        self._start()

    def on_shutdown(self):
        MCPExtension._instance = None
        print("trigger  on_shutdown for: ", self.ext_id)
        self._models = {}
        gc.collect()
        self._stop()

    def _start(self):
        if self.running:
            print("Server is already running")
            return

        self.running = True

        try:
            # Create socket
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((self.host, self.port))
            self.socket.listen(1)

            # Start server thread
            self.server_thread = threading.Thread(target=self._server_loop)
            self.server_thread.daemon = True
            self.server_thread.start()

            print(f"Isaac Sim MCP server started on {self.host}:{self.port}")
        except Exception as e:
            print(f"Failed to start server: {str(e)}")
            self.stop()

    def _stop(self):
        self.running = False

        # Close socket
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None

        # Wait for thread to finish
        if self.server_thread:
            try:
                if self.server_thread.is_alive():
                    self.server_thread.join(timeout=1.0)
            except:
                pass
            self.server_thread = None

        print("Isaac Sim MCP server stopped")

    def _server_loop(self):
        """Main server loop in a separate thread"""
        print("Server thread started")
        self.socket.settimeout(1.0)  # Timeout to allow for stopping
        if not hasattr(self, "running"):
            self.running = False

        while self.running:
            try:
                # Accept new connection
                try:
                    client, address = self.socket.accept()
                    print(f"Connected to client: {address}")

                    # Handle client in a separate thread
                    client_thread = threading.Thread(
                        target=self._handle_client, args=(client,)
                    )
                    client_thread.daemon = True
                    client_thread.start()
                except socket.timeout:
                    # Just check running condition
                    continue
                except Exception as e:
                    print(f"Error accepting connection: {str(e)}")
                    time.sleep(0.5)
            except Exception as e:
                print(f"Error in server loop: {str(e)}")
                if not self.running:
                    break
                time.sleep(0.5)

        print("Server thread stopped")

    def _handle_client(self, client):
        """Handle connected client"""
        print("Client handler started")
        client.settimeout(None)  # No timeout
        buffer = b""

        try:
            while self.running:
                # Receive data
                try:
                    data = client.recv(16384)
                    if not data:
                        print("Client disconnected")
                        break

                    buffer += data
                    try:
                        # Try to parse command
                        command = json.loads(buffer.decode("utf-8"))
                        buffer = b""

                        # Execute command in Isaac Sim's main thread
                        async def execute_wrapper():
                            try:
                                response = self.execute_command(command)
                                response_json = json.dumps(response)
                                print("response_json: ", response_json)
                                try:
                                    client.sendall(response_json.encode("utf-8"))
                                except:
                                    print(
                                        "Failed to send response - client disconnected"
                                    )
                            except Exception as e:
                                print(f"Error executing command: {str(e)}")
                                traceback.print_exc()
                                try:
                                    error_response = {
                                        "status": "error",
                                        "message": str(e),
                                    }
                                    client.sendall(
                                        json.dumps(error_response).encode("utf-8")
                                    )
                                except:
                                    pass
                            return None

                        # import omni.kit.commands
                        # import omni.kit.async
                        from omni.kit.async_engine import run_coroutine

                        task = run_coroutine(execute_wrapper())
                        # import asyncio
                        # asyncio.ensure_future(execute_wrapper())
                        # time.sleep(30)

                        #
                        # omni.kit.async.get_event_loop().create_task(create_sphere_async())
                        # TODO:Schedule execution in main thread
                        # bpy.app.timers.register(execute_wrapper, first_interval=0.0)
                        # omni.kit.app.get_app().post_to_main_thread(execute_wrapper())
                        # carb.apputils.get_app().get_update_event_loop().post(execute_wrapper)

                        # from omni.kit.async_engine import run_coroutine
                        # run_coroutine(execute_wrapper())
                        # omni.kit.app.get_app().get_update_event_stream().push(0, 0, {"fn": execute_wrapper})
                    except json.JSONDecodeError:
                        # Incomplete data, wait for more
                        pass
                except Exception as e:
                    print(f"Error receiving data: {str(e)}")
                    break
        except Exception as e:
            print(f"Error in client handler: {str(e)}")
        finally:
            try:
                client.close()
            except:
                pass
            print("Client handler stopped")

    # TODO: This is a temporary function to execute commands in the main thread
    def execute_command(self, command):
        """Execute a command in the main thread"""
        try:
            cmd_type = command.get("type")
            params = command.get("params", {})

            # TODO: Ensure we're in the right context
            return self._execute_command_internal(command)

        except Exception as e:
            print(f"Error executing command: {str(e)}")
            traceback.print_exc()
            return {"status": "error", "message": str(e)}

    def _execute_command_internal(self, command):
        """Internal command execution with proper context"""
        cmd_type = command.get("type")
        params = command.get("params", {})

        # todo: add a handler for extend simulation method if necessary
        handlers = {
            # "create_object": self.create_object,
            # "modify_object": self.modify_object,
            # "delete_object": self.delete_object,
            # "get_object_info": self.get_object_info,
            # "execute_script": self.execute_script,
            "get_scene_info": self.get_scene_info,
            "create_physics_scene": self.create_physics_scene,
            "create_robot": self.create_robot,
            "browse_scene_repository": self.browse_scene_repository,
            "load_scene": self.load_scene,
            "save_scene": self.save_scene,
            "delete_prim": self.delete_prim,
            "get_scene_info": self.get_scene_info,
            "set_prim_scale": self.set_prim_scale,
            "adjust_pose": self.adjust_pose,
            "set_prim_activate_state": self.set_prim_activate_state,
            "set_collision_enabled": self.set_collision_enabled,
            "set_collision_offsets": self.set_collision_offsets,
            "set_collision_approximation": self.set_collision_approximation,
            "set_material_properties": self.set_material_properties,
            "set_physics_properties": self.set_physics_properties,
            "set_physics_scene_config": self.set_physics_scene_config,
            "create_joint": self.create_joint,
            "set_drive_parameters": self.set_drive_parameters,
            "get_selected_prim": self.get_selected_prim,
            "focus_on_prim": self.focus_on_prim,
            "create_camera": self.create_camera,
            "check_prim_overlap": self.check_prim_overlap,
            "create_object": self.create_object,
        }

        handler = handlers.get(cmd_type)
        if handler:
            try:
                print(f"Executing handler for {cmd_type}")
                result = handler(**params)
                print(f"Handler execution complete: /n", result)
                # return result
                if result and result.get("status") == "success":
                    return {"status": "success", "result": result}
                else:
                    return {
                        "status": "error",
                        "message": result.get("message", "Unknown error"),
                    }
            except Exception as e:
                print(f"Error in handler: {str(e)}")
                traceback.print_exc()
                return {"status": "error", "message": str(e)}
        else:
            return {"status": "error", "message": f"Unknown command type: {cmd_type}"}

    ###############################################################################################################
    ###############################################################################################################

    def delete_prim(self, prim_path):
        """
        删除指定路径的prim
        """
        try:
            # 获取当前USD stage
            self._stage = omni.usd.get_context().get_stage()
            if not self._stage:
                return {"status": "error", "message": "No USD stage is open."}

            # 验证路径格式
            if not prim_path or not prim_path.startswith('/'):
                return {"status": "error", "message": f"Invalid prim path format: {prim_path}"}

            # 获取prim对象
            prim = self._stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                return {"status": "error", "message": f"Prim not found or invalid: {prim_path}"}

            # 检查prim是否处于激活状态
            if not prim.IsActive():
                return {"status": "warning", "message": f"Prim is already inactive: {prim_path}"}

            # 尝试多种删除方法
            success = False

            # 方法1: 使用RemovePrim (您原来的方法)
            try:
                self._stage.RemovePrim(prim_path)
                success = True
            except Exception as e1:
                print(f"RemovePrim failed: {e1}")

                # 方法2: 先设置为非激活状态，然后删除
                try:
                    prim.SetActive(False)
                    self._stage.RemovePrim(prim_path)
                    success = True
                except Exception as e2:
                    print(f"SetActive + RemovePrim failed: {e2}")

                    # 方法3: 使用编辑上下文
                    try:
                        with Sdf.ChangeBlock():
                            self._stage.RemovePrim(prim_path)
                        success = True
                    except Exception as e3:
                        print(f"ChangeBlock + RemovePrim failed: {e3}")

                        # 方法4: 递归删除所有子prim然后删除父prim
                        try:
                            # 获取所有子prim
                            children = prim.GetChildren()
                            for child in children:
                                self._stage.RemovePrim(child.GetPath())

                            # 删除父prim
                            self._stage.RemovePrim(prim_path)
                            success = True
                        except Exception as e4:
                            print(f"Recursive deletion failed: {e4}")

            # 验证删除是否成功
            if success:
                # 检查prim是否真的被删除了
                check_prim = self._stage.GetPrimAtPath(prim_path)
                if check_prim and check_prim.IsValid() and check_prim.IsActive():
                    return {"status": "warning", "message": f"Prim still exists after deletion attempt: {prim_path}"}
                else:
                    return {"status": "success", "message": f"Successfully deleted {prim_path}"}
            else:
                return {"status": "error", "message": f"All deletion methods failed for: {prim_path}"}

        except Exception as e:
            return {"status": "error", "message": f"Unexpected error while deleting {prim_path}: {str(e)}"}

    def get_scene_info(self, max_depth: int = 2) -> Dict[str, Any]:
        try:
            self._stage = omni.usd.get_context().get_stage()
            if not self._stage:
                print("[Scene Info] No USD stage is open.")
                return {"status": "error", "message": "No USD stage is open."}

            # 清空之前的信息
            self.prim_info = []

            for prim in self._stage.Traverse():
                try:
                    # 计算prim的层级深度
                    prim_path = str(prim.GetPath().pathString)
                    # 计算路径深度：去掉开头的'/'，然后按'/'分割计算层数
                    if prim_path == '/':
                        depth = 0
                    else:
                        depth = prim_path.strip('/').count('/') + 1

                    # 如果超过最大深度限制，跳过此prim
                    if depth > max_depth:
                        continue
                    # 获取世界变换矩阵并转换为可序列化的格式
                    world_transform = omni.usd.get_world_transform_matrix(prim)
                    transform_list = None
                    if world_transform is not None:
                        try:
                            # 将Matrix4d对象转换为嵌套列表
                            if hasattr(world_transform, '__iter__') and hasattr(world_transform, '__getitem__'):
                                transform_list = []
                                for i in range(4):
                                    row = []
                                    for j in range(4):
                                        try:
                                            row.append(float(world_transform[i][j]))
                                        except:
                                            row.append(0.0)
                                    transform_list.append(row)
                            else:
                                # 如果无法访问矩阵元素，设为None
                                transform_list = None
                        except Exception as e:
                            print(f"[Scene Info] Failed to convert transform matrix for {prim.GetPath()}: {e}")
                            transform_list = None

                    # 获取文档信息，确保是字符串格式
                    doc = prim.GetMetadata("doc")
                    doc_str = str(doc) if doc is not None else None

                    # 暂时跳过world_transform以避免序列化问题
                    # 可以后续单独提供获取变换矩阵的方法

                    info = {
                        "path": str(prim.GetPath().pathString),
                        "name": str(prim.GetName()),
                        "type": str(prim.GetTypeName()),
                        "is_active": bool(prim.IsActive()),
                        "is_defined": bool(prim.IsDefined()),
                        "depth": depth,  # 添加深度信息
                        # "world_transform": transform_list,  # 暂时注释掉
                        "doc": doc_str,
                    }
                    self.prim_info.append(info)
                except Exception as e:
                    print(f"[Scene Info] Failed to extract info for prim: {prim.GetPath()} — {e}")
                    continue

            return {"status": "success", "result": self.prim_info}
        except Exception as e:
            print(f"[Scene Info] Unexpected error while getting prims: {e}")
            return {"status": "error", "message": str(e)}

    def get_selected_prim(self) -> Dict[str, str | Any]:
        ctx = omni.usd.get_context()
        selected_paths = ctx.get_selection().get_selected_prim_paths()
        return {"status": "success", "result": selected_paths}

    def check_prim_overlap(self, position: list, threshold: float = 0.1) -> Dict[str, Any]:
        """
        检查给定位置附近是否已有 prim，避免重复创建

        :param position: [x, y, z] 世界坐标列表
        :param threshold: 半径阈值（与舞台单位一致）
        :return: 如果检测到重叠则返回 True，否则 False
        """
        # 将列表转为 Vec3d
        pos_vec = Gf.Vec3d(*position)
        self._stage = omni.usd.get_context().get_stage()

        for prim in self._stage.Traverse():
            if not prim.IsValid():
                continue

            xformable = UsdGeom.Xformable(prim)
            if not xformable:
                continue

            try:
                for op in xformable.GetOrderedXformOps():
                    # 只检查平移或整体变换
                    if op.GetOpType() in (UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.TypeTransform):
                        existing_pos = Gf.Vec3d(op.Get())
                        if (existing_pos - pos_vec).GetLength() < threshold:
                            return {"status": "error", "message": f"Prim already exists near position {position}"}
            except Exception:
                continue
        return {"status": "success", "message": "No overlapping prims found at the specified position."}

    def check_prim_overlapping(self, position: list, threshold: float = 0.1) -> bool:
        """
        检查给定位置附近是否已有 prim，避免重复创建

        :param position: [x, y, z] 世界坐标列表
        :param threshold: 半径阈值（与舞台单位一致）
        :return: 如果检测到重叠则返回 True，否则 False
        """
        # 将列表转为 Vec3d
        pos_vec = Gf.Vec3d(*position)
        self._stage = omni.usd.get_context().get_stage()

        for prim in self._stage.Traverse():
            if not prim.IsValid():
                continue

            xformable = UsdGeom.Xformable(prim)
            if not xformable:
                continue

            try:
                for op in xformable.GetOrderedXformOps():
                    # 只检查平移或整体变换
                    if op.GetOpType() in (UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.TypeTransform):
                        existing_pos = Gf.Vec3d(op.Get())
                        if (existing_pos - pos_vec).GetLength() < threshold:
                            return False
            except Exception:
                continue
        return True

    def focus_on_prim(self, prim_path: str) -> Dict[str, Any]:
        from omni.kit.viewport.utility import get_active_viewport, frame_viewport_selection
        try:
            ctx = omni.usd.get_context()
        except Exception as e:
            return {"status": "error", "message": f"Unable to retrieve USD context: {e}"}

        # 选中目标 prim
        try:
            ctx.get_selection().set_selected_prim_paths([prim_path], True)
        except Exception as e:
            return {"status": "error", "message": f"Failed to select prim '{prim_path}': {e}"}

        # 获取活跃视口
        try:
            viewport = get_active_viewport()
            if viewport is None:
                return {"status": "error", "message": f"No active viewport to focus on '{prim_path}'"}
        except Exception as e:
            return {"status": "error", "message": f"Error obtaining active viewport: {e}"}

        # 框选并聚焦
        try:
            frame_viewport_selection(viewport)
        except Exception as e:
            return {"status": "error", "message": f"Failed to frame prim '{prim_path}' in viewport: {e}"}

        return {"status": "success", "message": f"Prim '{prim_path}' focused in viewport"}

    def create_robot(self, robot_type: str = "g1", position: List[float] = [0, 0, 0]) -> Dict[str, Any]:

        from config.variables import ASSET_PATH

        ROBOT_CONFIGS = {
            "franka": {
                "usd_path": "/Isaac/Robots/Franka/franka.usd",
                "base_prim_path": "/World/Arm",
                "base_name": "my_arm",
            },
            "jetbot": {
                "usd_path": "/Isaac/Robots/Jetbot/jetbot.usd",
                "base_prim_path": "/World/Jetbot",
                "base_name": "my_jetbot",
            },
            "carter": {
                "usd_path": "/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd",
                "base_prim_path": "/World/Car",
                "base_name": "my_car",
            },
            "g1": {
                "usd_path": "/Isaac/Robots/Unitree/G1/g1.usd",
                "base_prim_path": "/World/G1",
                "base_name": "my_g1",
            },
            "go1": {
                "usd_path": "/Isaac/Robots/Unitree/Go1/go1.usd",
                "base_prim_path": "/World/Go1",
                "base_name": "my_go1",
            },
            "h1": {
                "usd_path": "/Isaac/Robots/Unitree/H1/h1.usd",
                "base_prim_path": "/World/H1",
                "base_name": "my_h1",
            },
        }

        robot_type = robot_type.lower()
        config = ROBOT_CONFIGS.get(robot_type, ROBOT_CONFIGS["franka"])
        asset_path = ASSET_PATH + config["usd_path"]

        # 获取当前 prim 信息
        scene_info = self.get_scene_info()
        existing_paths = []
        if scene_info["status"] == "success":
            for prim in scene_info["result"]:
                existing_paths.append(prim["path"])

        # 生成唯一 prim_path 和 name
        base_path = config["base_prim_path"]
        base_name = config["base_name"]
        suffix = 0
        new_prim_path = base_path
        new_name = base_name

        while new_prim_path in existing_paths:
            suffix += 1
            new_prim_path = f"{base_path}_{suffix}"
            new_name = f"{base_name}_{suffix}"

        # 添加到 Stage
        add_reference_to_stage(usd_path=asset_path, prim_path=new_prim_path)

        # 创建 Articulation 对象
        robot = Articulation(prim_paths_expr=new_prim_path, name=new_name)

        # 设置初始位姿
        robot.set_world_poses(positions=np.array([position]) / get_stage_units())
        return {"status": "success", "message": f"{robot_type} robot created at {new_prim_path}"}

    def create_object(self, usd_path: str, position: List[float], orientation: List[float]) -> Dict[str, Any]:


        # 检查重合
#        if self.check_prim_overlapping(position):
#            return {"status": "error", "message": f"检测到重合，位置 {position} 已有对象，创建取消。"}

        # 获取场景中已有 prim 路径
        scene_info = self.get_scene_info()
        existing_paths = []
        if scene_info.get("status") == "success":
            for prim in scene_info.get("result", []):
                existing_paths.append(prim["path"])

        # 生成唯一 prim 路径与名称
        base_name = Path(usd_path).stem
        base_prim_path = f"/World/{base_name}"
        suffix = 0
        new_prim_path = base_prim_path
        new_name = base_name
        while new_prim_path in existing_paths:
            suffix += 1
            new_prim_path = f"{base_prim_path}_{suffix}"
            new_name = f"{base_name}_{suffix}"

        asset_path = ASSET_PATH + "/Isaac/" + usd_path

        # 将 asset 引用添加到 Stage
        add_reference_to_stage(usd_path=asset_path, prim_path=new_prim_path)

        # 设置初始位姿（使用 adjust_pose 方法）
        result = self.adjust_pose(new_prim_path, position,
                                  self.eluer_to_quaternion(orientation[0], orientation[1], orientation[2]))
        if result.get("status") != "success":
            return result

        return {"status": "success", "result": new_prim_path}

    def eluer_to_quaternion(
            self,
            roll: float = 0.0,
            pitch: float = 0.0,
            yaw: float = 0.0,
            degrees: bool = True,
    ) -> List[float]:
        """Convert Euler angles to quaternion.

        Args:
            roll: Roll angle in degrees.
            pitch: Pitch angle in degrees.
            yaw: Yaw angle in degrees.
            degrees: Whether the input angles are in degrees (default is True). If False, angles are assumed to be in radians.
        Returns:
            Quaternion as a list [w, x, y, z].
        """
        if degrees:
            x, y, z = roll * (math.pi / 180), pitch * (math.pi / 180), yaw * (math.pi / 180)
        else:
            x, y, z = roll, pitch, yaw

        # 计算每个轴的半角
        cx = math.cos(x / 2)
        sx = math.sin(x / 2)
        cy = math.cos(y / 2)
        sy = math.sin(y / 2)
        cz = math.cos(z / 2)
        sz = math.sin(z / 2)

        # XYZ 顺序旋转生成的四元数
        w = cx * cy * cz + sx * sy * sz
        x = sx * cy * cz - cx * sy * sz
        y = cx * sy * cz + sx * cy * sz
        z = cx * cy * sz - sx * sy * cz

        return [w, x, y, z]

    def create_camera(self,
                      position: List[float],
                      orientation: List[float],
                      prim_path: str = "/World/MyCam") -> Dict[str, Any]:

        from omni.kit.viewport.utility import get_active_viewport
        try:
            # 获取当前 USD Stage
            ctx = omni.usd.get_context()
            stage = ctx.get_stage()
            if stage is None:
                return {"status": "error", "message": "No USD stage is currently open."}

            # 定义或获取 Camera prim
            cam = UsdGeom.Camera.Define(stage, prim_path)
            xformable = UsdGeom.Xformable(cam)

            # 查找已有的 translate/orient Op
            ops = xformable.GetOrderedXformOps()
            translate_op = next(
                (op for op in ops if op.GetOpType() == UsdGeom.XformOp.TypeTranslate), None
            )
            orient_op = next(
                (op for op in ops if op.GetOpType() == UsdGeom.XformOp.TypeOrient), None
            )

            # 如果不存在就创建
            if translate_op is None:
                translate_op = xformable.AddTranslateOp()
            if orient_op is None:
                orient_op = xformable.AddOrientOp()

            # 设置平移
            translate_op.Set(Gf.Vec3d(*position))

            # 构建欧拉旋转：先 X，再 Y，再 Z
            rot_x = Gf.Rotation(Gf.Vec3d(1, 0, 0), orientation[0])
            rot_y = Gf.Rotation(Gf.Vec3d(0, 1, 0), orientation[1])
            rot_z = Gf.Rotation(Gf.Vec3d(0, 0, 1), orientation[2])
            combined = rot_z * rot_y * rot_x
            quat = Gf.Quatf(combined.GetQuat())

            # 设置旋转
            orient_op.Set(quat)

            # 切换 GUI 视口到此相机
            viewport = get_active_viewport()
            if viewport is None:
                return {"status": "error", "message": "No active viewport to focus on camera."}
            viewport.camera_path = prim_path

            return {
                "status": "success",
                "message": f"Camera prim '{prim_path}' created (or reused) and focused."
            }

        except Exception as e:
            return {
                "status": "error",
                "message": f"Failed to create or focus camera at '{prim_path}': {e}"
            }

    def set_prim_scale(self, prim_path: str, scale: list) -> dict:
        try:
            # 获取当前 USD stage
            self._stage = omni.usd.get_context().get_stage()
            if not self._stage:
                return {"status": "error", "message": "No USD stage is currently open."}

            prim = self._stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                return {"status": "error", "message": f"Prim {prim_path} not found."}

            xform = UsdGeom.Xformable(prim)

            # 查找是否已有 ScaleOp
            scale_op = None
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                    scale_op = op
                    break

            # 如果没有，才添加
            if not scale_op:
                scale_op = xform.AddScaleOp()

            # 设置缩放值
            scale_op.Set(Gf.Vec3f(*scale))

            return {"status": "success", "message": f"Scale set to {scale} for {prim_path}"}

        except Exception as e:
            return {"status": "error", "message": f"Failed to set scale: {str(e)}"}

    def adjust_pose(self, prim_path: str, position: list, orientation: list) -> dict:


        try:
            self._stage = omni.usd.get_context().get_stage()
            if not self._stage:
                return {"status": "error", "message": "No USD stage is currently open."}

            prim = self._stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                return {"status": "error", "message": f"Prim {prim_path} not found."}

            is_articulation = False

            # 检查是否是 Articulation 类型
            try:
                robot = Articulation(prim_path)
                if robot.is_valid():
                    is_articulation = True
                    # 设置位置（Articulation 不支持旋转，需用 Xformable 设置）
                    import numpy as np
                    position_np = np.array([position]) / get_stage_units()
                    robot.set_world_poses(positions=position_np)
            except Exception as e:
                print(f"[DEBUG] Not an articulation or failed to init: {e}")

            xform = UsdGeom.Xformable(prim)

            # 设置旋转（四元数）
            quat = Gf.Quatd(orientation[0], orientation[1], orientation[2], orientation[3])
            orient_op = None
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                    orient_op = op
                    break
            if not orient_op:
                orient_op = xform.AddOrientOp()
            orient_op.Set(quat)

            # 非 articulation 类型，再设置位置
            if not is_articulation:
                translate_op = None
                for op in xform.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        translate_op = op
                        break
                if not translate_op:
                    translate_op = xform.AddTranslateOp()
                translate_op.Set(Gf.Vec3d(*position))

            return {
                "status": "success",
                "message": f"Set position and orientation for {prim_path}"
            }

        except Exception as e:
            return {"status": "error", "message": f"Failed to set transform: {str(e)}"}

    def set_prim_activate_state(self, prim_path: str, activate: bool = True) -> dict:
        """Set the active state of a prim."""
        try:
            # 获取当前 USD Stage
            self._stage = omni.usd.get_context().get_stage()
            if not self._stage:
                return {"status": "error", "message": "No USD stage is currently open."}

            # 获取指定 prim
            prim = self._stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                return {"status": "error", "message": f"Prim '{prim_path}' not found."}

            # 设置激活状态
            prim.SetActive(activate)

            state_str = "activated" if activate else "deactivated"
            return {"status": "success", "message": f"Prim '{prim_path}' {state_str}."}

        except Exception as e:
            return {"status": "error", "message": f"Failed to set prim active state: {str(e)}"}

    def set_collision_offsets(self, prim_path: str, contact_offset: float, rest_offset: float) -> dict:
        try:
            # 获取并缓存 Stage
            self._stage = omni.usd.get_context().get_stage()
            if not self._stage:
                return {"status": "error", "message": "No USD stage is currently open."}

            # 查找 prim
            prim = self._stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                return {"status": "error", "message": f"Prim '{prim_path}' not found."}

            # 导入并应用基础碰撞 API
            UsdPhysics.CollisionAPI.Apply(prim)

            # 应用 PhysX 扩展碰撞 API，设置偏移
            physx_api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            physx_api.CreateContactOffsetAttr().Set(contact_offset)
            physx_api.CreateRestOffsetAttr().Set(rest_offset)

            return {
                "status": "success",
                "message": f"Set contactOffset={contact_offset}, restOffset={rest_offset} for '{prim_path}'",
                "result": prim_path
            }

        except Exception as e:
            return {"status": "error", "message": f"Failed to set collision offsets: {e}"}

    def set_collision_enabled(self, prim_path: str, collision_enabled: bool) -> dict:
        try:
            # 获取 Stage
            self._stage = omni.usd.get_context().get_stage()
            if not self._stage:
                return {"status": "error", "message": "No USD stage is currently open."}

            prim = self._stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                return {"status": "error", "message": f"Prim {prim_path} not found."}

            UsdPhysics.CollisionAPI.Apply(prim)

            # 设置 collision enabled 属性
            physx_api = PhysxSchema.PhysxCollisionAPI.Apply(prim)

            return {
                "status": "success",
                "message": f"Collision enabled set for {prim_path}",
                "result": prim_path
            }

        except Exception as e:
            return {"status": "error", "message": f"Failed to set collision enabled: {str(e)}"}

    def set_physics_properties(self, prim_path: str, mass: float, velocity: list, gravity_enabled: bool) -> dict:
        try:
            # 获取并缓存 Stage
            self._stage = omni.usd.get_context().get_stage()
            if not self._stage:
                return {"status": "error", "message": "No USD stage is currently open."}

            # 查找 prim
            prim = self._stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                return {"status": "error", "message": f"Prim '{prim_path}' not found."}

            # 导入所需 API
            # 1. 应用基础刚体 API 并设置初速度
            rigid_api = UsdPhysics.RigidBodyAPI.Apply(prim)
            rigid_api.CreateVelocityAttr().Set(Gf.Vec3f(*velocity))

            # 2. 应用 PhysX 扩展刚体 API，设置 disableGravity（取反 gravity_enabled）
            physx_rigid_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            physx_rigid_api.CreateDisableGravityAttr().Set(not gravity_enabled)

            # 3. 应用 MassAPI，创建并设置质量属性
            mass_api = UsdPhysics.MassAPI.Apply(prim)
            mass_api.CreateMassAttr().Set(mass)

            return {
                "status": "success",
                "message": f"Physics properties set for '{prim_path}'",
                "result": prim_path
            }

        except Exception as e:
            return {"status": "error", "message": f"Failed to set physics properties: {e}"}

    def set_collision_approximation(self, prim_path: str, approximation: str) -> dict:
        """
        在当前 USD Stage 上，为指定 Mesh prim 设置碰撞近似（approximation）属性。
        参数:
        prim_path: prim 路径，如 "/World/Cube"
        approximation: one of {
            "none", "meshSimplification", "convexHull",
            "convexDecomposition", "boundingSphere",
            "boundingCube", "sdf"
        }
        """
        try:
            # 获取并缓存 Stage
            self._stage = omni.usd.get_context().get_stage()
            if not self._stage:
                return {"status": "error", "message": "No USD stage is currently open."}

            # 查找 prim
            prim = self._stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                return {"status": "error", "message": f"Prim '{prim_path}' not found."}

            # 1. 应用基础 CollisionAPI
            UsdPhysics.CollisionAPI.Apply(prim)

            # 2. 仅对 Mesh prim 应用 MeshCollisionAPI
            if prim.GetTypeName() != "Mesh":
                return {"status": "error", "message": f"Prim '{prim_path}' is not a Mesh."}
            mesh_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
            mesh_api.CreateApproximationAttr().Set(approximation)

            # 3. 若选择 SDF，需要额外设置 SDF 分辨率
            if approximation == "sdf":
                sdf_api = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim)
                sdf_api.CreateSdfResolutionAttr().Set(300)

            return {
                "status": "success",
                "message": f"Set collision approximation '{approximation}' for '{prim_path}'",
                "result": prim_path
            }

        except Exception as e:
            return {"status": "error", "message": f"Failed to set collision approximation: {e}"}

    def set_material_properties(
            self,
            material_path: str,
            static_friction: float,
            dynamic_friction: float,
            restitution: float,
            static_friction_mode: str,
            dynamic_friction_mode: str,
            restitution_mode: str
    ) -> dict:
        """
        在当前 USD Stage 上，为指定 Material prim 设置物理材质属性：
        - staticFriction: 静摩擦系数
        - dynamicFriction: 动摩擦系数
        - restitution: 恢复系数
        - staticFrictionCombineMode: 静摩擦合并方式
        - dynamicFrictionCombineMode: 动摩擦合并方式
        - restitutionCombineMode: 恢复系数合并方式
        """
        try:
            # 获取并缓存 Stage
            self._stage = omni.usd.get_context().get_stage()
            if not self._stage:
                return {"status": "error", "message": "No USD stage is currently open."}

            # 查找 material prim
            prim = self._stage.GetPrimAtPath(material_path)
            if not prim or not prim.IsValid():
                return {"status": "error", "message": f"Material '{material_path}' not found."}

            # 1. 应用 MaterialAPI 设置摩擦与恢复系数
            mat_api = UsdPhysics.MaterialAPI.Apply(prim)
            mat_api.CreateStaticFrictionAttr().Set(static_friction)
            mat_api.CreateDynamicFrictionAttr().Set(dynamic_friction)
            mat_api.CreateRestitutionAttr().Set(restitution)

            # 2. 应用 PhysX 材质扩展设置合并模式
            physx_api = PhysxSchema.PhysxMaterialAPI.Apply(prim)
            physx_api.CreateStaticFrictionCombineModeAttr().Set(static_friction_mode)
            physx_api.CreateDynamicFrictionCombineModeAttr().Set(dynamic_friction_mode)
            physx_api.CreateRestitutionCombineModeAttr().Set(restitution_mode)

            return {
                "status": "success",
                "message": f"Material properties updated for '{material_path}'",
                "result": material_path
            }

        except Exception as e:
            return {"status": "error", "message": f"Failed to set material properties: {e}"}

    def set_physics_scene_config(
            self,
            gravity: list,
            unit_system: str,
            scale: float,
            default_material: str = ""
    ) -> dict:
        """
        在当前 USD Stage 上配置全局物理场景：
        - gravity: 重力方向向量 [x, y, z]
        - unit_system: "metric" 或 "imperial"（仅供记录）
        - scale: 每个单位代表的米数 (metersPerUnit)
        - default_material: 默认物理材质 prim 路径（可选）
        """
        try:
            # 获取并缓存 Stage
            self._stage = omni.usd.get_context().get_stage()
            if not self._stage:
                return {"status": "error", "message": "No USD stage is currently open."}

            stage = self._stage
            scene_path = "/physicsScene"
            scene_prim = stage.GetPrimAtPath(scene_path)
            if not scene_prim or not scene_prim.IsValid():
                scene_prim = stage.DefinePrim(scene_path, "PhysicsScene")
            scene = UsdPhysics.Scene(scene_prim)

            # 设置重力
            vec = Gf.Vec3f(*gravity)
            scene.CreateGravityDirectionAttr().Set(vec)
            scene.CreateGravityMagnitudeAttr().Set(vec.GetLength())

            # 设置舞台线性单位
            UsdGeom.SetStageMetersPerUnit(stage, scale)

            # 绑定默认物理材质（可选）
            if default_material:
                PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
                mat_binding = UsdShade.MaterialBindingAPI.Apply(scene_prim)
                mat_prim = stage.GetPrimAtPath(default_material)
                if not mat_prim or not mat_prim.IsValid():
                    return {"status": "error", "message": f"Default material '{default_material}' not found."}
                material = UsdShade.Material(mat_prim)
                mat_binding.Bind(material, materialPurpose="physics")

            return {
                "status": "success",
                "message": "Physics scene configuration updated.",
                "result": scene_path
            }

        except Exception as e:
            return {"status": "error", "message": f"Failed to set physics scene config: {e}"}

    def create_joint(
            self,
            joint_path: str,
            joint_type: str,
            body0: str,
            body1: str,
            axis: list,
            local_pos0: list,
            local_pos1: list,
            lower_limit: float = None,
            upper_limit: float = None
    ) -> dict:
        try:
            self._stage = omni.usd.get_context().get_stage()
            if not self._stage:
                return {"status": "error", "message": "No USD stage is currently open."}

            # 校验两个刚体 prim 是否有效
            if not self._stage.GetPrimAtPath(body0).IsValid():
                return {"status": "error", "message": f"Body0 '{body0}' not found."}
            if not self._stage.GetPrimAtPath(body1).IsValid():
                return {"status": "error", "message": f"Body1 '{body1}' not found."}

            # 创建 joint prim 并选择类型
            joint_type = joint_type.lower()
            if joint_type == "revolute":
                joint = UsdPhysics.RevoluteJoint.Define(self._stage, joint_path)
            elif joint_type == "prismatic":
                joint = UsdPhysics.PrismaticJoint.Define(self._stage, joint_path)
            elif joint_type == "fixed":
                joint = UsdPhysics.FixedJoint.Define(self._stage, joint_path)
            else:
                return {"status": "error", "message": f"Invalid joint type: {joint_type}"}

            # 绑定连接体
            joint.CreateBody0Rel().SetTargets([body0])
            joint.CreateBody1Rel().SetTargets([body1])

            # 设置局部连接点
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*local_pos0))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*local_pos1))

            # 设置轴（仅 Revolute / Prismatic）
            if joint_type in ["revolute", "prismatic"]:
                joint.CreateAxisAttr().Set(Gf.Vec3f(*axis).GetNormalized())

                # 设置限制（可选）
                if lower_limit is not None:
                    joint.CreateLowerLimitAttr().Set(lower_limit)
                if upper_limit is not None:
                    joint.CreateUpperLimitAttr().Set(upper_limit)

            return {
                "status": "success",
                "message": f"{joint_type.capitalize()} joint created at {joint_path}",
                "result": joint_path
            }

        except Exception as e:
            return {"status": "error", "message": f"Failed to create joint: {str(e)}"}

    def set_drive_parameters(
            self,
            joint_path: str,
            drive_name: str,
            target_position: float,
            target_velocity: float,
            stiffness: float,
            damping: float,
            max_force: float
    ) -> dict:
        try:
            self._stage = omni.usd.get_context().get_stage()
            if not self._stage:
                return {"status": "error", "message": "No USD stage is currently open."}

            joint = self._stage.GetPrimAtPath(joint_path)
            if not joint.IsValid():
                return {"status": "error", "message": f"Joint prim '{joint_path}' not found."}

            # 应用 DriveAPI 到关节上（名称可以是 "angular" 或 "linear"）
            drive_api = UsdPhysics.DriveAPI.Apply(joint, drive_name)

            drive_api.CreateTargetPositionAttr().Set(target_position)
            drive_api.CreateTargetVelocityAttr().Set(target_velocity)
            drive_api.CreateStiffnessAttr().Set(stiffness)
            drive_api.CreateDampingAttr().Set(damping)
            drive_api.CreateMaxForceAttr().Set(max_force)

            return {
                "status": "success",
                "message": f"Drive parameters set on {joint_path} ({drive_name})",
                "result": joint_path
            }

        except Exception as e:
            return {"status": "error", "message": f"Failed to set drive parameters: {str(e)}"}

    ###############################################################################################################
    ###############################################################################################################

    def create_physics_scene(
            self,
            objects: List[Dict[str, Any]] = [],
            floor: bool = True,
            gravity: List[float] = (0.0, -9.81, 0.0),
            scene_name: str = "physics_scene",
    ) -> Dict[str, Any]:
        """Create a physics scene with multiple objects."""
        try:

                DynamicCone,
                DynamicCylinder,
                DynamicCapsule,
            )

            # 类型映射表
            type_class_map = {
                "Cube": DynamicCuboid,
                "Box": DynamicCuboid,
                "Sphere": DynamicSphere,
                "Cone": DynamicCone,
                "Cylinder": DynamicCylinder,
                "Capsule": DynamicCapsule,
            }

            world = World().instance()

            if floor:
                world.scene.add_default_ground_plane()

            print("Start creating objects:", objects)
            objects_created = 0
            created_paths = []

            for i, obj in enumerate(objects):
                obj_name = obj.get("name", f"object_{i}")
                obj_type = obj.get("type", "Cube")
                obj_path = obj.get("path", f"/World/{obj_name}")
                obj_class = type_class_map.get(obj_type)

                if not obj_class:
                    print(f"[Warning] Unsupported object type: {obj_type}, skipping.")
                    continue

                # 通用参数
                shape_args = {
                    "prim_path": obj_path,
                    "name": obj_name,
                    "position": np.array(obj.get("position", [0, 0, 0])),
                    "orientation": np.array(
                        obj.get("orientation", [1, 0, 0, 0])
                    ),  # scalar-first quaternion
                    "scale": np.array(obj.get("scale", [1, 1, 1])),
                    "color": np.array(obj.get("color", [0.5, 0.5, 1.0])),
                    "mass": obj.get("mass", 1.0),
                }

                # 类型特定参数
                if obj_type == "Sphere":
                    shape_args["radius"] = obj.get("radius", 1.0)

                elif obj_type in ["Cone", "Cylinder", "Capsule"]:
                    shape_args["radius"] = obj.get("radius", 1.0)
                    shape_args["height"] = obj.get("height", 2.0)

                elif obj_type in ["Cube", "Box"]:
                    shape_args["size"] = obj.get("size", None)  # 可选，Cube 支持 size

                # 创建对象
                world.scene.add(obj_class(**shape_args))
                objects_created += 1
                created_paths.append(obj_path)

            return {
                "status": "success",
                "message": f"Created physics scene with {objects_created} objects",
                "result": created_paths,
            }

        except Exception as e:
            return {
                "status": "error",
                "message": f"Failed to create physics scene: {e}",
                "result": None,
            }

    # TODO:扩展到isaacsim官方Assets,支持更多的场景，或者使用Asset browser mcp的resource 功能。
    def browse_scene_repository(
            self,
    ) -> Dict[str, Any]:
        """Browse the scene repository and return a list of files."""
        try:
            import os

            # 获取场景目录
            scene_dir = self.assert_repository_path
            print("scene_dir: ", scene_dir)

            # 获取目录下的所有文件和子目录
            items = os.listdir(scene_dir)
            # 过滤出所有 usd 文件以及其绝对路径
            usd_files = [
                os.path.join(scene_dir, item)
                for item in items
                if item.endswith(".usd") or item.endswith(".usda")
            ]
            print("usd_files: ", usd_files)

            return {
                "status": "success",
                "message": "Scene repository browsed successfully",
                "result": usd_files,
            }

        except Exception as e:
            return {
                "status": "error",
                "message": f"Failed to browse scene repository: {e}",
                "result": None,
            }

    def load_scene(self, usd_path: str, prim_path_root: str = "/World"):
        """

        Create a scene from config.(But just input usd file yet.)
        Args:
            usd_path (str): path to scene config file(use to be a .usd file)
            prim_path_root (str): path to root prim
        """
        if (
                usd_path.endswith("usd")
                or usd_path.endswith("usda")
                or usd_path.endswith("usdc")
        ):


            create_prim(
                prim_path_root,
                usd_path=usd_path,
                # scale=self.simulation.scene_scale,
                scale=[1, 1, 1],
                # translation=[0, 0, 0.81696],
                # orientation=[0.610, -0.789, -0.05184, 0.040] # wxyz, xyz还是zyx顺序不确定
            )
        else:
            raise RuntimeError("Env file path needs to end with .usd, .usda or .usdc .")
        return

    def save_scene(
            self,
            scene_name: str = None,
    ) -> Dict[str, Any]:
        """Save the current scene to the repository."""
        try:
            omni.usd.get_context().save_as_stage(
                f"{self.assert_repository_path}/{scene_name}.usd", None
            )
            return {
                "status": "success",
                "message": f"Scene saved successfully as {scene_name}",
                "result": None,
            }
        except Exception as e:
            return {
                "status": "error",
                "message": f"Failed to save scene: {e}",
                "result": None,
            }

