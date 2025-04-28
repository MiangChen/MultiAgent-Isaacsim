import carb
import threading
import time
import socket
import json
import traceback

import gc

import omni
from typing import Dict, Any, List, Optional, Union
import numpy as np

from isaac_sim_mcp_extension.gen3d import Beaver3d
from isaac_sim_mcp_extension.usd import USDLoader


# Extension Methods required by Omniverse Kit
# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class MCPExtension(omni.ext.IExt):
    def __init__(self) -> None:
        """Initialize the extension."""
        super().__init__()
        self.ext_id = None
        self.running = False
        self.host = None
        self.port = None
        self.socket = None
        self.server_thread = None
        self._usd_context = None
        self._physx_interface = None
        self._timeline = None
        self._window = None
        self._status_label = None
        self._server_thread = None
        self._models = None
        self._settings = carb.settings.get_settings()
        self._image_url_cache = {}  # cache for image url
        self._text_prompt_cache = {}  # cache for text prompt

    def on_startup(self, ext_id: str):
        """Initialize extension and UI elements"""
        print("trigger  on_startup for: ", ext_id)
        print("settings: ", self._settings.get("/exts/omni.kit.pipapi"))
        carb.settings.get_settings().set(
            "/persistent/isaac/asset_root/default",
            "/home/yons/isaacsim_assets/Assets/Isaac/4.5",
        )
        self.port = self._settings.get("/exts/isaac.sim.mcp/server, port") or 8766
        self.host = self._settings.get("/exts/isaac.sim.mcp/server.host") or "localhost"
        if not hasattr(self, "running"):
            self.running = False

        self.ext_id = ext_id
        self._usd_context = omni.usd.get_context()
        # omni.kit.commands.execute("CreatePrim", prim_type="Sphere")

        # print("sphere created")
        # result = self.execute_script('omni.kit.commands.execute("CreatePrim", prim_type="Cube")')
        # print("script executed", result)
        self._start()
        # result = self.execute_script('omni.kit.commands.execute("CreatePrim", prim_type="Cube")')
        # print("script executed", result)

    def on_shutdown(self):
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

    def get_scene_info(self):

        self._stage = omni.usd.get_context().get_stage()
        assert self._stage is not None

        stage_path = self._stage.GetRootLayer().realPath

        settings = carb.settings.get_settings()
        assets_root_path = settings.get("/persistent/isaac/asset_root/default")
        return {
            "status": "success",
            "message": "pong",
            "assets_root_path": assets_root_path,
        }

    def create_robot(self, robot_type: str = "g1", position: List[float] = [0, 0, 0]):
        from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
        from isaacsim.storage.native import get_assets_root_path
        from isaacsim.core.prims import Articulation

        ROBOT_CONFIGS = {
            "franka": {
                "usd_path": "/Isaac/Robots/Franka/franka.usd",
                "prim_path": "/World/Arm",
                "name": "my_arm",
            },
            "jetbot": {
                "usd_path": "/Isaac/Robots/Jetbot/jetbot.usd",
                "prim_path": "/World/Jetbot",
                "name": "my_jetbot",
            },
            "carter": {
                "usd_path": "/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd",
                "prim_path": "/World/Car",
                "name": "my_car",
            },
            "g1": {
                "usd_path": "/Isaac/Robots/Unitree/G1/g1.usd",
                "prim_path": "/World/G1",
                "name": "my_g1",
            },
            "go1": {
                "usd_path": "/Isaac/Robots/Unitree/Go1/go1.usd",
                "prim_path": "/World/Go1",
                "name": "my_go1",
            },
        }
        assets_root_path = get_assets_root_path()
        print("position: ", position)

        robot_type = robot_type.lower()
        config = ROBOT_CONFIGS.get(robot_type)

        if config is None:
            # 默认使用 franka
            config = ROBOT_CONFIGS["franka"]

        # 拼接完整路径
        asset_path = assets_root_path + config["usd_path"]

        # 添加到 Stage
        add_reference_to_stage(usd_path=asset_path, prim_path=config["prim_path"])

        # 创建 Articulation 对象
        robot = Articulation(prim_paths_expr=config["prim_path"], name=config["name"])

        # 设置初始位姿，注意除以 get_stage_units()
        robot.set_world_poses(positions=np.array([position]) / get_stage_units())

        return {"status": "success", "message": f"{robot_type} robot created"}

    def create_physics_scene(
            self,
            objects: List[Dict[str, Any]] = [],
            floor: bool = True,
            gravity: List[float] = (0.0, -9.81, 0.0),
            scene_name: str = "physics_scene",
    ) -> Dict[str, Any]:
        """Create a physics scene with multiple objects."""
        try:
            from isaacsim.core.api import World
            from isaacsim.core.api.objects import (
                DynamicCuboid,
                DynamicSphere,
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
