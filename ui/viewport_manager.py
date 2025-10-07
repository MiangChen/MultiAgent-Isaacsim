
"""
ViewportManager Enhanced - 简洁高效的viewport管理器
负责管理viewport和camera之间的映射关系，专注于核心功能实现。
所有功能均通过ViewportManager类提供，通过 get_viewport_manager() 获取全局实例。
"""

from typing import Dict, List, Optional, Any, Tuple


class ViewportManager:
    """
    简洁高效的viewport管理器

    核心功能:
    1. viewport的注册和管理
    2. viewport-camera映射关系
    3. viewport切换功能
    4. 批量robot viewport操作
    """

    def __init__(self):
        """初始化ViewportManager的核心数据结构"""
        # viewport注册表 (name -> viewport_obj)
        self._viewports: Dict[str, Any] = {}

        # viewport->camera映射 (viewport_name -> camera_path)
        self._mappings: Dict[str, str] = {}

        # camera->viewports反向映射 (camera_path -> viewport_names)
        self._reverse_mappings: Dict[str, List[str]] = {}

        # viewport来源记录 (viewport_name -> source_type)
        self._viewport_sources: Dict[str, str] = {}

    # === Viewport注册管理方法 ===

    def register_viewport(self, name: str, viewport_obj: Any) -> bool:
        """
        注册viewport对象

        Args:
            name: viewport名称
            viewport_obj: viewport对象

        Returns:
            bool: 注册成功返回True，失败返回False
        """
        if not self._is_valid_viewport_name(name) or viewport_obj is None:
            return False

        # 检查是否已存在，给出覆盖警告
        if name in self._viewports:
            print(f"Warning: Viewport '{name}' already exists, overwriting previous registration")

        self._viewports[name] = viewport_obj
        return True

    def unregister_viewport(self, name: str) -> bool:
        """
        注销viewport并清理相关映射

        Args:
            name: viewport名称

        Returns:
            bool: 注销成功返回True，不存在或失败返回False
        """
        if not self._is_valid_viewport_name(name) or name not in self._viewports:
            return False

        # 清理viewport注册和来源记录
        del self._viewports[name]
        self._viewport_sources.pop(name, None)

        # 清理camera映射 (复用unmap_camera逻辑)
        if name in self._mappings:
            self.unmap_camera(name)

        return True

    def get_viewport(self, name: str) -> Optional[Any]:
        """
        获取指定名称的viewport对象

        Args:
            name: viewport名称

        Returns:
            Optional[Any]: viewport对象，不存在返回None
        """
        if self._is_valid_viewport_name(name):
            return self._viewports.get(name)

        return None

    def list_viewports(self) -> List[str]:
        """
        列出所有已注册的viewport名称

        Returns:
            List[str]: viewport名称列表
        """
        return list(self._viewports.keys())

    # === Camera映射管理方法 ===

    def map_camera(self, viewport_name: str, camera_path: str) -> bool:
        """
        建立viewport到camera的映射关系

        Args:
            viewport_name: viewport名称
            camera_path: camera的prim路径

        Returns:
            bool: 映射成功返回True，失败返回False
        """
        # 验证输入
        if not self._is_valid_camera_path(camera_path) or \
                not self._is_valid_viewport_name(viewport_name) or \
                viewport_name not in self._viewports:
            return False

        # 清理旧的映射关系
        if viewport_name in self._mappings:
            old_camera_path = self._mappings[viewport_name]
            if old_camera_path in self._reverse_mappings:
                self._reverse_mappings[old_camera_path].remove(viewport_name)
                if not self._reverse_mappings[old_camera_path]:
                    del self._reverse_mappings[old_camera_path]

        # 建立新的映射关系
        self._mappings[viewport_name] = camera_path

        # 建立反向映射
        if camera_path not in self._reverse_mappings:
            self._reverse_mappings[camera_path] = []
        self._reverse_mappings[camera_path].append(viewport_name)

        return True

    def unmap_camera(self, viewport_name: str) -> bool:
        """
        清理viewport的camera映射关系

        Args:
            viewport_name: viewport名称

        Returns:
            bool: 清理成功返回True，不存在映射返回False
        """
        if not self._is_valid_viewport_name(viewport_name) or viewport_name not in self._mappings:
            return False

        camera_path = self._mappings[viewport_name]

        # 清理正向映射
        del self._mappings[viewport_name]

        # 清理反向映射
        if camera_path in self._reverse_mappings:
            self._reverse_mappings[camera_path].remove(viewport_name)
            if not self._reverse_mappings[camera_path]:
                del self._reverse_mappings[camera_path]

        return True

    def get_camera_path(self, viewport_name: str) -> Optional[str]:
        """
        查询viewport对应的camera路径

        Args:
            viewport_name: viewport名称

        Returns:
            Optional[str]: camera路径，不存在映射返回None
        """
        if not self._is_valid_viewport_name(viewport_name):
            return None
        return self._mappings.get(viewport_name)

    def list_camera_mappings(self) -> Dict[str, str]:
        """
        列出所有viewport到camera的映射关系

        Returns:
            Dict[str, str]: 完整的映射信息字典
        """
        return self._mappings.copy()

    # === Viewport切换核心功能 ===

    def change_viewport(self, camera_prim_path: str, viewport_name: str = None) -> bool:
        """
        切换指定viewport的camera视角

        Args:
            camera_prim_path: camera的prim路径
            viewport_name: viewport名称，为空时使用当前活跃的viewport

        Returns:
            bool: 切换成功返回True，失败返回False
        """
        if not self._is_valid_camera_path(camera_prim_path):
            return False

        target_viewport = self.get_viewport(viewport_name)

        if target_viewport is None:
            return False

        success = self._set_viewport_camera(target_viewport, camera_prim_path)

        # 如果切换成功且指定了viewport名称，则更新映射
        if success and viewport_name:
            self.map_camera(viewport_name, camera_prim_path)

        return success

    # === 批量robot viewport注册 ===

    def register_robot_viewports(self, robots: List[Any]) -> int:
        """
        批量注册robot的viewport信息

        Args:
            robots: robot对象列表

        Returns:
            int: 成功注册的viewport数量
        """
        if not isinstance(robots, list):
            return 0

        success_count = 0
        for robot in robots:
            viewport_name, viewport_obj = self._extract_robot_viewport_info(robot)

            # 确保提取到有效信息，并进行注册
            if viewport_name and viewport_obj and self.register_viewport(viewport_name, viewport_obj):
                self._viewport_sources[viewport_name] = "robot"
                success_count += 1

                # 尝试提取并映射camera
                camera_path = self._extract_robot_camera_path(robot)
                if camera_path:
                    self.map_camera(viewport_name, camera_path)

        return success_count

    # === 私有方法 ===

    def _is_valid_camera_path(self, camera_path: str) -> bool:
        """检查camera路径的有效性"""
        return bool(camera_path and isinstance(camera_path, str) and camera_path.strip())

    def _is_valid_viewport_name(self, viewport_name: str) -> bool:
        """检查viewport名称的有效性"""
        return bool(viewport_name and isinstance(viewport_name, str) and viewport_name.strip())

    def _extract_robot_viewport_info(self, robot: Any) -> Tuple[Optional[str], Optional[Any]]:
        """从robot对象中提取viewport信息"""
        viewport_name = None
        viewport_obj = None

        try:
            if hasattr(robot, 'get_viewport_info'):
                viewport_info = robot.get_viewport_info()
                if viewport_info and viewport_info.get('enabled'):
                    viewport_name = viewport_info.get('viewport_name')
                    if viewport_name:
                        viewport_obj = self._get_viewport_by_name(viewport_name)

            elif hasattr(robot, 'cfg_robot') and hasattr(robot.cfg_robot, 'id'):
                viewport_name = f"Viewport_Robot_{robot.cfg_robot.id}"
                viewport_obj = self._get_viewport_by_name(viewport_name)
            elif hasattr(robot, 'id'):
                viewport_name = f"Viewport_Robot_{robot.id}"
                viewport_obj = self._get_viewport_by_name(viewport_name)
            elif hasattr(robot, 'type') and hasattr(robot, 'id'):
                viewport_name = f"Viewport_{robot.type}_{robot.id}"
                viewport_obj = self._get_viewport_by_name(viewport_name)

        except (AttributeError, Exception):
            pass

        return viewport_name, viewport_obj

    def _extract_robot_camera_path(self, robot: Any) -> Optional[str]:
        """从robot对象中提取camera路径信息"""
        try:
            if hasattr(robot, 'get_viewport_info'):
                viewport_info = robot.get_viewport_info()
                if viewport_info and viewport_info.get('enabled'):
                    camera_path = viewport_info.get('camera_path')
                    if camera_path:
                        return camera_path

            if hasattr(robot, 'camera_prim_path') and robot.camera_prim_path:
                return robot.camera_prim_path
            elif hasattr(robot, 'cfg_robot') and hasattr(robot.cfg_robot, 'camera_path'):
                return robot.cfg_robot.camera_path
            elif hasattr(robot, 'cfg_robot') and hasattr(robot.cfg_robot, 'id'):
                return f"/World/Robot_{robot.cfg_robot.id}_Camera"

        except (AttributeError, Exception):
            pass

        return None

    def _get_viewport_by_name(self, viewport_name: str) -> Optional[Any]:
        """通过名称获取viewport对象"""
        if not self._is_valid_viewport_name(viewport_name):
            return None

        if viewport_name in self._viewports:
            return self._viewports[viewport_name]

    def _set_viewport_camera(self, viewport: Any, camera_path: str) -> bool:
        """设置viewport的camera路径"""
        if viewport is None or not self._is_valid_camera_path(camera_path):
            return False
        try:
            viewport.camera_path = camera_path
            return True
        except (AttributeError, Exception):
            return False
