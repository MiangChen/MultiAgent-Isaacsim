#!/usr/bin/env python3
"""
ViewportManager Enhanced - 简洁高效的viewport管理器
负责管理viewport和camera之间的映射关系，专注于核心功能实现
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
        if not self._is_valid_viewport_name(name):
            return False
        
        if viewport_obj is None:
            return False
        
        # 检查是否已存在，给出覆盖警告
        if name in self._viewports:
            print(f"Warning: Viewport '{name}' already exists, overwriting previous registration")
        
        self._viewports[name] = viewport_obj
        self._viewport_sources[name] = "manual"
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
        
        # 清理camera映射
        if name in self._mappings:
            camera_path = self._mappings[name]
            del self._mappings[name]
            
            # 清理反向映射
            if camera_path in self._reverse_mappings:
                self._reverse_mappings[camera_path].remove(name)
                if not self._reverse_mappings[camera_path]:
                    del self._reverse_mappings[camera_path]
        
        return True
    
    def get_viewport(self, name: str) -> Optional[Any]:
        """
        获取指定名称的viewport对象
        
        Args:
            name: viewport名称
            
        Returns:
            Optional[Any]: viewport对象，不存在返回None
        """
        if not self._is_valid_viewport_name(name):
            return None
        
        return self._viewports.get(name)
    
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
        # 验证camera路径有效性 (Requirements 6.1)
        if not self._is_valid_camera_path(camera_path):
            return False
        
        # 验证viewport名称和注册状态
        if not self._is_valid_viewport_name(viewport_name) or viewport_name not in self._viewports:
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
        
        # 清理正向映射 (Requirements 6.3)
        del self._mappings[viewport_name]
        
        # 清理反向映射 (Requirements 6.3)
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
            Optional[str]: camera路径，不存在映射返回None (Requirements 6.2)
        """
        if not self._is_valid_viewport_name(viewport_name):
            return None
        
        return self._mappings.get(viewport_name)
    
    def list_camera_mappings(self) -> Dict[str, str]:
        """
        列出所有viewport到camera的映射关系
        
        Returns:
            Dict[str, str]: 完整的映射信息字典 (Requirements 6.4)
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
            bool: 切换成功返回True，失败返回False (Requirements 2.1, 2.4)
        """
        # 验证camera路径有效性 (Requirements 2.3)
        if not self._is_valid_camera_path(camera_prim_path):
            return False
        
        # 确定目标viewport对象 (Requirements 2.2)
        target_viewport = None
        
        if viewport_name:
            # 使用指定的viewport名称
            if not self._is_valid_viewport_name(viewport_name):
                return False
            target_viewport = self.get_viewport(viewport_name)
        else:
            # 使用当前活跃的viewport (Requirements 2.2)
            target_viewport = self._get_active_viewport()
        
        # 检查viewport对象是否有效
        if target_viewport is None:
            return False
        
        # 执行viewport camera切换 (Requirements 2.4)
        success = self._set_viewport_camera(target_viewport, camera_prim_path)
        
        # 如果切换成功且有viewport名称，更新映射关系
        if success and viewport_name:
            self.map_camera(viewport_name, camera_prim_path)
        
        return success
    
    # === 基础验证私有方法 ===
    
    def _is_valid_camera_path(self, camera_path: str) -> bool:
        """
        检查camera路径的有效性
        
        Args:
            camera_path: camera的prim路径
            
        Returns:
            bool: 路径有效返回True，无效返回False
        """
        return bool(camera_path and isinstance(camera_path, str) and camera_path.strip())
    
    def _is_valid_viewport_name(self, viewport_name: str) -> bool:
        """
        检查viewport名称的有效性
        
        Args:
            viewport_name: viewport名称
            
        Returns:
            bool: 名称有效返回True，无效返回False
        """
        return bool(viewport_name and isinstance(viewport_name, str) and viewport_name.strip())
    
    # === 批量robot viewport注册 ===
    
    def register_robot_viewports(self, robots: List[Any]) -> int:
        """
        批量注册robot的viewport信息
        
        Args:
            robots: robot对象列表
            
        Returns:
            int: 成功注册的viewport数量 (Requirements 5.1, 5.3)
        """
        if not robots or not isinstance(robots, list):
            return 0
        
        success_count = 0
        
        for robot in robots:
            # 提取robot的viewport信息 (Requirements 5.2)
            viewport_name, viewport_obj = self._extract_robot_viewport_info(robot)
            
            if viewport_name and viewport_obj:
                # 使用现有的register_viewport方法注册
                if self.register_viewport(viewport_name, viewport_obj):
                    # 标记来源为robot
                    self._viewport_sources[viewport_name] = "robot"
                    success_count += 1
                    
                    # 如果robot有camera路径信息，也建立映射关系
                    camera_path = self._extract_robot_camera_path(robot)
                    if camera_path:
                        self.map_camera(viewport_name, camera_path)
            # 跳过无效的robot (Requirements 5.4)
        
        return success_count
    
    def _extract_robot_viewport_info(self, robot: Any) -> Tuple[Optional[str], Optional[Any]]:
        """
        从robot对象中提取viewport信息
        
        Args:
            robot: robot对象
            
        Returns:
            Tuple[Optional[str], Optional[Any]]: (viewport_name, viewport_obj)
        """
        viewport_name = None
        viewport_obj = None
        
        try:
            # 优先使用robot的get_viewport_info方法 (新的RobotBase集成)
            if hasattr(robot, 'get_viewport_info'):
                viewport_info = robot.get_viewport_info()
                if viewport_info and viewport_info.get('enabled'):
                    viewport_name = viewport_info.get('viewport_name')
                    # 尝试获取viewport对象
                    if viewport_name:
                        viewport_obj = self._get_viewport_by_name(viewport_name)
            
            # 回退到旧的方式：从robot配置中获取viewport名称 (Requirements 5.2)
            elif hasattr(robot, 'cfg_body') and hasattr(robot.cfg_body, 'id'):
                viewport_name = f"Viewport_Robot_{robot.cfg_body.id}"
                viewport_obj = self._get_viewport_by_name(viewport_name)
            elif hasattr(robot, 'id'):
                viewport_name = f"Viewport_Robot_{robot.id}"
                viewport_obj = self._get_viewport_by_name(viewport_name)
            elif hasattr(robot, 'name_prefix') and hasattr(robot, 'id'):
                viewport_name = f"Viewport_{robot.name_prefix}_{robot.id}"
                viewport_obj = self._get_viewport_by_name(viewport_name)
        
        except (AttributeError, Exception):
            # 如果提取失败，返回None值 (Requirements 5.4)
            pass
        
        return viewport_name, viewport_obj
    
    def _extract_robot_camera_path(self, robot: Any) -> Optional[str]:
        """
        从robot对象中提取camera路径信息
        
        Args:
            robot: robot对象
            
        Returns:
            Optional[str]: camera路径，提取失败返回None
        """
        try:
            # 优先使用robot的get_viewport_info方法 (新的RobotBase集成)
            if hasattr(robot, 'get_viewport_info'):
                viewport_info = robot.get_viewport_info()
                if viewport_info and viewport_info.get('enabled'):
                    camera_path = viewport_info.get('camera_path')
                    if camera_path:
                        return camera_path
            
            # 回退到旧的方式：直接从robot属性获取camera路径
            if hasattr(robot, 'camera_prim_path') and robot.camera_prim_path:
                return robot.camera_prim_path
            elif hasattr(robot, 'cfg_body') and hasattr(robot.cfg_body, 'camera_path'):
                return robot.cfg_body.camera_path
            elif hasattr(robot, 'cfg_body') and hasattr(robot.cfg_body, 'id'):
                # 根据robot ID构造标准camera路径
                return f"/World/Robot_{robot.cfg_body.id}_Camera"
        
        except (AttributeError, Exception):
            pass
        
        return None
    
    def _get_viewport_by_name(self, viewport_name: str) -> Optional[Any]:
        """
        通过名称获取viewport对象
        
        Args:
            viewport_name: viewport名称
            
        Returns:
            Optional[Any]: viewport对象，获取失败返回None
        """
        if not self._is_valid_viewport_name(viewport_name):
            return None
        
        # 首先检查是否已经注册
        if viewport_name in self._viewports:
            return self._viewports[viewport_name]
        
        # 尝试通过Isaac Sim API获取
        try:
            from isaacsim.core.utils.viewports import get_viewport_from_window_name
            return get_viewport_from_window_name(viewport_name)
        except (ImportError, Exception):
            return None
    
    # === Isaac Sim API封装私有方法 ===
    
    def _get_active_viewport(self) -> Optional[Any]:
        """
        获取当前活跃的viewport对象
        封装Isaac Sim API调用，仅在此方法中使用try-except处理API异常
        
        Returns:
            Optional[Any]: 活跃的viewport对象，获取失败返回None
        """
        try:
            from omni.kit.viewport.utility import get_active_viewport
            return get_active_viewport()
        except (ImportError, Exception):
            # Isaac Sim API不可用或调用失败时返回None
            return None
    
    def _set_viewport_camera(self, viewport: Any, camera_path: str) -> bool:
        """
        设置viewport的camera路径
        封装Isaac Sim API调用，仅在此方法中使用try-except处理API异常
        
        Args:
            viewport: viewport对象
            camera_path: camera的prim路径
            
        Returns:
            bool: 设置成功返回True，失败返回False
        """
        if viewport is None or not self._is_valid_camera_path(camera_path):
            return False
        
        try:
            # 使用Isaac Sim标准API设置viewport的camera路径
            viewport.camera_path = camera_path
            return True
        except (AttributeError, Exception):
            # viewport对象无camera_path属性或设置失败时返回False
            return False


# 全局viewport管理器实例
viewport_manager = ViewportManager()


# === 模块级别的便捷函数 ===

def register_viewport(name: str, viewport_obj: Any) -> bool:
    """
    便捷函数：注册viewport对象
    
    Args:
        name: viewport名称
        viewport_obj: viewport对象
        
    Returns:
        bool: 注册成功返回True，失败返回False
    """
    return viewport_manager.register_viewport(name, viewport_obj)


def unregister_viewport(name: str) -> bool:
    """
    便捷函数：注销viewport并清理相关映射
    
    Args:
        name: viewport名称
        
    Returns:
        bool: 注销成功返回True，不存在或失败返回False
    """
    return viewport_manager.unregister_viewport(name)


def get_viewport(name: str) -> Optional[Any]:
    """
    便捷函数：获取指定名称的viewport对象
    
    Args:
        name: viewport名称
        
    Returns:
        Optional[Any]: viewport对象，不存在返回None
    """
    return viewport_manager.get_viewport(name)


def list_viewports() -> List[str]:
    """
    便捷函数：列出所有已注册的viewport名称
    
    Returns:
        List[str]: viewport名称列表
    """
    return viewport_manager.list_viewports()


def map_camera(viewport_name: str, camera_path: str) -> bool:
    """
    便捷函数：建立viewport到camera的映射关系
    
    Args:
        viewport_name: viewport名称
        camera_path: camera的prim路径
        
    Returns:
        bool: 映射成功返回True，失败返回False
    """
    return viewport_manager.map_camera(viewport_name, camera_path)


def unmap_camera(viewport_name: str) -> bool:
    """
    便捷函数：清理viewport的camera映射关系
    
    Args:
        viewport_name: viewport名称
        
    Returns:
        bool: 清理成功返回True，不存在映射返回False
    """
    return viewport_manager.unmap_camera(viewport_name)


def get_camera_path(viewport_name: str) -> Optional[str]:
    """
    便捷函数：查询viewport对应的camera路径
    
    Args:
        viewport_name: viewport名称
        
    Returns:
        Optional[str]: camera路径，不存在映射返回None
    """
    return viewport_manager.get_camera_path(viewport_name)


def change_viewport(camera_prim_path: str, viewport_name: str = None) -> bool:
    """
    便捷函数：切换指定viewport的camera视角
    
    Args:
        camera_prim_path: camera的prim路径
        viewport_name: viewport名称，为空时使用当前活跃的viewport
        
    Returns:
        bool: 切换成功返回True，失败返回False
    """
    return viewport_manager.change_viewport(camera_prim_path, viewport_name)


def register_robot_viewports(robots: List[Any]) -> int:
    """
    便捷函数：批量注册robot的viewport信息
    
    Args:
        robots: robot对象列表
        
    Returns:
        int: 成功注册的viewport数量
    """
    return viewport_manager.register_robot_viewports(robots)


def list_camera_mappings() -> Dict[str, str]:
    """
    便捷函数：列出所有viewport到camera的映射关系
    
    Returns:
        Dict[str, str]: 完整的映射信息字典
    """
    return viewport_manager.list_camera_mappings()


# === 向后兼容性接口 ===

def get_viewport_manager() -> ViewportManager:
    """
    获取全局viewport管理器实例
    提供向后兼容性支持
    
    Returns:
        ViewportManager: 全局viewport管理器实例
    """
    return viewport_manager


# 向后兼容的别名函数
def switch_viewport(camera_prim_path: str, viewport_name: str = None) -> bool:
    """
    向后兼容的别名函数：切换viewport
    
    Args:
        camera_prim_path: camera的prim路径
        viewport_name: viewport名称，为空时使用当前活跃的viewport
        
    Returns:
        bool: 切换成功返回True，失败返回False
    """
    return change_viewport(camera_prim_path, viewport_name)


def add_viewport(name: str, viewport_obj: Any) -> bool:
    """
    向后兼容的别名函数：添加viewport
    
    Args:
        name: viewport名称
        viewport_obj: viewport对象
        
    Returns:
        bool: 添加成功返回True，失败返回False
    """
    return register_viewport(name, viewport_obj)


def remove_viewport(name: str) -> bool:
    """
    向后兼容的别名函数：移除viewport
    
    Args:
        name: viewport名称
        
    Returns:
        bool: 移除成功返回True，不存在或失败返回False
    """
    return unregister_viewport(name)