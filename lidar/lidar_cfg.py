from typing import Optional, Tuple
from pydantic import BaseModel, Field

# 导入 CameraCfg 中使用的 BaseCfg，以保持一致性
from camera.camera_cfg import BaseCfg


class LidarCfg(BaseCfg):
    """
    配置类，用于定义一个 RTX Lidar 传感器的属性。
    """
    name_prefix: Optional[str] = Field(default='lidar', description="Lidar的名称前缀")
    type: Optional[str] = Field(default='lidar', description="传感器类型")
    prim_path: Optional[str] = Field(default=None, description="Lidar 在 USD 舞台中的路径")

    # --- 位姿设置 ---
    position: Tuple[float, float, float] = Field(default=(0.0, 0.0, 0.0), description="相对于机器人中心的偏移位置")
    quat: Optional[Tuple[float, float, float, float]] = Field(default=(1.0, 0.0, 0.0, 0.0),
                                                              description="相对于机器人中心的四元数朝向 (WXYZ)")

    # --- 核心 Lidar 参数 ---
    # 这个参数指向一个定义了 Lidar 物理特性的 JSON 配置文件
    config_file_name: str = Field(
        default="Simple_Example_Solid_State",
        description="要使用的Lidar配置文件名 (无需 .json 后缀), "
                    "对应 isaacsim.sensors.rtx/data/lidar_configs/ 目录下的文件"
    )

    # --- 性能和数据设置 ---
    # Lidar 的数据更新频率
    frequency: Optional[int] = Field(default=10, description="Lidar 的扫描频率 (Hz)")

    # 是否在初始化时自动添加数据读取器 (Annotator)
    attach_annotator: bool = Field(default=True, description="是否自动附加数据读取器以获取数据")