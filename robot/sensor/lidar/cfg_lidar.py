# =============================================================================
# Config LiDAR Module - LiDAR Sensor Configuration
# =============================================================================
#
# This module provides configuration parameters for RTX LiDAR sensors,
# including positioning, scanning parameters, and output settings.
#
# =============================================================================

# Standard library imports
from typing import Optional, Tuple

# Third-party library imports
from pydantic import Field

# Local project imports
from config.cfg_base import CfgBase


class CfgLidar(CfgBase):
    """
    配置类，用于定义一个 RTX Lidar 传感器的属性。
    """

    type: str = Field(default="lidar", description="传感器类型")
    name: str = Field(default="name", description="雷达的名字, 同一个机器人上的雷达名字需要不一样")
    prim_path: str = Field(
        default=None, description="Lidar 在 USD 舞台中的路径"
    )

    # --- 位姿设置 ---
    translation: Tuple[float, float, float] = Field(
        default=(0.0, 0.0, 0.0), description="相对于parent prim的偏移位置"
    )
    quat: Tuple[float, float, float, float] = Field(
        default=(1.0, 0.0, 0.0, 0.0), description="相对于机器人中心的四元数朝向 (WXYZ)"
    )

    # --- 核心 Lidar 参数 ---
    #  Lidar 物理特性的 JSON 配置文件
    config_file_name: str = Field(
        default="Hesai_XT32_SD10",
        description="要使用的Lidar配置文件名 (去掉 .json 后缀), "
                    "对应 .../{python所在的路径}/lib/python3.10/site-packages/isaacsim/exts/isaacsim.sensors.rtx/data/lidar_configs 目录下的文件"
                    "如果用自定义的, 需要自己将lidar 的配置文件放到上述的文件夹中, 比如autel_perception_120x352"
    )
    output_size: Tuple[int, int] = Field(description="雷达的输出维度")
    max_depth: float = Field(default=1000, description="雷达的最大深度, 如果超过该深度, 则会被设置成这个数值")

    # --- ERP 投影参数 ---
    erp_width: int = Field(default=120, description="等距投影图像宽度（像素）")
    erp_height: int = Field(default=352, description="等距投影图像高度（像素）")
    erp_width_fov: float = Field(default=90.0, description="水平视场角（度）")
    erp_height_fov: float = Field(default=270.0, description="垂直视场角（度）")

    # --- 性能和数据设置 ---
    # Lidar 的数据更新频率
    frequency: int = Field(default=10, description="Lidar 的扫描频率 (Hz)")

    # 是否在初始化时自动添加数据读取器 (Annotator)
    attach_annotator: bool = Field(
        default=True, description="是否自动附加数据读取器以获取数据"
    )
