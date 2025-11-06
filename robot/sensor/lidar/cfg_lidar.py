# Third-party library imports
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class CfgLidar:
    type: str = "lidar"
    # 雷达的名字, 同一个机器人上的雷达名字需要不一样
    name: str = "name"
    # Lidar 在 USD 舞台中的路径
    prim_path: Optional[str] = None
    # 相对于parent prim的偏移位置
    translation: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    # 相对于机器人中心的四元数朝向 (WXYZ)
    quat: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    """
    要使用的Lidar配置文件名 (去掉 .json 后缀), "
        "对应 .../{python所在的路径}/lib/python3.10/site-packages/isaacsim/exts/isaacsim.sensors.rtx/data/lidar_configs 目录下的文件"
    如果用自定义的, 需要自己将lidar 的配置文件放到上述的文件夹中, 比如autel_perception_120x352
    """
    config_file_name: str = "Hesai_XT32_SD10"
    # 雷达的输出维度, 和json中的配置要一致
    output_size: Optional[Tuple[int, int]] = None
    # 雷达的最大深度, 如果超过该深度, 则会被设置成这个数值
    max_depth: float = 1000.0
    # 等距投影图像宽度（像素）
    erp_width: int = 120
    # 等距投影图像高度（像素)
    erp_height: int = 352
    # 水平视场角（度）
    erp_width_fov: float = 90.0
    # 垂直视场角（度）
    erp_height_fov: float = 270.0
    # Lidar 的扫描频率 (Hz)
    frequency: int = 10
    # 是否自动附加数据读取器以获取数据
    attach_annotator: bool = True
