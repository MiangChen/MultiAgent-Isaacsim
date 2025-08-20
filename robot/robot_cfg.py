from typing import List, Optional, Tuple
from pydantic import BaseModel

from files.variables import ASSET_PATH

class BaseCfg(BaseModel):
    def update(self, **kwargs):
        return self.model_copy(update=kwargs, deep=True)


class RobotCfg(BaseCfg):
    # meta info
    name_prefix: str = 'robot'
    type: str = 'robot'
    prim_path: str = '/World/robots'
    usd_path: str

    # common config
    position: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)
    euler_degree: Optional[Tuple[float, float, float]] = (0, 0, 90)  # 角度模式
    quat: Optional[Tuple[float, float, float, float]] = (0.0, 0.0, 0.0, 1.0)  # 使用4元数的方法. euler_degree优先级更高
    scale: Optional[Tuple[float, float, float]] = (1.0, 1.0, 1.0)
    # controllers: Optional[List[ControllerCfg]] = None
    # cameras: Optional[List[SensorCfg]] = None
    camera_path: Optional[str]  = None