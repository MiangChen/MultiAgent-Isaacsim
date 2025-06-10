from typing import List, Optional, Tuple
from pydantic import BaseModel
# from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.nucleus import get_assets_root_path

assets_root_path = get_assets_root_path()  # 不可以放在
if assets_root_path is None:
    print("Could not find nucleus server with /Isaac folder")

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
