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
    name_prefix: str
    type: str
    prim_path: str
    usd_path: Optional[str] = None  # If Optional, use default usd_path

    # common config
    position: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)
    orientation: Optional[Tuple[float, float, float, float]] = (0.0, 0.0, 0.0, 1.0)
    scale: Optional[Tuple[float, float, float]] = (1.0, 1.0, 1.0)
    # controllers: Optional[List[ControllerCfg]] = None
    # cameras: Optional[List[SensorCfg]] = None
