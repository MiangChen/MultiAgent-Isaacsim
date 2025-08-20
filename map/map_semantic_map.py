
import numpy as np
import omni
# from isaacsim.asset.gen.omap.bindings import _omap


class MapSemantic():
    def __init__(self,):
        import yaml
        with open('/home/ubuntu/multiagent-isaacsimROS/src/multiagent_isaacsim/multiagent_isaacsim/files/env_cfg.yaml', 'r') as f:
            cfg = yaml.safe_load(f)
        self.map_semantic = cfg['map_semantic']
