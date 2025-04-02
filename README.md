
开发阶段，使用中文来进行记录, 集群和AI都基本是华人在做，基于中文发布README，最后再提供一个english翻译

根据isaaclab要求，使用python 3.10



pip install -r requirements.txt


***
随手记录 开发记录
IsaacSim
从Grutopia吸取经验，

    def __init__(self, simulator_runtime: SimulatorRuntime) -> None:
        self._render = None
        self._runtime = simulator_runtime

from grutopia.core.runtime import SimulatorRuntime
底层里面有一个这个：
        # Init Isaac Sim
        from isaacsim import SimulationApp  # noqa

        self.headless = headless
        self._simulation_app = SimulationApp(
            {'headless': self.headless, 'anti_aliasing': 0, 'hide_ui': False, 'multi_gpu': False}
        )

[![Lines of Code](https://img.shields.io/badge/LoC-1442-blue)](https://github.com/MiangChen/multiagent-isaacsim)

cloc . --exclude-dir=IsaacLab
