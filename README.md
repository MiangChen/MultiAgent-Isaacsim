开发阶段，使用中文来进行记录, 集群和AI都基本是华人在做，基于中文发布README，最后再提供一个english翻译

根据isaaclab要求，使用python 3.10

按照官方文档来安装IsaacSim和IsaacLab:
https://docs.robotsfan.com/isaaclab/source/setup/installation/pip_installation.

更新MVP的城市场景，说明在```scene```文件夹里，下载链接为：https://pan.quark.cn/s/0694d8c27c6c

```
pip install -r requirements.txt
```

运行
```
python main.py --enable isaacsim.asset.gen.omap
```

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


之所以用skill, 是为了和action区分, action一般是底层的关节动作, gym里就是用action表示电机或者铰链的控制量
skill是上层用的


[![Lines of Code](https://img.shields.io/badge/LoC-2055-blue)](https://github.com/MiangChen/multiagent-isaacsim)

cloc . --exclude-dir=IsaacLab --exclude-dir=PX4-Autopilot

