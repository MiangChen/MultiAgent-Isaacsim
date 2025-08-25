开发阶段，使用中文来进行记录, 基于中文发布README，最后发布后再提供一个english翻译

根据isaaclab要求，使用python 3.10

按照官方文档来安装IsaacSim和IsaacLab:
https://docs.robotsfan.com/isaaclab/source/setup/installation/pip_installation.

更新MVP的城市场景，说明在```scene```文件夹里，下载链接为：https://pan.quark.cn/s/0694d8c27c6c



# 安装环境

这部分参考了官方的Isaaclab和Isaacsim的安装过程, 如果按照官方的安装完了环境, 可以跳过, 如果没安装成功, 可以参考下面的代码(已经修复过): 

```
conda create -n env_isaaclab python=3.10
conda activate env_isaaclab
```

安装torch的cuda版本

```

pip install --upgrade pip # ubuntu
conda install pip==24 # windows
pip cache purge
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
```

安装isaacsim

```
pip install 'isaacsim[all,extscache]==4.5.0' --extra-index-url https://pypi.nvidia.com
```

验证isaacsim安装成功, 首次启动约10min内:

```
isaacsim
```

安装Isaaclab

```
cd <项目的位置>
git clone https://github.com/isaac-sim/IsaacLab.git

sudo apt install cmake build-essential # ubuntu 系统

cd IsaacLab
./isaaclab.sh --install # or "./isaaclab.sh -i" # ubuntu系统
isaaclab.bat --install :: or "isaaclab.bat -i" # windows系统
```

验证Isaaclab安装

```
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py # ubuntu系统

isaaclab.bat -p scripts\tutorials\00_sim\create_empty.py # windows系统
```



安装本次项目的其他依赖:

```
pip install -r requirements.txt
```



# 运行



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

***
***

主要把启动文件main.py从线性逻辑修改成持续性运行逻辑，

ros2节点改是env下的一个属性

节点的基类在ros/ros_swarm.py下   
ros/msg是消息类型文件

目前能实现两条信息通路：

用户在Isaac里直接拖拽资产，删除资产。下层的node会订阅Isaac的事件，然后发布消息，语义层的node收到消息，回调会在task_context里面做相应的变更，变更包括删除对应边和节点，修改节点的shape属性。
发布所有时间步的plan到下层。因为平台的具体执行还没有很完善，所以目前只是开环的任务发布。

在实现中，为了实现语意图和仿真场景的对齐，语意图的节点id应该要和isaac中资产prim路径的最后一个'/'后面的内容相同
比如语意图中有“building1”，那么Isaac中也应该要有“/stage/building1”这样一个prim

***

[![Lines of Code](https://img.shields.io/badge/LoC-32328-blue)](https://github.com/MiangChen/multiagent-isaacsim)

cloc . --exclude-dir=IsaacLab 
