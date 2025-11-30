# Omni Anim People API 探索文档

## 概述

本文档记录了对 Isaac Sim 4.5 中 `omni.anim.people` 模块的 API 探索结果。

**模块路径**: `omni.anim.people`  
**版本**: Isaac Sim 4.5.0  
**扩展版本**: omni.anim.people-0.6.7+106.5.0

## ⚠️ 重要提示

**角色 USD 文件路径要求**:
- ✅ **必须使用**: `/Characters/Reallusion/{Character}/{Character}.usd` （根目录）
- ❌ **不要使用**: `/Characters/Reallusion/{Character}/Props/Bones/{Character}.usd` （骨骼文件，无网格）
- ❌ **不要使用**: `/AnimGraph/.../Props/Bones/{Character}.*.usd` （动画骨骼文件）

详见 [重要：角色 USD 文件路径要求](#重要角色-usd-文件路径要求) 章节。

## 模块结构

### 顶层模块

```python
from omni.anim.people import scripts

dir(scripts) = [
    '__doc__', '__file__', '__loader__', '__name__', 
    '__package__', '__path__', '__spec__', 
    'custom_command'
]
```

### 核心子模块

```python
from omni.anim.people.scripts import character_behavior
from omni.anim.people.scripts import navigation_manager  # 存在但需要单独导入
```

**注意**: `common` 模块不存在，无法导入 `CharacterState`

## CharacterBehavior 类

### 基本信息

**类文档**: Character controller class that reads commands from a command file and drives character actions.

**构造函数签名**:
```python
CharacterBehavior(prim_path: pxr.Sdf.Path)
```

**参数**:
- `prim_path`: 角色的 USD prim 路径（Sdf.Path 类型）

### 关键属性

创建实例后可用的属性：

```python
behavior = CharacterBehavior(Sdf.Path("/World/characters/worker_0"))

# 核心属性
behavior.prim_path              # Sdf.Path: 角色路径
behavior.character              # 角色对象（运行时初始化）
behavior.character_name         # str: 角色名称
behavior.character_manager      # 角色管理器
behavior.navigation_manager     # NavigationManager: 导航管理器
behavior.settings               # carb.settings.ISettings: 设置对象

# 命令相关
behavior.commands               # list: 命令队列
behavior.current_command        # Command: 当前执行的命令
behavior.command_path           # str: 命令文件路径

# 状态标志
behavior.navmeshEnabled         # bool: 是否启用 NavMesh
behavior.avoidanceOn            # bool: 是否启用动态避障
behavior.interruptable          # bool: 是否可中断
behavior.loop_commands          # bool: 是否循环命令
behavior.in_queue               # bool: 是否在队列中

# 管理器
behavior.global_character_manager  # 全局角色管理器
behavior.queue_manager             # 队列管理器
```

### 核心方法

#### 1. 初始化方法

```python
on_init(self)
```
- 当脚本附加到角色或加载 stage 时调用
- 使用 `renew_character_state()` 初始化角色状态

```python
on_play(self)
```
- 进入运行时（点击 play 按钮）时调用
- 使用 `renew_character_state()` 初始化角色状态

```python
init_character(self)
```
- 初始化全局变量并获取附加到角色的 animation graph
- 只能在运行时调用（因为 `ag.get_character()` 只能在运行时使用）

```python
renew_character_state(self)
```
- 定义角色变量并加载设置

#### 2. 更新方法

```python
on_update(self, current_time: float, delta_time: float)
```
- 每帧调用
- 在开始时初始化角色
- 发布角色位置
- 执行角色命令

**参数**:
- `current_time`: 当前时间（秒）
- `delta_time`: 自上次更新以来的时间（秒）

#### 3. 命令管理方法

```python
inject_command(self, command_list, executeImmediately=True)
```
- 将命令注入到当前命令队列

**参数**:
- `command_list`: 用户想要注入的命令信息列表
- `executeImmediately`: 命令是否立即执行或在模拟结束时执行

**命令格式**（待测试）:
```python
# 格式 1: 嵌套列表（推测）
command_list = [
    ["goto", "5", "5", "0"],  # goto x y z
    ["idle", "5"],             # idle duration
]

# 格式 2: 字符串（待验证）
command_list = ["goto 5 5 0"]
```

```python
read_commands_from_file(self)
```
- 从 `self.command_path` 指向的文件读取命令
- 如果指定了队列，则使用队列管理器创建队列

**返回**: 命令列表（python list）

```python
convert_str_to_command(self, cmd_line)
```
- 将字符串命令转换为命令对象

**测试结果**: 返回 `None`（可能需要在运行时或特定上下文中使用）

```python
get_command(self, command_pair)
```
- 根据命令返回命令对象实例

**参数**: `command_pair` - 描述命令的字符串列表

**返回**: 命令对象实例（python object）

**测试结果**: 参数格式不明确（"too many values to unpack"）

```python
execute_command(self, commands, delta_time)
```
- 按顺序执行命令列表中的命令
- 完成后删除命令

**参数**:
- `commands`: 命令列表（list[list]）
- `delta_time`: 自上次执行以来的时间

#### 4. 其他方法

```python
get_current_position(self)
```
- 获取角色当前位置

```python
get_agent_name(self)
```
- 查找此角色资产在命令文件中使用的名称

```python
end_current_command(self, set_status: bool = True)
```
- 结束当前命令

```python
check_interruptable(self)
set_interruptable(self, target_value)
```
- 检查/设置是否可中断

```python
on_destroy(self)
```
- 通过删除全局变量实例来清除角色状态

```python
on_stop(self)
```
- 退出运行时（点击 stop 按钮）时调用
- 使用 `on_destroy()` 清除状态

## NavigationManager 类

### 构造函数签名

```python
NavigationManager(character_name, navmesh_enabled, dynamic_avoidance_enabled=True)
```

**参数**:
- `character_name`: 角色名称
- `navmesh_enabled`: 是否启用 NavMesh
- `dynamic_avoidance_enabled`: 是否启用动态避障（默认 True）

### 主要方法

```python
# 路径生成
generate_path(self)
generate_goto_path(self)

# 路径管理
update_path(self)
update_target_path_progress(self)
clean_path_targets(self)

# 路径点操作
get_path_points(self)
set_path_points(self)
get_path_target_pos(self)
get_path_target_rot(self)
set_path_target_rot(self)

# 状态检查
destination_reached(self)
is_still_moving(self)
check_proximity_to_point(self)

# 避障
detect_collision(self)
get_avoid_angle(self)

# 其他
calculate_rotation_diff(self)
publish_character_positions(self)
destroy(self)
```

## Command 类（基类）

### GoTo 命令

**类文档**: Command class to go to a location/locations.

**构造函数签名**:
```python
GoTo(self, *args, **kwargs)
```

**注意**: 需要 `character` 参数（必需位置参数）

**主要方法**:
```python
setup(self)           # 设置命令
execute(self, dt)     # 执行命令
update(self, dt)      # 更新命令
walk(self, dt)        # 行走逻辑
rotate(self, dt)      # 旋转逻辑
exit_command(self)    # 退出命令
force_quit_command(self)  # 强制退出命令

# 状态管理
get_command_id(self)      # 获取命令 ID
get_command_name(self)    # 获取命令名称
set_stauts(self, target_status)  # 设置命令状态
fetch_command_info(self)  # 获取命令信息以跟踪当前命令状态
```

**属性**:
```python
command_description   # 命令描述
command_usage        # 命令用法
parameters_info      # 参数信息
```

### Idle 命令

**构造函数签名**:
```python
Idle(self, *args, **kwargs)
```

**注意**: 同样需要 `character` 参数

**方法**: 与 GoTo 类似的基础方法

## 可用的命令类

从 `character_behavior` 模块导入的命令类：

```python
from omni.anim.people.scripts.character_behavior import (
    GoTo,           # 前往位置
    GoToObject,     # 前往对象
    GoToSection,    # 前往区域
    GoToBlendTemplate,  # 前往混合模板
    Idle,           # 待机
    LookAround,     # 环顾四周
    Sit,            # 坐下
    QueueCmd,       # 队列命令
    CustomCommand,  # 自定义命令
)
```

## 辅助类和管理器

```python
from omni.anim.people.scripts.character_behavior import (
    BehaviorScript,              # 行为脚本基类
    NavigationManager,           # 导航管理器
    GlobalCharacterPositionManager,  # 全局角色位置管理器
    GlobalQueueManager,          # 全局队列管理器
    CustomCommandManager,        # 自定义命令管理器
    InteractableObjectHelper,    # 可交互对象助手
)
```

## 枚举和数据类

```python
from omni.anim.people.scripts.character_behavior import (
    AgentEvent,      # 代理事件
    TaskStatus,      # 任务状态
    MetadataTag,     # 元数据标签
)
```

## 模板类

```python
from omni.anim.people.scripts.character_behavior import (
    CommandTemplateBase,     # 命令模板基类
    CustomCommandTemplate,   # 自定义命令模板
    TimingTemplate,          # 时间模板
    TimingToObjectTemplate,  # 到对象的时间模板
)
```

## 测试结果总结

### ✅ 成功的操作

1. **创建 CharacterBehavior 实例**
   ```python
   from omni.anim.people.scripts.character_behavior import CharacterBehavior
   from pxr import Sdf
   
   prim_path = Sdf.Path("/World/characters/worker_0")
   behavior = CharacterBehavior(prim_path)
   # ✅ 成功创建
   ```

2. **访问实例属性**
   ```python
   behavior.prim_path        # ✅ 可访问
   behavior.settings         # ✅ 可访问
   behavior.character_name   # ✅ 可访问
   ```

### ❌ 失败的操作

1. **导入 common 模块**
   ```python
   from omni.anim.people.scripts import common
   # ❌ ImportError: cannot import name 'common'
   ```

2. **convert_str_to_command 返回 None**
   ```python
   result = behavior.convert_str_to_command("goto 5 5 0")
   # ❌ 返回 None（可能需要运行时上下文）
   ```

3. **get_command 参数格式不明**
   ```python
   behavior.get_command(["goto", "5", "5", "0"])
   # ❌ too many values to unpack (expected 2)
   ```

4. **直接创建命令对象**
   ```python
   goto_cmd = GoTo()
   # ❌ TypeError: Command.__init__() missing 1 required positional argument: 'character'
   ```

## 待测试的功能

### 需要在运行时测试（main_example_with_character.py）

1. **inject_command 的正确格式**
   ```python
   # 待测试的格式
   behavior.inject_command([["goto", "5", "5", "0"]], executeImmediately=True)
   behavior.inject_command(["goto 5 5 0"], executeImmediately=True)
   ```

2. **命令执行流程**
   - 创建角色
   - 附加 CharacterBehavior
   - 调用 `on_play()` 初始化
   - 在主循环中调用 `on_update(current_time, delta_time)`
   - 使用 `inject_command()` 发送指令

3. **NavigationManager 的使用**
   - 如何与 CharacterBehavior 集成
   - 如何更新路径
   - 如何检查到达状态

4. **命令文件格式**
   - `read_commands_from_file()` 期望的文件格式
   - 命令字符串的语法

## 重要：角色 USD 文件路径要求

### ⚠️ 必须使用正确的 USD 文件路径

**关键发现**: 角色 USD 文件必须使用**根目录**下的文件，不能使用 `Props/Bones` 文件夹中的文件！

### 文件结构

NVIDIA 角色资产有三个层级：

```
/Characters/Reallusion/Worker/
├── Worker.usd                          ✅ 正确：根目录主文件（完整模型）
├── Props/
│   ├── Worker.usd                      ⚠️  Props 层级
│   └── Bones/
│       ├── Worker.usd                  ❌ 错误：骨骼文件（只有骨骼，无网格）
│       ├── Worker.StandingDiscussion_LookingDown_M.usd  ❌ 错误：带动画的骨骼
│       ├── Worker.TrafficGuard_M.usd   ❌ 错误：带动画的骨骼
│       └── ...
```

### 正确路径示例

```python
# ✅ 正确：使用根目录的 USD 文件
worker_usd = "/home/ubuntu/isaacsim_assets/Assets/Isaac/4.5/NVIDIA/Assets/Characters/Reallusion/Worker/Worker.usd"
debra_usd = "/home/ubuntu/isaacsim_assets/Assets/Isaac/4.5/NVIDIA/Assets/Characters/Reallusion/Debra/Debra.usd"
orc_usd = "/home/ubuntu/isaacsim_assets/Assets/Isaac/4.5/NVIDIA/Assets/Characters/Reallusion/Orc/Orc.usd"
```

### 错误路径示例

```python
# ❌ 错误：Props/Bones 文件夹中的文件（只有骨骼数据，没有网格）
worker_bones = "/home/ubuntu/isaacsim_assets/Assets/Isaac/4.5/NVIDIA/Assets/Characters/Reallusion/Worker/Props/Bones/Worker.usd"

# ❌ 错误：带动画的骨骼文件
worker_anim = "/home/ubuntu/isaacsim_assets/Assets/Isaac/4.5/NVIDIA/Assets/Characters/Reallusion/Worker/Props/Bones/Worker.StandingDiscussion_LookingDown_M.usd"

# ❌ 错误：AnimGraph 文件夹中的文件
debra_animgraph = "/home/ubuntu/isaacsim_assets/Assets/Isaac/4.5/NVIDIA/Assets/AnimGraph/105.0/Characters/Reallusion/Debra/Props/Bones/Debra.usd"
```

### 为什么必须使用根目录文件？

1. **完整模型**: 根目录的 USD 文件包含完整的角色模型（网格 + 骨骼 + 材质）
2. **骨骼文件**: `Props/Bones` 文件夹中的文件只包含骨骼绑定数据，没有网格，导入后是空的
3. **动画文件**: 带动画名称的文件（如 `Worker.TrafficGuard_M.usd`）是特定姿态的骨骼文件，用于 Animation Graph 系统

### 验证方法

可以通过以下命令查看文件结构：

```bash
# 查看 Worker 文件夹结构
find /home/ubuntu/isaacsim_assets/Assets/Isaac/4.5/NVIDIA/Assets/Characters/Reallusion/Worker -name "*.usd" -type f

# 输出示例：
# /path/to/Worker/Worker.usd                    ← 使用这个！
# /path/to/Worker/Props/Worker.usd
# /path/to/Worker/Props/Bones/Worker.usd        ← 不要用这个
# /path/to/Worker/Props/Bones/Worker.*.usd      ← 不要用这些
```

### 在代码中的应用

```python
from physics_engine.isaacsim_utils import define_prim
from physics_engine.pxr_utils import UsdGeom, Gf

# ✅ 正确的创建方式
character_usd = "/home/ubuntu/isaacsim_assets/Assets/Isaac/4.5/NVIDIA/Assets/Characters/Reallusion/Worker/Worker.usd"
prim_path = "/World/characters/worker_0"

# 创建 prim 并加载 USD
prim = define_prim(prim_path, "Xform")
prim.GetReferences().AddReference(character_usd)

# 设置缩放（USD 文件使用厘米，世界使用米）
xformable = UsdGeom.Xformable(prim)
xformable.AddScaleOp().Set(Gf.Vec3f(0.01, 0.01, 0.01))

# 设置位置和旋转
xformable.AddTranslateOp().Set(Gf.Vec3d(5, 5, 0))
xformable.AddOrientOp().Set(Gf.Quatf(1, 0, 0, 0))  # wxyz
```

## 推荐的使用流程

基于测试结果，推荐的使用流程：

```python
from omni.anim.people.scripts.character_behavior import CharacterBehavior
from physics_engine.isaacsim_utils import define_prim
from physics_engine.pxr_utils import UsdGeom, Gf
from pxr import Sdf

# 1. 创建角色（⚠️ 必须使用根目录的 USD 文件）
character_usd = "/home/ubuntu/isaacsim_assets/Assets/Isaac/4.5/NVIDIA/Assets/Characters/Reallusion/Worker/Worker.usd"
prim_path = "/World/characters/worker_0"

# 2. 在 USD stage 中创建角色 prim
prim = define_prim(prim_path, "Xform")
prim.GetReferences().AddReference(character_usd)

# 设置变换
xformable = UsdGeom.Xformable(prim)
xformable.AddScaleOp().Set(Gf.Vec3f(0.01, 0.01, 0.01))  # cm to m
xformable.AddTranslateOp().Set(Gf.Vec3d(5, 5, 0))
xformable.AddOrientOp().Set(Gf.Quatf(1, 0, 0, 0))  # wxyz

# 3. 创建 CharacterBehavior 实例
behavior = CharacterBehavior(Sdf.Path(prim_path))

# 4. 在运行时初始化
# behavior.on_play() 会被自动调用

# 5. 在主循环中更新
while simulation_app.is_running():
    world.tick()
    current_time = world.get_simulation_time()
    delta_time = world.get_physics_dt()
    
    # 更新行为
    behavior.on_update(current_time, delta_time)
    
    # 发送命令（待测试正确格式）
    if some_condition:
        behavior.inject_command([["goto", "10", "10", "0"]], executeImmediately=True)
```

## 下一步

1. 在 `main_example_with_character.py` 中集成 `CharacterBehavior`
2. 测试 `inject_command` 的正确命令格式
3. 测试命令执行和状态转换
4. 验证 NavigationManager 的工作方式
5. 创建完整的使用示例

## 参考资料

- Isaac Sim 4.5 Documentation
- Extension: omni.anim.people-0.6.7+106.5.0
- 位置: `/home/ubuntu/anaconda3/envs/env_isaaclab/lib/python3.10/site-packages/isaacsim/extscache/omni.anim.people-0.6.7+106.5.0/`

---

**创建日期**: 2025-01-28  
**测试环境**: Isaac Sim 4.5.0, Python 3.10  
**状态**: API 探索阶段，待运行时测试
