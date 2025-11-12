# 基于状态机的机器人技能系统设计

## 概述

本文提出了一种基于有限状态机（Finite State Machine, FSM）的通用机器人技能管理架构。该架构将复杂的机器人任务分解为多个离散状态，通过明确的状态转换逻辑，实现了任务的模块化管理。该设计已成功应用于导航（navigate_to）、起飞（take_off）、拍照（take_photo）、操作（manipulation）等多种机器人技能。

---

## 1. 设计动机

### 1.1 机器人技能的复杂性

现代机器人需要执行多种复杂任务，这些任务通常具有以下特点：

1. **多步骤性**：任务需要按顺序执行多个步骤
   - 例如：导航 = 初始化 → 路径规划 → 轨迹生成 → 执行
   - 例如：拍照 = 移动到位置 → 调整姿态 → 对焦 → 拍摄

2. **异步性**：某些步骤需要等待外部结果
   - 路径规划可能需要数秒
   - 图像处理需要等待算法完成
   - 机械臂运动需要等待到达目标

3. **不确定性**：执行过程中可能遇到各种情况
   - 规划失败
   - 超时
   - 碰撞检测
   - 传感器故障

4. **可恢复性**：需要支持错误处理和重试
   - 重新规划路径
   - 重新尝试抓取
   - 调整相机参数

### 1.2 传统方法的局限性

**方法 1：单体式实现**
```python
def navigate_to(robot, goal):
    # 所有逻辑写在一个函数中
    plan_path()
    generate_trajectory()
    execute()
    # 难以维护，难以处理异步操作
```

**问题**：
- 代码耦合度高
- 难以处理异步操作
- 错误处理复杂
- 无法暂停和恢复

**方法 2：回调地狱**
```python
def navigate_to(robot, goal):
    plan_path(callback=lambda path: 
        generate_trajectory(path, callback=lambda traj:
            execute(traj, callback=lambda result:
                handle_result(result))))
```

**问题**：
- 代码可读性差
- 调试困难
- 状态管理混乱

### 1.3 状态机的优势

状态机提供了一种清晰的方式来管理复杂任务：

1. **清晰的状态分离**：每个状态职责单一
2. **明确的转换逻辑**：状态之间的转换条件清晰
3. **易于调试**：可以追踪状态转换历史
4. **支持异步**：自然支持异步操作
5. **可视化**：可以用状态图直观展示

---

## 2. 状态机理论基础

### 2.1 有限状态机定义

有限状态机（FSM）是一种数学模型，由以下五元组定义：

```
M = (S, Σ, δ, s₀, F)
```

**组成要素**：
- **S**：有限状态集合
- **Σ**：输入符号集合（事件/条件）
- **δ: S × Σ → S**：状态转换函数
- **s₀ ∈ S**：初始状态
- **F ⊆ S**：终止状态集合

### 2.2 状态机的特性

**1. 确定性**
- 给定当前状态和输入，下一个状态是唯一确定的
- 便于预测和调试

**2. 有限性**
- 状态数量有限
- 避免状态爆炸

**3. 可组合性**
- 可以组合多个状态机
- 支持层次化设计

### 2.3 Navigate To 技能的形式化示例

为了更好地理解状态机的应用，我们以 navigate_to 技能为例进行完整的形式化描述：

**定义**：`M_navigate = (S, Σ, δ, s₀, F)`

**状态集合 S**：
```
S = {INITIALIZING, EXECUTING, COMPLETED, FAILED}
```

**输入符号集合 Σ**：
```
Σ = {request_received, ros_unavailable, planning_succeeded, 
     planning_failed, goal_reached, execution_timeout}
```

**状态转换函数 δ**：
```
δ(None, request_received) = INITIALIZING
δ(INITIALIZING, ros_unavailable) = FAILED
δ(INITIALIZING, planning_succeeded) = EXECUTING
δ(INITIALIZING, planning_failed) = FAILED
δ(EXECUTING, goal_reached) = COMPLETED
δ(EXECUTING, execution_timeout) = FAILED
```

**初始状态 s₀**：
```
s₀ = None (隐式)
```

**终止状态集合 F**：
```
F = {COMPLETED, FAILED}
```

**执行轨迹示例**：
```
成功轨迹：
None --[request_received]--> INITIALIZING --[planning_succeeded]--> 
EXECUTING --[goal_reached]--> COMPLETED

失败轨迹 1（ROS 不可用）：
None --[request_received]--> INITIALIZING --[ros_unavailable]--> FAILED

失败轨迹 2（超时）：
None --[request_received]--> INITIALIZING --[planning_succeeded]--> 
EXECUTING --[execution_timeout]--> FAILED
```

**状态不变式（Invariants）**：
```
1. ∀s ∈ S, s ∉ F ⇒ ∃σ ∈ Σ, δ(s, σ) ∈ F
   （所有非终止状态最终都能到达终止状态）

2. ∀s ∈ F, ∀σ ∈ Σ, δ(s, σ) = s
   （终止状态不再转换）

3. EXECUTING 状态下，MPC 控制器并行运行
   （状态机与控制器解耦）
```

---

## 3. 通用状态机架构设计

### 3.1 核心设计原则

**原则 1：状态职责单一**
- 每个状态只负责一个明确的任务
- 避免在单个状态中处理多个逻辑

**原则 2：转换条件明确**
- 状态转换的条件必须清晰可判断
- 避免模糊的转换条件

**原则 3：数据与状态分离**
- 状态只表示"在做什么"
- 数据存储在独立的数据结构中

**原则 4：支持异步操作**
- 状态机不阻塞主循环
- 异步操作通过状态轮询检查

### 3.2 通用状态模式

大多数机器人技能可以抽象为以下状态模式：

```
通用状态流程：
IDLE → INITIALIZING → PREPARING → EXECUTING → FINALIZING → SUCCEEDED/FAILED
```

**状态语义**：

1. **IDLE（空闲）**
   - 等待任务请求
   - 系统处于待命状态

2. **INITIALIZING（初始化）**
   - 解析输入参数
   - 验证参数有效性
   - 获取必要的初始信息

3. **PREPARING（准备）**
   - 执行前置任务
   - 可能包含多个子步骤
   - 例如：路径规划、资源分配

4. **EXECUTING（执行）**
   - 执行主要任务
   - 持续监控执行状态
   - 发布实时反馈

5. **FINALIZING（收尾）**
   - 清理资源
   - 保存结果
   - 恢复初始状态

6. **SUCCEEDED（成功）**
   - 任务成功完成
   - 终止状态

7. **FAILED（失败）**
   - 任务失败
   - 记录失败原因
   - 终止状态

### 3.3 状态转换规则

**基本转换规则**：
```
IDLE ──[request]──→ INITIALIZING
INITIALIZING ──[success]──→ PREPARING
INITIALIZING ──[failed]──→ FAILED
PREPARING ──[success]──→ EXECUTING
PREPARING ──[failed]──→ FAILED
EXECUTING ──[completed]──→ FINALIZING
EXECUTING ──[failed]──→ FAILED
FINALIZING ──[success]──→ SUCCEEDED
```

**扩展转换规则**（支持重试和重规划）：
```
EXECUTING ──[need_replan]──→ PREPARING
FAILED ──[retry]──→ INITIALIZING
```

---

## 4. 技能实现示例（基于实际代码）

### 4.1 导航技能（Navigate To）

**代码位置**：`application/skills/base/navigation/navigate_to.py`

**任务描述**：机器人从当前位置导航到目标位置

**形式化定义**：

根据有限状态机的五元组定义 `M = (S, Σ, δ, s₀, F)`，navigate_to 技能可以形式化为：

**1. 状态集合 S**：
```
S = {INITIALIZING, EXECUTING, COMPLETED, FAILED}
```

**2. 输入符号集合 Σ**（事件/条件）：
```
Σ = {
    request_received,           # 收到导航请求
    ros_unavailable,           # ROS 不可用
    planning_request_sent,     # 规划请求已发送
    planning_accepted,         # 规划请求被接受
    planning_rejected,         # 规划请求被拒绝
    planning_succeeded,        # 路径规划成功
    planning_failed,           # 路径规划失败
    planning_timeout,          # 规划超时
    goal_reached,              # 到达目标
    execution_timeout,         # 执行超时
    user_cancel               # 用户取消
}
```

**3. 状态转换函数 δ: S × Σ → S**：

| 当前状态 | 输入符号 | 下一状态 | 条件 |
|---------|---------|---------|------|
| - | request_received | INITIALIZING | 收到导航请求 |
| INITIALIZING | ros_unavailable | FAILED | ROS 不可用 |
| INITIALIZING | planning_rejected | FAILED | 规划请求被拒绝 |
| INITIALIZING | planning_timeout | FAILED | 规划请求超时（3秒） |
| INITIALIZING | planning_succeeded | EXECUTING | 路径规划成功 |
| INITIALIZING | planning_failed | FAILED | 路径规划失败（15秒） |
| EXECUTING | goal_reached | COMPLETED | `move_event.is_set()` |
| EXECUTING | execution_timeout | FAILED | 执行时间 > 120秒 |
| EXECUTING | user_cancel | FAILED | 用户取消任务 |

**4. 初始状态 s₀**：
```
s₀ = INITIALIZING
```
（当收到 `request_received` 事件时，从隐式的 None 状态进入 INITIALIZING）

**5. 终止状态集合 F**：
```
F = {COMPLETED, FAILED}
```

**状态转换图**：
```
                [request_received]
    (None) ──────────────────────→ INITIALIZING
                                        │
                    ┌───────────────────┼───────────────────┐
                    │                   │                   │
        [ros_unavailable]    [planning_succeeded]  [planning_failed/
         [planning_rejected]                        planning_timeout]
         [planning_timeout]                                 │
                    │                   │                   │
                    ↓                   ↓                   ↓
                 FAILED ←──────── EXECUTING ──────────→ COMPLETED
                                        │
                                        │ [goal_reached]
                                        │
                            [execution_timeout/user_cancel]
                                        │
                                        ↓
                                     FAILED
```

**实际状态设计**：
```
None/INITIALIZING → EXECUTING → COMPLETED/FAILED
```

**状态实现分析**：

**状态 1: INITIALIZING（初始化）**

**形式化描述**：

输入参数：
- p_goal ∈ ℝ³ 为目标位置
- q_goal ∈ SO(3) 为目标姿态（四元数表示）
- p_start ∈ ℝ³ 为起始位置（可选）

状态转换规则：
- ROS不可用 / 规划被拒绝 → FAILED
- 规划请求超时（3秒） → FAILED  
- 路径规划成功 → EXECUTING
- 规划失败且超过最大时间（15秒） → FAILED
- 其他情况 → 保持INITIALIZING（继续等待）

**状态职责**：
1. **系统检查**：验证 ROS 通信层可用性
2. **参数解析**：提取并验证目标位置 p_goal 和姿态 q_goal
3. **请求构造**：创建路径规划请求消息 M_plan = (p_start, p_goal, q_goal, map)
4. **异步发送**：通过 ROS Action 接口发送规划请求，获取 Future 对象 F_plan
5. **结果轮询**：每帧检查 F_plan 的完成状态

**异步操作机制**：

该状态采用异步轮询模式：
- 首次进入时发送规划请求，存储 Future 对象
- 后续每帧检查 Future 状态：
  - 若 F_plan.done() = false，保持 INITIALIZING 状态
  - 若 F_plan.done() = true，根据结果转换状态

**数据流**：
用户输入 → 解析参数 → 构造规划请求 → 异步发送 → 轮询结果

**状态 2: EXECUTING（执行）**

**形式化描述**：

机器人当前位置为 p(t) ∈ ℝ³，目标位置为 p_goal ∈ ℝ³。

状态转换规则：
- 到达目标（由MPC控制器的事件信号确定） → COMPLETED
- 执行超时（120秒） → FAILED
- 用户取消 → FAILED
- 其他情况 → 保持EXECUTING（继续执行）

**状态职责**：
1. **状态监控**：持续查询 MPC 控制器的执行状态
2. **到达检测**：通过事件信号判断是否到达目标
3. **超时检测**：检查执行时间是否超过阈值
4. **反馈生成**：计算并发布实时反馈信息（进度、距离、速度）

**并行控制架构**：

该状态采用分层控制模式，状态机与 MPC 控制器并行运行：

- **状态机层**：负责监控和反馈，频率约10 Hz
- **控制层**：MPC控制器根据参考轨迹计算控制命令，频率60 Hz
- **执行层**：应用控制命令更新机器人状态

**关键设计原则**：

在 EXECUTING 状态下，遵循以下设计原则：

1. **控制解耦**：
   - 状态机不直接设置控制命令
   - MPC 控制器独立计算并应用控制

2. **事件驱动**：
   - 到达检测通过事件信号实现
   - 避免状态机频繁查询位置

3. **时间约束**：
   - 执行时间有上界（120秒）
   - 超时后必然进入终止状态

**进度计算**：

实时反馈中的进度百分比计算方式：
- 初始进度为 50%（规划完成）
- 随执行时间线性增长
- 最大不超过 95%（为到达检测预留空间）

**状态 3: COMPLETED/FAILED（终止状态）**

**形式化描述**：

终止状态集合 F = {COMPLETED, FAILED}，一旦进入终止状态，状态机停止演化。

**COMPLETED 状态**：
- **前置条件**：机器人到达目标位置
- **输出**：成功反馈（status: completed, progress: 100%）
- **后置条件**：机器人速度设为0（由MPC控制器设置）

**FAILED 状态**：
- **前置条件**：ROS不可用 / 规划失败 / 执行超时 / 用户取消
- **输出**：失败反馈（status: failed, reason: 失败原因）
- **后置条件**：记录失败原因到错误日志

**可达性分析**：

从初始状态到终止状态的所有可能路径：

- **成功路径**：收到请求 → INITIALIZING → 规划成功 → EXECUTING → 到达目标 → COMPLETED
- **失败路径1**：收到请求 → INITIALIZING → ROS不可用 → FAILED
- **失败路径2**：收到请求 → INITIALIZING → 规划失败 → FAILED
- **失败路径3**：收到请求 → INITIALIZING → 规划成功 → EXECUTING → 超时 → FAILED

**设计特点总结**：

1. **状态简化**：相比通用模式，navigate_to 只使用 3 个主要状态
   - 合并了 PREPARING 阶段到 INITIALIZING
   - 省略了 FINALIZING 阶段（清理操作在终止状态中完成）

2. **异步机制**：采用 Future 对象实现非阻塞路径规划
   - 每帧仅检查 Future 状态，计算开销小
   - 空间复杂度与路径点数量成正比

3. **分层控制**：状态机与 MPC 控制器解耦
   - 状态机频率：~10 Hz（监控层）
   - MPC 频率：60 Hz（控制层）
   - 通信方式：事件信号（单向，控制器 → 状态机）

4. **事件驱动**：使用事件信号而非轮询检测到达
   - 减少计算开销
   - 提高响应速度
   - 避免状态机频繁查询机器人位置

---

### 4.2 起飞技能（Take Off）

**代码位置**：`application/skills/drone/take_off.py`

**任务描述**：无人机从地面起飞到指定高度

**实际状态设计**：
```
None/INITIALIZING → EXECUTING → COMPLETED/FAILED
```

**状态实现分析**：

**状态 1: INITIALIZING（初始化）**
```python
if state in [None, "INITIALIZING"]:
    altitude = kwargs.get("altitude", 1.0)
    if altitude <= 0:
        skill_manager.set_skill_state(skill_name, "FAILED")
        return skill_manager.form_feedback("failed", "Invalid altitude")
    
    skill_manager.set_skill_data(skill_name, "target_altitude", altitude)
    skill_manager.set_skill_data(skill_name, "start_time", robot.sim_time)
    skill_manager.set_skill_state(skill_name, "EXECUTING")
```

**职责**：
1. 解析目标高度参数
2. 验证参数有效性（高度 > 0）
3. 存储目标高度和开始时间
4. 立即转换到 EXECUTING 状态

**状态 2: EXECUTING（执行）**
```python
elif state == "EXECUTING":
    target_alt = skill_manager.get_skill_data(skill_name, "target_altitude")
    current_alt = robot.position[2].item()
    
    # 检查是否到达
    if current_alt >= target_alt - 0.1:
        control = RobotControl()
        control.linear_velocity = [0.0, 0.0, 0.0]
        robot.apply_control(control)
        skill_manager.set_skill_state(skill_name, "COMPLETED")
        return skill_manager.form_feedback("completed", "Reached altitude", 100)
    
    # 检查超时
    if robot.sim_time - start_time > 30.0:
        skill_manager.set_skill_state(skill_name, "FAILED")
        return skill_manager.form_feedback("failed", "Timeout")
    
    # 应用上升速度
    control = RobotControl()
    control.linear_velocity = [0.0, 0.0, 0.5]
    robot.apply_control(control)
    
    progress = min(int((current_alt / target_alt) * 100), 95)
    return skill_manager.form_feedback("processing", f"{current_alt:.1f}m", progress)
```

**职责**：
1. 获取当前高度
2. 检查是否到达目标（阈值 0.1m）
3. 检查超时（30秒）
4. 应用上升速度（0.5 m/s）
5. 计算并返回进度

**设计特点**：
1. **极简状态机**：只有 2 个主要状态
2. **直接控制**：状态机直接设置机器人速度
3. **实时反馈**：每帧计算进度百分比
4. **到达检测**：使用阈值判断（0.1m）
5. **超时保护**：30秒超时

---

### 4.3 拍照技能（Take Photo）

**代码位置**：`application/skills/base/take_photo.py`

**任务描述**：使用机器人相机拍摄照片

**实际状态设计**：
```
None/INITIALIZING → EXECUTING → COMPLETED/FAILED
```

**状态实现分析**：

**状态 1: INITIALIZING（初始化）**
```python
def _init_take_photo(robot, skill_manager, skill_name, kwargs):
    camera_name = kwargs.get("camera_name", "default")
    save_to_file = kwargs.get("save_to_file", "")
    
    # 检查相机可用性
    if not hasattr(robot, "camera_dict") or camera_name not in robot.camera_dict:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = f"Camera '{camera_name}' not available"
        return skill_manager.form_feedback("failed", f"Camera '{camera_name}' not found")
    
    # 存储参数
    skill_manager.set_skill_data(skill_name, "camera_name", camera_name)
    skill_manager.set_skill_data(skill_name, "save_to_file", save_to_file)
    skill_manager.set_skill_state(skill_name, "EXECUTING")
    return skill_manager.form_feedback("processing", "Preparing camera", 20)
```

**职责**：
1. 解析相机名称和保存路径
2. 检查相机是否存在
3. 存储参数到 skill_data
4. 转换到 EXECUTING 状态

**状态 2: EXECUTING（执行）**
```python
def _handle_executing(robot, skill_manager, skill_name):
    camera_name = skill_manager.get_skill_data(skill_name, "camera_name")
    save_to_file = skill_manager.get_skill_data(skill_name, "save_to_file")
    
    # 获取相机
    camera = robot.camera_dict[camera_name]
    
    # 拍摄照片
    rgb = camera.get_rgb()
    
    if rgb is None:
        skill_manager.set_skill_state(skill_name, "FAILED")
        return skill_manager.form_feedback("failed", "Capture failed")
    
    # 存储照片数据
    photo_data = {"rgb_image": rgb, "camera_name": camera_name}
    skill_manager.set_skill_data(skill_name, "photo_data", photo_data)
    
    # 可选：保存到文件
    feedback_msg = "Photo captured"
    if save_to_file:
        save_result = camera.save_rgb_to_file(rgb=rgb, file_path=save_to_file)
        if save_result:
            feedback_msg = f"Saved to {save_to_file}"
    
    skill_manager.set_skill_state(skill_name, "COMPLETED")
    return skill_manager.form_feedback("completed", feedback_msg, 100)
```

**职责**：
1. 获取相机对象
2. 调用 `camera.get_rgb()` 拍摄照片
3. 验证照片数据
4. 存储照片到 skill_data
5. 可选：保存到文件
6. 转换到 COMPLETED 状态

**设计特点**：
1. **同步执行**：拍照是同步操作，一帧完成
2. **数据存储**：照片数据存储在 skill_data 中
3. **可选保存**：支持保存到文件
4. **错误处理**：检查照片是否为 None

---

### 4.4 抓取技能（Pick Up）

**代码位置**：`application/skills/manipulation/pick_up.py`

**任务描述**：机械臂抓取目标物体

**实际状态设计**：
```
None/INITIALIZING → CHECKING → ATTACHING → COMPLETED/FAILED
```

**状态实现分析**：

**状态 1: INITIALIZING（初始化）**
```python
def _init_pick_up(robot, skill_manager, skill_name, kwargs):
    from simulation.control.command import GraspControl, ControlAction
    
    # 解析参数
    robot_hand_prim_path = kwargs.get("robot_hand_prim_path")
    object_prim_path = kwargs.get("object_prim_path")
    distance_threshold = kwargs.get("distance_threshold", 2.0)
    
    # 创建 GraspControl 对象（纯数据）
    grasp_control = GraspControl(
        hand_prim_path=robot_hand_prim_path,
        object_prim_path=object_prim_path,
        action=ControlAction.CHECK_DISTANCE,
        distance_threshold=distance_threshold,
        local_pos_hand=local_pos_hand,
        local_pos_object=local_pos_object,
        axis=axis,
    )
    
    # 应用控制（Robot 将在 on_physics_step 中执行）
    robot.apply_manipulation_control(grasp_control)
    
    # 保存控制对象
    skill_manager.set_skill_data(skill_name, "grasp_control", grasp_control)
    skill_manager.set_skill_state(skill_name, "CHECKING")
    return skill_manager.form_feedback("processing", "Checking distance", 20)
```

**职责**：
1. 解析手部和物体的 prim 路径
2. 创建 GraspControl 对象（纯数据，不调用 Isaac Sim API）
3. 设置动作为 CHECK_DISTANCE
4. 应用控制到 Robot
5. 转换到 CHECKING 状态

**状态 2: CHECKING（检查距离）**
```python
def _handle_checking(robot, skill_manager, skill_name):
    # 查询 Robot 层的执行结果（异步）
    result = robot.get_manipulation_result()
    
    if result is None:
        # 仍在处理，等待下一帧
        return skill_manager.form_feedback("processing", "Checking distance", 30)
    
    if result["success"]:
        # 距离检查通过，准备抓取
        from simulation.control.command import ControlAction
        
        grasp_control = skill_manager.get_skill_data(skill_name, "grasp_control")
        grasp_control.action = ControlAction.ATTACH
        
        # 应用抓取控制
        robot.apply_manipulation_control(grasp_control)
        
        skill_manager.set_skill_state(skill_name, "ATTACHING")
        return skill_manager.form_feedback("processing", "Attaching object", 60)
    else:
        # 距离检查失败
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = result["message"]
        return skill_manager.form_feedback("failed", result["message"])
```

**职责**：
1. 查询 Robot 层的执行结果
2. 如果结果为 None，继续等待
3. 如果成功，修改控制动作为 ATTACH
4. 应用抓取控制
5. 转换到 ATTACHING 状态

**状态 3: ATTACHING（附加物体）**
```python
def _handle_attaching(robot, skill_manager, skill_name):
    # 查询附加结果
    result = robot.get_manipulation_result()
    
    if result is None:
        return skill_manager.form_feedback("processing", "Attaching object", 70)
    
    if result["success"]:
        # 附加成功
        joint_path = result["data"].get("joint_path")
        skill_manager.set_skill_data(skill_name, "joint_path", joint_path)
        
        skill_manager.set_skill_state(skill_name, "COMPLETED")
        return skill_manager.form_feedback("completed", "Object picked up", 100)
    else:
        # 附加失败
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = result["message"]
        return skill_manager.form_feedback("failed", result["message"])
```

**职责**：
1. 查询附加操作的结果
2. 如果成功，存储 joint_path
3. 转换到 COMPLETED 状态

**设计特点**：
1. **分层架构**：Application 层创建 Control 对象，Robot 层执行
2. **异步执行**：通过轮询 `get_manipulation_result()` 获取结果
3. **多阶段操作**：CHECK_DISTANCE → ATTACH
4. **纯数据对象**：GraspControl 不包含 Isaac Sim API 调用
5. **结果查询**：Application 层不直接调用 Isaac Sim API

---

### 4.5 技能对比总结

| 技能 | 状态数量 | 异步操作 | 控制方式 | 复杂度 |
|------|---------|---------|---------|--------|
| navigate_to | 3 | ✓ (ROS Action) | MPC 并行控制 | 高 |
| take_off | 2 | ✗ | 直接设置速度 | 低 |
| take_photo | 2 | ✗ | 同步拍照 | 低 |
| pick_up | 3 | ✓ (异步查询) | Control 对象 | 中 |

**观察**：
1. **状态数量简化**：实际实现比理论设计更简洁（2-3个状态）
2. **异步模式多样**：ROS Action Future、异步查询结果
3. **控制方式不同**：并行控制、直接控制、Control 对象
4. **分层清晰**：Application 层不直接调用 Isaac Sim API

---

## 5. 核心技术特性

### 5.1 异步操作支持

**问题**：某些操作（如路径规划、图像处理）需要较长时间，如何避免阻塞？

**解决方案**：异步状态机模式

**实现机制**：
```python
# 状态机每帧被调用
def execute_skill(skill_name, **kwargs):
    state = get_current_state(skill_name)
    
    if state == 'PLANNING':
        # 第一次进入：发送异步请求
        if not has_planning_future():
            future = send_planning_request_async()
            store_future(future)
            return {'status': 'processing', 'message': 'Planning...'}
        
        # 后续调用：检查是否完成
        future = get_future()
        if not future.done():
            return {'status': 'processing', 'message': 'Planning...'}
        
        # 完成：提取结果并转换状态
        result = future.result()
        if result.success:
            store_result(result)
            transition_to('EXECUTING')
            return {'status': 'processing', 'message': 'Planning completed'}
        else:
            transition_to('FAILED')
            return {'status': 'failed', 'message': 'Planning failed'}
```

**优势**：
- 不阻塞主循环
- 支持实时反馈
- 可以随时取消任务
- 系统保持响应

**应用场景**：
- 路径规划（navigate_to）
- 图像处理（take_photo）
- 运动规划（manipulation）
- 轨迹生成（所有运动任务）

---

### 5.2 状态与数据分离

**设计原则**：状态只表示"在做什么"，数据存储在独立结构中

**数据结构**：
```python
# 状态存储
skill_states = {
    'navigate_to': 'EXECUTING',
    'take_photo': 'FOCUSING',
    'manipulation': 'GRASPING'
}

# 数据存储
skill_data = {
    'navigate_to': {
        'start_pos': [0, 0, 0],
        'target_pos': [10, 20, 0],
        'path': [...],
        'trajectory': [...],
        'start_time': 1234567890.0
    },
    'take_photo': {
        'camera_pos': [5, 5, 2],
        'target_object': 'car',
        'focus_distance': 3.5,
        'image_data': None
    }
}
```

**优势**：
1. **状态可复用**：多个技能可以使用相同的状态名
2. **数据持久化**：可以保存和恢复任务
3. **调试友好**：可以查看任意时刻的数据
4. **支持并发**：多个技能可以同时执行

---

### 5.3 错误处理与重试

**错误类型**：
1. **可恢复错误**：可以通过重试解决
   - 路径规划失败 → 调整参数重试
   - 抓取失败 → 重新检测目标
   - 对焦失败 → 调整相机参数

2. **不可恢复错误**：必须终止任务
   - 目标不可达
   - 硬件故障
   - 超时

**重试机制**：
```python
# 在状态数据中记录重试次数
skill_data['navigate_to']['retry_count'] = 0
skill_data['navigate_to']['max_retries'] = 3

if state == 'PLANNING':
    if planning_failed():
        retry_count = skill_data['navigate_to']['retry_count']
        max_retries = skill_data['navigate_to']['max_retries']
        
        if retry_count < max_retries:
            # 重试
            skill_data['navigate_to']['retry_count'] += 1
            transition_to('INITIALIZING')  # 回到初始化
            return {'status': 'processing', 'message': f'Retrying ({retry_count+1}/{max_retries})'}
        else:
            # 超过最大重试次数
            transition_to('FAILED')
            return {'status': 'failed', 'message': 'Max retries exceeded'}
```

**动态重规划**：
```python
if state == 'EXECUTING':
    if collision_detected():
        # 不增加重试次数，直接重新规划
        current_pos = robot.get_world_pose()[0]
        skill_data['navigate_to']['start_pos'] = current_pos
        transition_to('PLANNING')
        return {'status': 'processing', 'message': 'Replanning due to collision'}
```

---

### 5.4 实时反馈机制

**反馈类型**：
1. **状态反馈**：当前处于哪个状态
2. **进度反馈**：任务完成百分比
3. **详细信息**：具体的执行信息

**反馈结构**：
```python
{
    'status': 'processing',      # processing/succeeded/failed
    'state': 'EXECUTING',        # 当前状态
    'progress': 65,              # 进度百分比 (0-100)
    'message': 'Navigating...',  # 人类可读的消息
    'details': {                 # 详细信息
        'distance_to_goal': 5.2,
        'current_velocity': 0.8,
        'estimated_time': 6.5
    }
}
```

**实现示例**：
```python
if state == 'EXECUTING':
    # 计算进度
    start_pos = skill_data['navigate_to']['start_pos']
    target_pos = skill_data['navigate_to']['target_pos']
    current_pos = robot.get_world_pose()[0]
    
    total_distance = distance(start_pos, target_pos)
    traveled_distance = distance(start_pos, current_pos)
    progress = int((traveled_distance / total_distance) * 100)
    
    # 计算剩余距离
    remaining_distance = distance(current_pos, target_pos)
    
    # 估计剩余时间
    current_velocity = robot.get_velocity().norm()
    estimated_time = remaining_distance / current_velocity if current_velocity > 0 else 0
    
    return {
        'status': 'processing',
        'state': 'EXECUTING',
        'progress': progress,
        'message': f'Navigating to goal ({progress}%)',
        'details': {
            'distance_to_goal': remaining_distance,
            'current_velocity': current_velocity,
            'estimated_time': estimated_time
        }
    }
```

**应用**：
- ROS Action 反馈
- UI 进度条
- 日志记录
- 性能分析

---

### 5.5 并行控制集成

**问题**：状态机负责高层决策，但底层控制需要高频运行（如 MPC）

**解决方案**：分层控制架构

**架构设计**：
```
┌─────────────────────────────────────────────────────┐
│         状态机层（低频，~10 Hz）                      │
│  • 任务调度和状态转换                                 │
│  • 进度监控和错误处理                                 │
│  • 设置参考目标                                       │
└─────────────────────────────────────────────────────┘
                    ↓ (设置目标)
┌─────────────────────────────────────────────────────┐
│         控制层（高频，60 Hz）                         │
│  • MPC 控制器（导航）                                 │
│  • PID 控制器（起飞/悬停）                            │
│  • 轨迹跟踪控制器（机械臂）                           │
└─────────────────────────────────────────────────────┘
                    ↓ (控制命令)
┌─────────────────────────────────────────────────────┐
│         执行层（60 Hz）                               │
│  • 应用控制命令                                       │
│  • 更新机器人状态                                     │
└─────────────────────────────────────────────────────┘
```

**关键机制**：
1. **状态机设置目标**：
   - navigate_to: 设置参考轨迹到 MPC
   - take_off: 设置目标高度到高度控制器
   - manipulation: 设置目标位姿到轨迹跟踪器

2. **控制器独立运行**：
   - 订阅仿真时钟，自动触发
   - 不依赖状态机的调用频率
   - 直接设置机器人速度/力矩

3. **状态机监控**：
   - 检查是否到达目标
   - 检查是否超时
   - 检查是否需要重规划

**优势**：
- 状态机和控制器解耦
- 控制器高频运行，保证性能
- 状态机只负责监控，不参与控制
- 易于扩展和维护

---

## 6. 实现架构

### 6.1 技能管理器（SkillManager）

**职责**：管理所有技能的注册、执行和状态

**核心组件**：
```python
class SkillManager:
    # 类变量：全局技能注册表
    _global_skills: Dict[str, Callable] = {}
    
    def __init__(self, robot):
        self.robot = robot
        self.skills = {}              # 实例技能
        self.skill_states = {}        # 状态存储
        self.skill_data = {}          # 数据存储
```

**技能注册**（装饰器模式）：
```python
@SkillManager.register()
def navigate_to(robot, goal_pos, **kwargs):
    """导航技能实现"""
    skill_manager = kwargs['skill_manager']
    state = skill_manager.get_skill_state('navigate_to')
    
    if state is None:
        # 首次调用，初始化
        state = 'INITIALIZING'
        skill_manager.set_skill_state('navigate_to', state)
    
    # 根据状态执行相应逻辑
    if state == 'INITIALIZING':
        return handle_initializing(robot, goal_pos, skill_manager)
    elif state == 'PLANNING':
        return handle_planning(robot, skill_manager)
    # ... 其他状态
```

**技能执行**：
```python
# 执行技能
result = skill_manager.execute_skill(
    'navigate_to',
    goal_pos=[10, 20, 0],
    timeout=60.0
)

# 返回结果
# {'status': 'processing', 'state': 'EXECUTING', 'progress': 65, ...}
```

**优势**：
- 全局注册，所有机器人共享技能定义
- 装饰器语法简洁
- 自动管理状态和数据
- 支持技能组合

---

### 6.2 状态转换实现

**方法 1：If-Elif 链**
```python
def execute_skill(skill_name, **kwargs):
    state = get_skill_state(skill_name)
    
    if state is None or state == 'IDLE':
        return handle_idle(**kwargs)
    elif state == 'INITIALIZING':
        return handle_initializing(**kwargs)
    elif state == 'PLANNING':
        return handle_planning(**kwargs)
    elif state == 'EXECUTING':
        return handle_executing(**kwargs)
    elif state == 'SUCCEEDED':
        return handle_succeeded(**kwargs)
    elif state == 'FAILED':
        return handle_failed(**kwargs)
```

**优势**：简单直观  
**劣势**：状态多时代码冗长

**方法 2：状态处理器字典**
```python
STATE_HANDLERS = {
    'IDLE': handle_idle,
    'INITIALIZING': handle_initializing,
    'PLANNING': handle_planning,
    'EXECUTING': handle_executing,
    'SUCCEEDED': handle_succeeded,
    'FAILED': handle_failed
}

def execute_skill(skill_name, **kwargs):
    state = get_skill_state(skill_name) or 'IDLE'
    handler = STATE_HANDLERS.get(state)
    
    if handler:
        return handler(**kwargs)
    else:
        raise ValueError(f"Unknown state: {state}")
```

**优势**：易于扩展  
**劣势**：需要定义多个处理函数

**本框架采用的方法**：混合方式
- 简单技能使用 If-Elif
- 复杂技能使用字典映射
- 平衡简洁性和可扩展性

---

### 6.3 状态持久化

**数据结构**：
```python
# 状态存储（轻量级）
skill_states = {
    'navigate_to': 'EXECUTING',
    'take_photo': 'FOCUSING'
}

# 数据存储（详细信息）
skill_data = {
    'navigate_to': {
        'start_pos': [0, 0, 0],
        'target_pos': [10, 20, 0],
        'path': [[0,0,0], [5,10,0], [10,20,0]],
        'trajectory': [...],
        'start_time': 1234567890.0,
        'retry_count': 0,
        'planning_future': <Future object>
    }
}
```

**持久化操作**：
```python
# 保存状态
def save_skill_state(skill_name, filename):
    state = skill_states.get(skill_name)
    data = skill_data.get(skill_name)
    
    with open(filename, 'w') as f:
        json.dump({
            'state': state,
            'data': serialize_data(data)
        }, f)

# 恢复状态
def load_skill_state(skill_name, filename):
    with open(filename, 'r') as f:
        saved = json.load(f)
    
    skill_states[skill_name] = saved['state']
    skill_data[skill_name] = deserialize_data(saved['data'])
```

**应用场景**：
- 任务暂停和恢复
- 系统重启后恢复
- 调试和回放
- 性能分析

---

### 6.4 ROS 集成

**通过 ROS Action 执行技能**：
```python
# ROS Action Server
class SkillExecutionActionServer:
    def execute_callback(self, goal_handle):
        """处理技能执行请求"""
        request = goal_handle.request
        skill_name = request.skill_list[0].skill
        params = parse_params(request.skill_list[0].params)
        
        # 循环执行技能
        while goal_handle.is_active:
            result = skill_manager.execute_skill(skill_name, **params)
            
            # 发布反馈
            feedback = SkillExecution.Feedback()
            feedback.status = result['status']
            feedback.progress = result.get('progress', 0)
            feedback.message = result['message']
            goal_handle.publish_feedback(feedback)
            
            # 检查完成
            if result['status'] == 'succeeded':
                goal_handle.succeed()
                return SkillExecution.Result(success=True)
            elif result['status'] == 'failed':
                goal_handle.abort()
                return SkillExecution.Result(success=False)
            
            # 等待下一帧
            time.sleep(0.1)
```

**ROS 命令行调用**：
```bash
# 导航
ros2 action send_goal /robot_0/skill_execution \
  plan_msgs/action/SkillExecution \
  '{skill_request: {skill_list: [{skill: "navigate_to", params: [{key: "goal_pos", value: "[10, 20, 0]"}]}]}}' \
  --feedback

# 起飞
ros2 action send_goal /robot_0/skill_execution \
  plan_msgs/action/SkillExecution \
  '{skill_request: {skill_list: [{skill: "take_off", params: [{key: "altitude", value: "2.0"}]}]}}' \
  --feedback

# 拍照
ros2 action send_goal /robot_0/skill_execution \
  plan_msgs/action/SkillExecution \
  '{skill_request: {skill_list: [{skill: "take_photo", params: [{key: "target", value: "car"}]}]}}' \
  --feedback
```

---

## 7. 设计优势总结

### 7.1 模块化

**优势**：
- 每个状态职责单一，易于理解
- 状态之间松耦合，易于修改
- 可以独立测试每个状态

**示例**：
- 修改路径规划算法，只需修改 PLANNING 状态
- 添加新的安全检查，只需添加新状态
- 不影响其他状态的逻辑

### 7.2 可扩展性

**优势**：
- 易于添加新技能
- 易于添加新状态
- 支持技能组合

**示例**：
```python
# 添加新技能只需定义状态转换逻辑
@SkillManager.register()
def new_skill(robot, **kwargs):
    state = get_state('new_skill')
    
    if state == 'IDLE':
        # 初始化
        pass
    elif state == 'EXECUTING':
        # 执行
        pass
    # ...
```

### 7.3 可维护性

**优势**：
- 状态转换逻辑清晰
- 易于调试（可以追踪状态历史）
- 易于文档化（状态图直观）

**调试支持**：
```python
# 记录状态转换历史
state_history = []

def transition_to(new_state):
    old_state = current_state
    current_state = new_state
    state_history.append({
        'from': old_state,
        'to': new_state,
        'timestamp': time.time()
    })
    
    # 打印状态转换
    print(f"State transition: {old_state} → {new_state}")
```

### 7.4 鲁棒性

**优势**：
- 明确的错误处理
- 支持重试和重规划
- 超时保护

**错误处理示例**：
```python
# 每个状态都有明确的失败处理
if state == 'PLANNING':
    if planning_failed():
        if can_retry():
            transition_to('INITIALIZING')  # 重试
        else:
            transition_to('FAILED')  # 失败
```

### 7.5 实时性

**优势**：
- 异步操作不阻塞
- 支持高频控制
- 实时反馈

**性能**：
- 状态机检查频率：~10 Hz
- 控制器运行频率：60 Hz
- 反馈延迟：< 100ms

---

## 8. 技能对比分析

### 8.1 不同技能的状态复杂度

| 技能 | 状态数量 | 异步状态 | 支持重试 | 并行控制 | 复杂度 |
|------|---------|---------|---------|---------|--------|
| navigate_to | 7 | 2 (规划、轨迹) | ✓ | MPC | 高 |
| take_off | 6 | 0 | ✓ | PID | 中 |
| take_photo | 8 | 1 (对焦) | ✓ | 姿态控制 | 中 |
| manipulation | 8 | 2 (检测、规划) | ✓ | 轨迹跟踪 | 高 |
| hover | 3 | 0 | - | PID | 低 |
| land | 5 | 0 | ✓ | PID | 中 |

**观察**：
- 复杂技能通常有更多状态
- 异步操作增加了实现复杂度
- 所有技能都支持错误处理
- 并行控制是常见模式

### 8.2 状态模式的通用性

**通用状态模式**：
```
IDLE → INITIALIZING → PREPARING → EXECUTING → SUCCEEDED/FAILED
```

**各技能的映射**：

**Navigate To**：
- PREPARING = PATH_PLANNING + TRAJECTORY_GENERATION
- EXECUTING = 轨迹跟踪

**Take Off**：
- PREPARING = CHECKING（安全检查）
- EXECUTING = ASCENDING + HOVERING

**Take Photo**：
- PREPARING = MOVING + ADJUSTING
- EXECUTING = FOCUSING + CAPTURING + SAVING

**Manipulation**：
- PREPARING = DETECTING + PLANNING
- EXECUTING = APPROACHING + GRASPING + LIFTING

**结论**：
- 大多数技能可以映射到通用模式
- PREPARING 阶段可能包含多个子状态
- EXECUTING 阶段可能包含多个子状态
- 可以通过状态嵌套进一步简化

---

## 9. 与其他方法的比较

### 9.1 vs. 行为树（Behavior Tree）

**行为树特点**：
- 层次化结构
- 支持复杂的控制流（Sequence, Selector, Parallel）
- 易于可视化编辑

**状态机特点**：
- 扁平化结构
- 状态转换明确
- 实现简单

**对比**：

| 特性 | 行为树 | 状态机 |
|------|--------|--------|
| 学习曲线 | 陡峭 | 平缓 |
| 适用场景 | 复杂决策 | 线性任务 |
| 可视化 | 树形图 | 状态图 |
| 实现复杂度 | 高 | 低 |
| 调试难度 | 中 | 低 |
| 异步支持 | 需要特殊节点 | 原生支持 |

**选择建议**：
- 简单线性任务：状态机
- 复杂决策任务：行为树
- 混合使用：状态机处理单个技能，行为树组合多个技能

### 9.2 vs. 任务规划器（Task Planner）

**任务规划器特点**：
- 基于 PDDL 等形式化语言
- 自动生成执行计划
- 适合高层决策

**状态机特点**：
- 手动定义状态转换
- 执行效率高
- 适合底层控制

**对比**：

| 特性 | 任务规划器 | 状态机 |
|------|-----------|--------|
| 灵活性 | 高（自动规划） | 低（手动定义） |
| 执行效率 | 低 | 高 |
| 实时性 | 差 | 好 |
| 适用层次 | 高层决策 | 底层执行 |
| 学习成本 | 高 | 低 |

**选择建议**：
- 高层任务规划：任务规划器
- 底层技能执行：状态机
- 混合使用：规划器生成任务序列，状态机执行单个任务

### 9.3 vs. 分层有限状态机（HFSM）

**HFSM 特点**：
- 支持状态嵌套
- 减少状态数量
- 支持状态复用

**本框架的状态机**：
- 扁平化结构
- 状态数量较多
- 实现简单

**对比**：

| 特性 | HFSM | 扁平状态机 |
|------|------|-----------|
| 状态数量 | 少 | 多 |
| 实现复杂度 | 高 | 低 |
| 可读性 | 中 | 高 |
| 适用场景 | 复杂系统 | 简单任务 |

**未来改进方向**：
- 引入状态嵌套
- 例如：PREPARING 状态可以包含子状态机
- 减少顶层状态数量

---

## 10. 最佳实践

### 10.1 状态设计原则

**1. 状态职责单一**
```python
# 好的设计
PLANNING  # 只负责路径规划
EXECUTING # 只负责执行

# 不好的设计
PLANNING_AND_EXECUTING  # 职责不清晰
```

**2. 转换条件明确**
```python
# 好的设计
if distance < threshold:
    transition_to('SUCCEEDED')

# 不好的设计
if maybe_arrived():  # 条件模糊
    transition_to('SUCCEEDED')
```

**3. 避免状态爆炸**
```python
# 如果状态过多，考虑：
# - 合并相似状态
# - 使用状态嵌套
# - 使用参数化状态
```

### 10.2 异步操作处理

**模式**：
```python
def handle_async_state(skill_name, **kwargs):
    # 第一次进入：发送请求
    if not has_future(skill_name):
        future = send_async_request(**kwargs)
        store_future(skill_name, future)
        return {'status': 'processing', 'message': 'Request sent'}
    
    # 后续调用：检查完成
    future = get_future(skill_name)
    if not future.done():
        return {'status': 'processing', 'message': 'Waiting...'}
    
    # 完成：处理结果
    result = future.result()
    if result.success:
        store_result(skill_name, result)
        transition_to(skill_name, 'NEXT_STATE')
        return {'status': 'processing', 'message': 'Completed'}
    else:
        transition_to(skill_name, 'FAILED')
        return {'status': 'failed', 'message': result.error}
```

### 10.3 错误处理策略

**分类处理**：
```python
def handle_error(error_type, retry_count, max_retries):
    if error_type == 'RECOVERABLE':
        if retry_count < max_retries:
            return 'RETRY'
        else:
            return 'FAIL'
    elif error_type == 'UNRECOVERABLE':
        return 'FAIL'
    elif error_type == 'NEED_REPLAN':
        return 'REPLAN'
```

### 10.4 状态转换日志

**记录所有转换**：
```python
def transition_to(skill_name, new_state):
    old_state = get_skill_state(skill_name)
    
    # 记录转换
    logger.info(f"[{skill_name}] State transition: {old_state} → {new_state}")
    
    # 更新状态
    set_skill_state(skill_name, new_state)
    
    # 触发回调
    on_state_changed(skill_name, old_state, new_state)
```

### 10.5 性能优化

**1. 避免频繁的状态检查**
```python
# 不好的做法
def execute_skill():
    if state == 'EXECUTING':
        # 每帧都检查
        if check_expensive_condition():
            pass

# 好的做法
def execute_skill():
    if state == 'EXECUTING':
        # 使用计数器减少检查频率
        if frame_count % 10 == 0:
            if check_expensive_condition():
                pass
```

**2. 缓存计算结果**
```python
# 将计算结果存储到 skill_data
skill_data[skill_name]['cached_result'] = expensive_computation()
```

---

## 11. 总结

### 11.1 核心思想

本文提出的状态机架构基于以下核心思想：

1. **任务分解**：将复杂任务分解为多个离散状态
2. **明确转换**：状态之间的转换条件清晰可判断
3. **数据分离**：状态和数据独立存储
4. **异步支持**：原生支持异步操作
5. **分层控制**：状态机负责决策，控制器负责执行

### 11.2 适用场景

该架构特别适合以下场景：

1. **多步骤任务**：需要按顺序执行多个步骤
2. **异步操作**：包含耗时的计算或等待
3. **错误处理**：需要支持重试和重规划
4. **实时反馈**：需要向用户报告进度
5. **并行控制**：需要与底层控制器集成

### 11.3 实际应用

该架构已成功应用于多种机器人技能：

- **导航类**：navigate_to, follow_path, explore
- **飞行类**：take_off, land, hover, waypoint_navigation
- **视觉类**：take_photo, object_detection, visual_servoing
- **操作类**：manipulation, pick_and_place, assembly

### 11.4 未来方向

**短期改进**：
1. 引入状态嵌套（HFSM）
2. 支持并发技能执行
3. 可视化状态机编辑器

**长期方向**：
1. 基于学习的状态转换
2. 自动生成状态机
3. 形式化验证

---

## 附录：完整代码示例

### A.1 简单技能示例（Hover）

```python
@SkillManager.register()
def hover(robot, altitude=2.0, duration=5.0, **kwargs):
    """悬停技能：在指定高度悬停指定时间"""
    skill_manager = kwargs['skill_manager']
    state = skill_manager.get_skill_state('hover')
    
    # 初始化
    if state is None:
        skill_manager.set_skill_state('hover', 'INITIALIZING')
        skill_manager.set_skill_data('hover', 'target_altitude', altitude)
        skill_manager.set_skill_data('hover', 'duration', duration)
        skill_manager.set_skill_data('hover', 'start_time', time.time())
        return {'status': 'processing', 'message': 'Initializing hover'}
    
    # 上升到目标高度
    if state == 'INITIALIZING':
        current_altitude = robot.get_world_pose()[0][2]
        target_altitude = skill_manager.get_skill_data('hover', 'target_altitude')
        
        if abs(current_altitude - target_altitude) < 0.1:
            skill_manager.set_skill_state('hover', 'HOVERING')
            return {'status': 'processing', 'message': 'Reached altitude'}
        else:
            # 设置目标高度（控制器会自动跟踪）
            robot.set_target_altitude(target_altitude)
            return {'status': 'processing', 'message': f'Ascending to {target_altitude}m'}
    
    # 悬停
    if state == 'HOVERING':
        start_time = skill_manager.get_skill_data('hover', 'start_time')
        duration = skill_manager.get_skill_data('hover', 'duration')
        elapsed = time.time() - start_time
        
        if elapsed >= duration:
            skill_manager.set_skill_state('hover', 'SUCCEEDED')
            return {'status': 'succeeded', 'message': 'Hover completed'}
        else:
            progress = int((elapsed / duration) * 100)
            return {
                'status': 'processing',
                'message': f'Hovering ({progress}%)',
                'progress': progress
            }
    
    # 成功
    if state == 'SUCCEEDED':
        return {'status': 'succeeded', 'message': 'Hover completed'}
```

### A.2 复杂技能示例（Navigate To）

参见 `docs/navigate_to_skill_architecture.md`

---

**文档版本**：v2.0  
**最后更新**：2024年  
**适用于**：技术文档、系统设计、论文参考
