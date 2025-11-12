# Navigate To 技能架构说明

## 概述

Navigate To 技能使用状态机模式实现，结合了路径规划、轨迹生成和 MPC 控制，实现机器人的自主导航。

---

## 一、状态机架构

### 状态定义

```
状态机：NavigateToSkill

状态列表：
1. IDLE          - 空闲状态
2. INITIALIZING  - 初始化状态
3. PATH_PLANNING - 路径规划状态
4. TRAJECTORY_GENERATION - 轨迹生成状态
5. EXECUTING     - 执行状态
6. SUCCEEDED     - 成功状态
7. FAILED        - 失败状态
```

### 状态转换图

```
                    [收到导航请求]
    IDLE ──────────────────────────→ INITIALIZING
                                           │
                                           │ [获取位置成功]
                                           ↓
                                    PATH_PLANNING
                                           │
                        ┌──────────────────┼──────────────────┐
                        │ [规划成功]       │                  │ [规划失败]
                        ↓                  │                  ↓
            TRAJECTORY_GENERATION          │                FAILED
                        │                  │
                        │ [生成成功]       │ [生成失败]
                        ↓                  ↓
                    EXECUTING ────────→ FAILED
                        │
                        │ [循环检查]
                        │ • 到达目标？
                        │ • 超时？
                        │ • 碰撞？
                        │
        ┌───────────────┼───────────────┐
        │ [到达目标]    │               │ [需要重规划]
        ↓               │               ↓
    SUCCEEDED           │         PATH_PLANNING
                        │ [超时/碰撞]
                        ↓
                     FAILED
```

---

## 二、各状态详细说明

### 状态 1: IDLE (空闲)

**职责**：等待导航请求

**输入**：无

**输出**：无

**转换条件**：
- 收到导航请求 → INITIALIZING

---

### 状态 2: INITIALIZING (初始化)

**职责**：初始化导航任务，获取必要信息

**执行操作**：
1. 解析目标位置和姿态 (target_pos, target_quat)
2. 获取机器人当前位置 (current_pos, current_quat)
3. 验证目标位置的有效性
4. 初始化技能数据存储

**输入**：
- `goal_pos: List[float]` - 目标位置 [x, y, z]
- `goal_quat: List[float]` - 目标姿态 [qx, qy, qz, qw] (可选)

**输出**：
- `start_pos: Tensor` - 起始位置
- `target_pos: Tensor` - 目标位置

**转换条件**：
- 获取位置成功 → PATH_PLANNING
- 获取位置失败 → FAILED

**代码示例**：
```python
if state == 'IDLE':
    # 首次调用，初始化
    start_pos, start_quat = robot.get_world_pose()
    target_pos = torch.tensor(goal_pos)
    
    # 存储到 skill_data
    skill_manager.set_skill_data('navigate_to', 'start_pos', start_pos)
    skill_manager.set_skill_data('navigate_to', 'target_pos', target_pos)
    
    # 转换状态
    skill_manager.set_skill_state('navigate_to', 'INITIALIZING')
    return {'status': 'processing', 'message': 'Initializing navigation'}
```

---

### 状态 3: PATH_PLANNING (路径规划)

**职责**：使用 OMPL 算法规划无碰撞路径并进行平滑

**执行操作**：
1. 获取 GridMap 的 3D 点云地图
2. 调用 OMPL Planner (RRT/RRT*/PRM 等算法)
3. 输入：起始位置、目标位置、点云地图
4. 输出：原始路径点列表
5. 对路径进行平滑处理
6. 验证路径的可行性

**输入**：
- `start_pos: Tensor` - 起始位置
- `target_pos: Tensor` - 目标位置
- `grid_map: GridMap` - 3D 点云地图
- `robot_radius: float` - 机器人半径（用于碰撞检测）

**输出**：
- `raw_path: List[List[float]]` - 原始路径点
- `smoothed_path: List[List[float]]` - 平滑后的路径点

**转换条件**：
- 规划成功 → TRAJECTORY_GENERATION
- 规划失败（无可行路径）→ FAILED
- 超时 → FAILED

**代码示例**：
```python
if state == 'INITIALIZING':
    # 发送路径规划请求
    start_pos = skill_manager.get_skill_data('navigate_to', 'start_pos')
    target_pos = skill_manager.get_skill_data('navigate_to', 'target_pos')
    
    # 调用 OMPL Planner (通过 ROS Action)
    goal = ComputePathToPose.Goal()
    goal.start = pose_to_msg(start_pos)
    goal.goal = pose_to_msg(target_pos)
    
    future = action_client_path_planner.send_goal_async(goal)
    skill_manager.set_skill_data('navigate_to', 'planning_future', future)
    
    # 转换状态
    skill_manager.set_skill_state('navigate_to', 'PATH_PLANNING')
    return {'status': 'processing', 'message': 'Planning path'}

if state == 'PATH_PLANNING':
    # 检查规划是否完成
    future = skill_manager.get_skill_data('navigate_to', 'planning_future')
    
    if not future.done():
        return {'status': 'processing', 'message': 'Planning in progress'}
    
    result = future.result()
    if result.status == 'success':
        path = result.path
        skill_manager.set_skill_data('navigate_to', 'path', path)
        skill_manager.set_skill_state('navigate_to', 'TRAJECTORY_GENERATION')
        return {'status': 'processing', 'message': 'Path planning succeeded'}
    else:
        skill_manager.set_skill_state('navigate_to', 'FAILED')
        return {'status': 'failed', 'message': 'Path planning failed'}
```

**OMPL 算法选择**：
- **RRT (Rapidly-exploring Random Tree)**：快速，适合简单环境
- **RRT\***：最优路径，适合复杂环境
- **PRM (Probabilistic Roadmap)**：预计算，适合多次查询

**路径平滑**：
- 使用 B-spline 或 Bezier 曲线
- 移除冗余路径点
- 确保路径连续性

---

### 状态 4: TRAJECTORY_GENERATION (轨迹生成)

**职责**：使用 TOPPRA 算法生成带时间戳的轨迹

**执行操作**：
1. 获取平滑后的路径
2. 获取机器人动力学约束：
   - 最大线速度 (v_max)
   - 最大角速度 (ω_max)
   - 最大线加速度 (a_max)
   - 最大角加速度 (α_max)
3. 调用 TOPPRA 算法
4. 生成带时间戳的轨迹：`[(t0, pos0, vel0), (t1, pos1, vel1), ...]`
5. 将轨迹发送到 MPC 控制器

**输入**：
- `smoothed_path: List[List[float]]` - 平滑路径
- `robot_dynamics: Dict` - 机器人动力学约束
  ```python
  {
      'v_max': 1.0,      # m/s
      'omega_max': 1.0,  # rad/s
      'a_max': 0.5,      # m/s²
      'alpha_max': 0.5   # rad/s²
  }
  ```

**输出**：
- `trajectory: List[Tuple]` - 带时间戳的轨迹
  ```python
  [
      (t=0.0, pos=[0,0,0], vel=[0,0,0]),
      (t=0.1, pos=[0.05,0,0], vel=[0.5,0,0]),
      ...
  ]
  ```

**转换条件**：
- 生成成功 → EXECUTING
- 生成失败（动力学约束无法满足）→ FAILED

**代码示例**：
```python
if state == 'TRAJECTORY_GENERATION':
    # 发送轨迹生成请求
    path = skill_manager.get_skill_data('navigate_to', 'path')
    
    # 调用 Trajectory Generator (通过 ROS Action)
    goal = GenerateTrajectory.Goal()
    goal.path = path
    goal.dynamics = robot.get_dynamics_constraints()
    
    future = action_client_trajectory_gen.send_goal_async(goal)
    skill_manager.set_skill_data('navigate_to', 'traj_future', future)
    
    # 等待生成完成
    if not future.done():
        return {'status': 'processing', 'message': 'Generating trajectory'}
    
    result = future.result()
    if result.status == 'success':
        trajectory = result.trajectory
        
        # 将轨迹设置到 MPC 控制器
        mpc_controller.set_reference_trajectory(trajectory)
        
        skill_manager.set_skill_data('navigate_to', 'trajectory', trajectory)
        skill_manager.set_skill_state('navigate_to', 'EXECUTING')
        return {'status': 'processing', 'message': 'Trajectory generated'}
    else:
        skill_manager.set_skill_state('navigate_to', 'FAILED')
        return {'status': 'failed', 'message': 'Trajectory generation failed'}
```

**TOPPRA 算法**：
- Time-Optimal Path Parameterization with Reachability Analysis
- 考虑动力学约束生成时间最优轨迹
- 确保速度和加速度在限制范围内

---

### 状态 5: EXECUTING (执行中)

**职责**：MPC 控制器循环跟踪轨迹，直到到达目标

**执行操作**：
1. MPC 控制器自动运行（通过 clock_callback 触发）
2. 每帧执行：
   - 获取机器人当前位置
   - 从轨迹中提取当前时刻的参考点
   - MPC 计算最优控制命令
   - 直接设置 `robot.target_velocity`
3. 技能管理器每帧检查：
   - 计算到目标的距离
   - 检查是否到达目标（距离 < 阈值）
   - 检查是否超时
   - 检查是否发生碰撞
4. 发布实时反馈（进度、距离、速度等）

**输入**：
- `trajectory: List[Tuple]` - 参考轨迹
- `current_pos: Tensor` - 当前位置（每帧更新）
- `current_time: float` - 当前时间

**输出**：
- `control_command: Tuple[Tensor, Tensor]` - 控制命令 (v, ω)
- `feedback: Dict` - 实时反馈

**转换条件**：
- 到达目标（距离 < 0.1m）→ SUCCEEDED
- 超时（时间 > timeout）→ FAILED
- 碰撞检测 → PATH_PLANNING (重新规划)
- 轨迹跟踪误差过大 → PATH_PLANNING (重新规划)

**代码示例**：
```python
if state == 'EXECUTING':
    # 获取当前位置
    current_pos, current_quat = robot.get_world_pose()
    target_pos = skill_manager.get_skill_data('navigate_to', 'target_pos')
    
    # 计算到目标的距离
    distance = torch.norm(current_pos - target_pos).item()
    
    # 检查是否到达目标
    if distance < 0.1:  # 阈值 10cm
        skill_manager.set_skill_state('navigate_to', 'SUCCEEDED')
        return {
            'status': 'succeeded',
            'message': 'Reached goal',
            'final_distance': distance
        }
    
    # 检查超时
    start_time = skill_manager.get_skill_data('navigate_to', 'start_time')
    elapsed_time = time.time() - start_time
    timeout = kwargs.get('timeout', 60.0)
    
    if elapsed_time > timeout:
        skill_manager.set_skill_state('navigate_to', 'FAILED')
        return {
            'status': 'failed',
            'message': 'Navigation timeout',
            'elapsed_time': elapsed_time
        }
    
    # 检查碰撞（可选）
    if robot.is_collision_detected():
        # 重新规划
        skill_manager.set_skill_state('navigate_to', 'PATH_PLANNING')
        return {
            'status': 'processing',
            'message': 'Collision detected, replanning'
        }
    
    # 计算进度
    start_pos = skill_manager.get_skill_data('navigate_to', 'start_pos')
    total_distance = torch.norm(target_pos - start_pos).item()
    traveled_distance = torch.norm(current_pos - start_pos).item()
    progress = int((traveled_distance / total_distance) * 100)
    
    # 返回反馈
    return {
        'status': 'processing',
        'message': 'Navigating to goal',
        'progress': progress,
        'distance_to_goal': distance,
        'current_velocity': robot.get_velocity().tolist()
    }
```

**MPC 控制循环（并行运行）**：
```python
# 在 NodeMpcController 中
def clock_callback(self, msg: Clock):
    """每次 world.tick() 后自动调用"""
    self.control_loop()

def control_loop(self):
    """MPC 控制循环"""
    if not self.has_reference_trajectory():
        return
    
    # 1. 获取当前状态
    current_pos, current_quat = self.robot.get_world_pose()
    current_vel = self.robot.get_velocity()
    current_time = self.get_current_time()
    
    # 2. 从轨迹中提取参考点
    ref_segment = self.get_trajectory_segment(
        current_time,
        horizon=self.mpc_horizon  # 例如 1.0 秒
    )
    
    # 3. MPC 求解
    optimal_command = self.mpc_controller.solve(
        current_state=(current_pos, current_vel),
        reference=ref_segment,
        constraints=self.robot.get_dynamics_constraints()
    )
    
    # 4. 直接设置目标速度（同步，无延迟）
    self.robot.target_linear_velocity = torch.tensor([
        optimal_command[0],  # vx
        optimal_command[1],  # vy
        0.0
    ])
    self.robot.target_angular_velocity = torch.tensor([
        0.0,
        0.0,
        optimal_command[2]   # ωz
    ])
    
    # 5. 发布 cmd_vel（用于监控）
    self.publish_cmd_vel(optimal_command)
```

**关键设计**：
- MPC 自动运行，技能管理器只负责监控
- 同步设置速度，无 ROS 延迟
- 实时反馈让用户了解执行进度

---

### 状态 6: SUCCEEDED (成功)

**职责**：导航成功完成

**执行操作**：
1. 停止机器人（设置速度为 0）
2. 清理技能数据
3. 返回成功结果

**输出**：
```python
{
    'status': 'succeeded',
    'message': 'Navigation completed successfully',
    'final_distance': 0.05,
    'total_time': 15.3
}
```

---

### 状态 7: FAILED (失败)

**职责**：导航失败

**执行操作**：
1. 停止机器人
2. 记录失败原因
3. 清理技能数据
4. 返回失败结果

**输出**：
```python
{
    'status': 'failed',
    'message': 'Navigation failed: timeout',
    'reason': 'timeout',
    'elapsed_time': 60.5
}
```

---

## 三、数据流图

```
输入数据流：
┌─────────────┐
│  User Input │
│ goal_pos    │
│ goal_quat   │
└──────┬──────┘
       │
       ↓
┌─────────────────────────────────────────────────────────┐
│                    Navigate To Skill                     │
│                                                          │
│  ┌──────────────┐                                       │
│  │ INITIALIZING │                                       │
│  │ • 获取当前位置│                                       │
│  │ • 验证目标   │                                       │
│  └──────┬───────┘                                       │
│         │                                                │
│         ↓                                                │
│  ┌──────────────┐      ┌──────────────┐               │
│  │PATH_PLANNING │ ←──→ │   GridMap    │               │
│  │ • OMPL (RRT) │      │ (3D点云地图) │               │
│  │ • 路径平滑   │      └──────────────┘               │
│  └──────┬───────┘                                       │
│         │                                                │
│         ↓                                                │
│  ┌──────────────┐      ┌──────────────┐               │
│  │TRAJECTORY_GEN│ ←──→ │Robot Dynamics│               │
│  │ • TOPPRA     │      │ v_max, a_max │               │
│  │ • 时间标签   │      │ ω_max, α_max │               │
│  └──────┬───────┘      └──────────────┘               │
│         │                                                │
│         ↓                                                │
│  ┌──────────────┐                                       │
│  │  EXECUTING   │                                       │
│  │ • 监控进度   │                                       │
│  │ • 检查到达   │                                       │
│  │ • 发布反馈   │                                       │
│  └──────┬───────┘                                       │
│         │                                                │
│         ↓                                                │
│  ┌──────────────┐                                       │
│  │  SUCCEEDED   │                                       │
│  └──────────────┘                                       │
└─────────────────────────────────────────────────────────┘
       │
       ↓
┌──────────────┐
│ MPC Controller│ (并行运行)
│ • 订阅轨迹   │
│ • 计算控制   │
│ • 设置速度   │
└──────┬───────┘
       │
       ↓
┌──────────────┐
│    Robot     │
│ • 应用速度   │
│ • 更新位置   │
└──────────────┘
```

---

## 四、时序图（一次完整导航）

```
时间轴 →

t=0.0s    用户发起请求
          ↓
t=0.1s    INITIALIZING
          • 获取当前位置: [0, 0, 0]
          • 目标位置: [10, 20, 0]
          ↓
t=0.2s    PATH_PLANNING
          • 调用 OMPL Planner
          ↓
t=1.5s    • OMPL 计算完成
          • 路径点: [[0,0,0], [2,3,0], [5,8,0], [10,20,0]]
          • 路径平滑
          ↓
t=1.8s    TRAJECTORY_GENERATION
          • 调用 TOPPRA
          ↓
t=2.0s    • TOPPRA 生成完成
          • 轨迹: [(0.0, [0,0,0], [0,0,0]), (0.1, [0.05,0,0], [0.5,0,0]), ...]
          • 设置到 MPC 控制器
          ↓
t=2.1s    EXECUTING
          ├─ 技能管理器（每帧）
          │  • 检查距离: 22.4m
          │  • 检查超时: 2.1s / 60s
          │  • 发布反馈: progress=0%
          │
          └─ MPC 控制器（并行，每帧）
             • 获取当前位置: [0, 0, 0]
             • 提取参考轨迹段
             • 计算最优控制: v=0.5m/s, ω=0.1rad/s
             • 设置 robot.target_velocity
          ↓
t=2.2s    EXECUTING (循环)
          ├─ 技能管理器
          │  • 检查距离: 22.3m
          │  • 发布反馈: progress=1%
          │
          └─ MPC 控制器
             • 当前位置: [0.05, 0, 0]
             • 计算控制: v=0.6m/s, ω=0.15rad/s
          ↓
...       (持续循环)
          ↓
t=17.5s   EXECUTING
          ├─ 技能管理器
          │  • 检查距离: 0.08m < 0.1m (阈值)
          │  • 到达目标！
          │  • 转换状态: SUCCEEDED
          │
          └─ MPC 控制器
             • 停止控制
          ↓
t=17.6s   SUCCEEDED
          • 返回成功结果
          • 清理数据
```

---

## 五、关键设计特性

### 1. 状态机模式

**优势**：
- 清晰的状态转换逻辑
- 支持异步操作（路径规划、轨迹生成）
- 易于调试和维护
- 支持状态持久化（可以暂停和恢复）

**实现**：
```python
def execute_skill(skill_name: str, **kwargs) -> Dict:
    state = skill_manager.get_skill_state(skill_name)
    
    if state is None or state == 'IDLE':
        # 首次调用，初始化
        return _handle_idle_state(**kwargs)
    elif state == 'INITIALIZING':
        return _handle_initializing_state(**kwargs)
    elif state == 'PATH_PLANNING':
        return _handle_path_planning_state(**kwargs)
    # ... 其他状态
```

### 2. 异步操作

**路径规划和轨迹生成是异步的**：
- 发送请求后立即返回
- 下次调用时检查是否完成
- 不阻塞主循环

### 3. MPC 并行控制

**MPC 控制器独立运行**：
- 订阅仿真时钟，自动触发
- 直接设置 `robot.target_velocity`
- 技能管理器只负责监控，不参与控制

### 4. 实时反馈

**每帧发布反馈**：
- 进度百分比
- 到目标的距离
- 当前速度
- 预计剩余时间

### 5. 错误处理和重规划

**支持动态重规划**：
- 碰撞检测 → 重新规划路径
- 轨迹跟踪误差过大 → 重新规划
- 超时 → 失败

---

## 六、代码位置

```
application/
└── skills/
    └── base/
        └── navigation/
            ├── navigate_to.py              # 技能主逻辑
            ├── node_planner_ompl.py        # OMPL 路径规划器
            ├── node_trajectory_generator.py # TOPPRA 轨迹生成器
            └── node_controller_mpc.py      # MPC 控制器
```

---

## 七、使用示例

### Python API
```python
# 执行导航
result = skill_manager.execute_skill(
    'navigate_to',
    goal_pos=[10, 20, 0],
    goal_quat=[0, 0, 0, 1],  # 可选
    timeout=60.0              # 可选
)
```

### ROS Action
```bash
ros2 action send_goal /robot_0/skill_execution \
  plan_msgs/action/SkillExecution \
  '{skill_request: {
    skill_list: [{
      skill: "navigate_to",
      params: [
        {key: "goal_pos", value: "[10, 20, 0]"},
        {key: "timeout", value: "60.0"}
      ]
    }]
  }}' --feedback
```

---

## 八、性能指标

- **路径规划时间**：通常 0.5-2.0 秒（取决于环境复杂度）
- **轨迹生成时间**：通常 0.1-0.5 秒
- **MPC 控制频率**：60 Hz（与仿真频率一致）
- **到达精度**：< 10 cm
- **最大速度**：1.0 m/s（可配置）
- **最大角速度**：1.0 rad/s（可配置）

---

**文档版本**：v1.0  
**最后更新**：2024年  
**作者**：Navigation Team
