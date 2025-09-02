# 代码清理需求文档

## 介绍

本项目是一个基于Isaac Sim的机器人仿真系统，包含多个机器人类型、控制器、地图管理和路径规划等功能。经过代码分析，发现存在大量未使用的代码文件和函数，需要进行清理以提高代码可维护性和项目结构清晰度。

## 需求

### 需求 1: 清理未使用的机器人相关文件

**用户故事:** 作为开发者，我希望移除未使用的机器人类文件，以便项目结构更加清晰。

#### 验收标准

1. WHEN 检查代码引用时 THEN 系统应将未被引用的机器人文件移动到 /trash 文件夹
2. WHEN 移动 robot_cartpole.py 时 THEN 该文件应被移动到 /trash/robot/ 目录
3. WHEN 移动 robot_g1.py 时 THEN 该文件应被移动到 /trash/robot/ 目录（该文件存在但未被使用）
4. WHEN 移动 robot_trajectory.py 时 THEN 该文件应被移动到 /trash/robot/ 目录

### 需求 2: 清理未使用的控制器文件

**用户故事:** 作为开发者，我希望移除未使用的控制器文件，以便减少代码冗余。

#### 验收标准

1. WHEN 检查控制器使用情况时 THEN 系统应将未被引用的控制器文件移动到 /trash 文件夹
2. WHEN 移动 controller_policy.py 时 THEN 该文件应被移动到 /trash/controller/ 目录
3. WHEN 移动 controller_policy_g1.py 时 THEN 该文件应被移动到 /trash/controller/ 目录
4. WHEN 移动 controller_head_to.py 时 THEN 该文件应被移动到 /trash/controller/ 目录

### 需求 3: 清理未使用的工具文件

**用户故事:** 作为开发者，我希望移除未使用的工具文件，以便项目更加精简。

#### 验收标准

1. WHEN 检查工具文件使用情况时 THEN 系统应将未被引用的工具文件移动到 /trash 文件夹
2. WHEN 移动 utils/amazon2local.py 时 THEN 该文件应被移动到 /trash/utils/ 目录（如果存在）
3. WHEN 移动 utils/usdc2usd.py 时 THEN 该文件应被移动到 /trash/utils/ 目录
4. WHEN 移动 utils/quat_to_angle.py 时 THEN 该文件应被移动到 /trash/utils/ 目录

### 需求 4: 清理未使用的地图和路径规划文件

**用户故事:** 作为开发者，我希望移除未使用的地图和路径规划文件，以便项目结构更清晰。

#### 验收标准

1. WHEN 检查地图文件使用情况时 THEN 系统应将未被引用的地图文件移动到 /trash 文件夹
2. WHEN 移动 map/map_point_cloud.py 时 THEN 该文件应被移动到 /trash/map/ 目录（如果存在）
3. WHEN 移动 path_planning/path_planning_astar.py 时 THEN 该文件应被移动到 /trash/path_planning/ 目录
4. WHEN 移动 path_planning/path_planning_rrt.py 时 THEN 该文件应被移动到 /trash/path_planning/ 目录

### 需求 5: 清理未使用的相机配置文件

**用户故事:** 作为开发者，我希望评估相机配置文件的使用情况，决定是否保留。

#### 验收标准

1. WHEN 检查 camera_third_person_cfg.py 使用情况时 THEN 系统应分析该文件是否被实际使用
2. IF 该文件仅在未使用的代码中被引用 THEN 该文件应被移动到 /trash/camera/ 目录
3. IF 该文件在活跃代码中被使用 THEN 该文件应保留在原位置

### 需求 6: 保持项目功能完整性

**用户故事:** 作为开发者，我希望在清理代码的同时保持项目的核心功能不受影响。

#### 验收标准

1. WHEN 移动文件到 /trash 目录时 THEN 系统应确保不影响 main.py 的正常运行
2. WHEN 移动文件后 THEN 系统应确保所有被 main.py 直接或间接引用的文件都保持在原位置
3. WHEN 清理完成后 THEN 项目应能正常启动和运行基本功能

### 需求 7: 创建清理记录

**用户故事:** 作为开发者，我希望有清理操作的记录，以便了解哪些文件被移动了。

#### 验收标准

1. WHEN 执行清理操作时 THEN 系统应创建一个清理日志文件
2. WHEN 记录清理操作时 THEN 日志应包含被移动文件的原路径和新路径
3. WHEN 清理完成时 THEN 日志应包含清理操作的总结信息