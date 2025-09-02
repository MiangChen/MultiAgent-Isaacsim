# Requirements Document

## Introduction

本项目旨在增强现有的viewport管理器功能，实现一个简洁高效的viewport管理系统。该系统将从scene_manager中拆分出viewport相关功能，提供viewport实例化记录、相机映射、以及精确的viewport切换功能。重点关注代码简洁性和性能优化。

## Requirements

### Requirement 1

**User Story:** 作为开发者，我希望能够记录和管理所有viewport对应的windows实例化名字，以便后续能够通过名字精确访问这些viewport。

#### Acceptance Criteria

1. WHEN 创建viewport时 THEN 系统 SHALL 自动记录viewport的名字和对应的实例对象
2. WHEN 查询viewport时 THEN 系统 SHALL 能够通过名字快速返回对应的viewport实例
3. WHEN viewport被销毁时 THEN 系统 SHALL 自动清理相关的记录信息
4. IF viewport名字重复 THEN 系统 SHALL 覆盖旧的记录并给出警告信息

### Requirement 2

**User Story:** 作为开发者，我希望能够指定使用哪个viewport以及哪个相机的prim path来实现精确的viewport切换。

#### Acceptance Criteria

1. WHEN 调用change_viewport函数时 THEN 系统 SHALL 接受viewport_name和camera_prim_path两个参数
2. WHEN viewport_name为空时 THEN 系统 SHALL 使用当前活跃的viewport
3. WHEN camera_prim_path无效时 THEN 系统 SHALL 返回错误信息而不是抛出异常
4. WHEN 切换成功时 THEN 系统 SHALL 返回成功状态和相关信息

### Requirement 3

**User Story:** 作为开发者，我希望将viewport相关的功能从scene_manager中独立出来，形成一个专门的viewport_manager模块。

#### Acceptance Criteria

1. WHEN 重构完成后 THEN viewport_manager SHALL 包含所有viewport相关的核心功能
2. WHEN scene_manager调用viewport功能时 THEN 应该委托给viewport_manager处理
3. WHEN viewport_manager初始化时 THEN 不应该依赖scene_manager的其他功能
4. IF 需要与scene_manager交互 THEN 应该通过明确的接口进行

### Requirement 4

**User Story:** 作为开发者，我希望代码尽量简洁，减少try-except的使用，并且class设计要精简。

#### Acceptance Criteria

1. WHEN 编写代码时 THEN 应该优先使用条件判断而不是异常处理
2. WHEN 设计class时 THEN 每个class的方法数量 SHALL 不超过15个
3. WHEN 实现功能时 THEN 单个方法的代码行数 SHALL 不超过30行
4. IF 必须使用try-except THEN 应该只在与外部API交互时使用

### Requirement 5

**User Story:** 作为开发者，我希望能够批量管理robot的viewport，包括注册和映射功能。

#### Acceptance Criteria

1. WHEN 传入robot列表时 THEN 系统 SHALL 能够批量注册这些robot的viewport
2. WHEN robot有viewport信息时 THEN 系统 SHALL 自动提取viewport_name和viewport_obj
3. WHEN 批量操作完成时 THEN 系统 SHALL 返回成功和失败的统计信息
4. IF robot缺少viewport信息 THEN 系统 SHALL 跳过该robot并记录失败原因

### Requirement 6

**User Story:** 作为开发者，我希望能够查询和管理viewport与camera之间的映射关系。

#### Acceptance Criteria

1. WHEN 建立映射关系时 THEN 系统 SHALL 验证camera prim path的有效性
2. WHEN 查询映射时 THEN 系统 SHALL 能够返回viewport对应的camera路径
3. WHEN 删除映射时 THEN 系统 SHALL 同时清理正向和反向映射关系
4. WHEN 列出所有映射时 THEN 系统 SHALL 返回完整的映射信息列表