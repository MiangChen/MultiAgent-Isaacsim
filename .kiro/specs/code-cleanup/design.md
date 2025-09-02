# 代码清理设计文档

## 概述

本设计文档描述了如何安全地清理Isaac Sim机器人仿真项目中的未使用代码。设计采用分析驱动的方法，通过静态代码分析确定文件依赖关系，然后安全地移动未使用的文件到trash目录。

## 架构

### 核心组件

```
代码清理系统
├── 依赖分析器 (DependencyAnalyzer)
├── 文件移动器 (FileMover)  
├── 清理日志器 (CleanupLogger)
└── 安全检查器 (SafetyChecker)
```

### 数据流

1. **分析阶段**: 扫描所有Python文件，构建依赖图
2. **识别阶段**: 从main.py开始，标记所有被使用的文件
3. **验证阶段**: 双重检查确保不会破坏核心功能
4. **执行阶段**: 安全地移动未使用的文件
5. **记录阶段**: 生成清理报告

## 组件和接口

### DependencyAnalyzer

负责分析代码依赖关系的核心组件。

```python
class DependencyAnalyzer:
    def __init__(self, project_root: str)
    def scan_imports(self, file_path: str) -> List[str]
    def build_dependency_graph(self) -> Dict[str, List[str]]
    def find_used_files(self, entry_points: List[str]) -> Set[str]
```

**关键方法:**
- `scan_imports()`: 使用AST解析Python文件，提取import语句
- `build_dependency_graph()`: 构建文件间的依赖关系图
- `find_used_files()`: 从入口点开始，递归查找所有被使用的文件

### FileMover

负责安全地移动文件到trash目录。

```python
class FileMover:
    def __init__(self, project_root: str, trash_root: str)
    def move_file(self, source_path: str, preserve_structure: bool = True) -> bool
    def create_trash_structure(self, relative_path: str) -> str
    def backup_file(self, file_path: str) -> str
```

**关键特性:**
- 保持目录结构：在trash中重现原有的目录结构
- 冲突处理：如果目标文件已存在，添加时间戳后缀
- 原子操作：确保移动操作的原子性

### SafetyChecker

提供多层安全检查机制。

```python
class SafetyChecker:
    def __init__(self, critical_files: List[str])
    def is_safe_to_move(self, file_path: str) -> bool
    def check_syntax_after_move(self, moved_files: List[str]) -> bool
    def validate_imports(self, entry_point: str) -> bool
```

**安全检查包括:**
- 关键文件保护：永不移动main.py、containers.py等核心文件
- 语法验证：移动后检查剩余代码的语法正确性
- 导入验证：确保主要入口点的导入仍然有效

### CleanupLogger

记录清理操作的详细日志。

```python
class CleanupLogger:
    def __init__(self, log_file: str)
    def log_move(self, source: str, destination: str)
    def log_skip(self, file_path: str, reason: str)
    def generate_summary(self) -> Dict[str, Any]
```

## 数据模型

### 文件分类模型

```python
@dataclass
class FileAnalysis:
    path: str
    is_used: bool
    imported_by: List[str]
    imports: List[str]
    file_type: FileType
    safety_level: SafetyLevel

class FileType(Enum):
    CORE = "core"           # 核心文件，不可移动
    ROBOT = "robot"         # 机器人相关文件
    CONTROLLER = "controller" # 控制器文件
    UTILS = "utils"         # 工具文件
    MAP = "map"            # 地图文件
    CAMERA = "camera"       # 相机文件
    OTHER = "other"        # 其他文件

class SafetyLevel(Enum):
    CRITICAL = "critical"   # 关键文件，绝对不移动
    SAFE = "safe"          # 安全移动
    REVIEW = "review"      # 需要人工审查
```

### 清理配置模型

```python
@dataclass
class CleanupConfig:
    entry_points: List[str] = field(default_factory=lambda: ["main.py"])
    critical_files: List[str] = field(default_factory=lambda: [
        "main.py", "containers.py", "__init__.py"
    ])
    exclude_patterns: List[str] = field(default_factory=lambda: [
        "*/test/*", "*/__pycache__/*", "*/trash/*"
    ])
    trash_directory: str = "trash"
    create_backup: bool = True
    dry_run: bool = False
```

## 错误处理

### 错误分类和处理策略

1. **文件系统错误**
   - 权限不足：记录错误，跳过该文件
   - 磁盘空间不足：停止操作，回滚已移动的文件
   - 文件被占用：等待重试，超时后跳过

2. **依赖分析错误**
   - 语法错误：记录警告，使用正则表达式作为备用分析
   - 循环依赖：记录但不影响清理过程
   - 动态导入：保守处理，标记为可能使用

3. **安全检查失败**
   - 关键文件误标记：立即停止，人工审查
   - 导入验证失败：回滚操作，生成错误报告

### 回滚机制

```python
class RollbackManager:
    def __init__(self)
    def record_move(self, source: str, destination: str)
    def rollback_all(self) -> bool
    def rollback_partial(self, files: List[str]) -> bool
```

## 测试策略

### 单元测试

- **DependencyAnalyzer**: 测试各种import语句的解析
- **FileMover**: 测试文件移动的各种场景
- **SafetyChecker**: 测试安全检查的准确性

### 集成测试

- **端到端清理**: 在测试项目上执行完整清理流程
- **回滚测试**: 验证回滚机制的可靠性
- **安全性测试**: 确保不会误删关键文件

### 测试数据

创建包含以下场景的测试项目：
- 未使用的文件
- 循环依赖
- 动态导入
- 条件导入

## 实现细节

### 依赖分析算法

1. **AST解析**: 使用Python的ast模块解析源代码
2. **导入提取**: 识别from/import语句，处理相对导入
3. **路径解析**: 将导入语句转换为实际文件路径
4. **图构建**: 构建有向图表示文件依赖关系
5. **可达性分析**: 从入口点开始DFS/BFS遍历

### 文件移动策略

1. **预检查**: 验证源文件存在，目标目录可写
2. **结构创建**: 在trash中创建对应的目录结构
3. **原子移动**: 使用os.rename()确保原子性
4. **验证**: 确认移动成功，源文件不存在

### 性能优化

- **缓存**: 缓存AST解析结果，避免重复解析
- **并行处理**: 对独立文件的分析可以并行进行
- **增量分析**: 支持只分析修改过的文件

## 配置和部署

### 配置文件格式

```yaml
# cleanup_config.yaml
entry_points:
  - "main.py"
  - "containers.py"

critical_files:
  - "main.py"
  - "containers.py"
  - "**/__init__.py"

exclude_patterns:
  - "*/test/*"
  - "*/__pycache__/*"
  - "*/trash/*"
  - "*.pyc"

trash_directory: "trash"
create_backup: true
dry_run: false

logging:
  level: "INFO"
  file: "cleanup.log"
```

### 使用方式

```bash
# 干运行模式，只分析不移动
python cleanup_tool.py --dry-run

# 正常清理
python cleanup_tool.py

# 指定配置文件
python cleanup_tool.py --config custom_config.yaml

# 回滚上次操作
python cleanup_tool.py --rollback
```

## 监控和日志

### 日志格式

```
2024-01-15 10:30:00 [INFO] Starting code cleanup analysis
2024-01-15 10:30:01 [INFO] Found 156 Python files
2024-01-15 10:30:02 [INFO] Built dependency graph with 89 nodes
2024-01-15 10:30:03 [INFO] Identified 23 unused files
2024-01-15 10:30:04 [MOVE] robot/robot_cartpole.py -> trash/robot/robot_cartpole.py
2024-01-15 10:30:05 [SKIP] main.py (critical file)
2024-01-15 10:30:06 [INFO] Cleanup completed: 23 files moved, 0 errors
```

### 清理报告

生成包含以下信息的HTML报告：
- 移动文件列表
- 跳过文件及原因
- 依赖关系图可视化
- 清理前后的项目统计

这个设计确保了代码清理过程的安全性、可追溯性和可恢复性，同时提供了灵活的配置选项来适应不同的项目需求。