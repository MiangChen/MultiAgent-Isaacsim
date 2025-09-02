# Isaac Sim WebManager 参数解析器

## 概述

为了改善代码组织和可维护性，我们将 `main.py` 中的参数解析逻辑移动到了独立的 `argument_parser.py` 模块中。

## 文件结构

```
├── argument_parser.py          # 参数解析器模块
├── main.py                     # 主程序（已简化）
├── start_isaac_sim.py          # 简化的启动脚本
├── test_argument_parser.py     # 参数解析器测试
└── ARGUMENT_PARSER_README.md   # 本文档
```

## 主要改进

### 1. 代码组织
- **分离关注点**: 参数解析逻辑与主程序逻辑分离
- **模块化**: 参数解析器可以独立测试和维护
- **可重用性**: 其他脚本可以重用参数解析逻辑

### 2. 参数分组
参数按功能分组，提高可读性：

- **Simulation Configuration**: 仿真相关配置
- **WebManager Configuration**: WebManager 相关配置
- **Logging Configuration**: 日志配置
- **Process Management**: 进程管理
- **Performance Optimization**: 性能优化

### 3. 参数验证
- 自动验证参数组合的有效性
- 检查端口范围、数值范围等
- 提供清晰的错误消息

### 4. 配置摘要
- 自动生成配置摘要用于日志记录
- 便于调试和问题排查

## 使用方法

### 基本用法

```bash
# 使用默认配置启动
python3 main.py

# 启用 WebManager
python3 main.py --enable-webmanager

# 自定义配置
python3 main.py --enable-webmanager --web-port 9090 --data-collection-rate 15.0
```

### 使用启动脚本

```bash
# 查看配置选项
python3 start_isaac_sim.py

# 启动 WebManager
python3 start_isaac_sim.py --enable-webmanager

# 低影响模式（推荐用于 Isaac Sim）
python3 start_isaac_sim.py --enable-webmanager --webmanager-low-impact

# 高性能模式
python3 start_isaac_sim.py --enable-webmanager --webmanager-high-performance
```

### 查看帮助

```bash
python3 main.py --help
python3 start_isaac_sim.py --help
```

## 参数说明

### 核心配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--config` | `./files/sim_cfg.yaml` | 仿真配置文件路径 |
| `--enable-webmanager` | `False` | 启用 WebManager |
| `--web-host` | `0.0.0.0` | WebManager 服务器地址 |
| `--web-port` | `8080` | WebManager 服务器端口 |

### 性能优化

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--data-collection-rate` | `5.0` | 数据收集频率 (Hz) |
| `--webmanager-low-impact` | - | 低影响模式 |
| `--webmanager-high-performance` | - | 高性能模式 |

### 日志配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--log-level` | `INFO` | 日志级别 |
| `--log-file` | `isaac_sim_webmanager.log` | 日志文件路径 |

## 性能模式

### 低影响模式 (推荐)
```bash
python3 main.py --enable-webmanager --webmanager-low-impact --data-collection-rate 5.0
```
- 最小化对 Isaac Sim 性能的影响
- 适合开发和调试

### 高性能模式
```bash
python3 main.py --enable-webmanager --webmanager-high-performance --data-collection-rate 20.0
```
- 最大化 WebManager 性能
- 可能影响 Isaac Sim 性能

## 测试

运行参数解析器测试：

```bash
python3 test_argument_parser.py
```

测试包括：
- 基本参数解析
- 性能模式配置
- 参数验证
- 帮助输出生成

## 开发指南

### 添加新参数

1. 在 `argument_parser.py` 中的相应函数中添加参数
2. 在 `_validate_arguments()` 中添加验证逻辑（如需要）
3. 在 `get_argument_summary()` 中添加摘要显示（如需要）
4. 更新测试文件

### 参数分组

参数按功能分组到不同的函数中：
- `_add_simulation_args()`: 仿真相关
- `_add_webmanager_args()`: WebManager 相关
- `_add_logging_args()`: 日志相关
- `_add_process_management_args()`: 进程管理
- `_add_performance_args()`: 性能优化

### 验证规则

在 `_validate_arguments()` 函数中添加新的验证规则：

```python
def _validate_arguments(args):
    # 检查参数组合
    if args.param1 and args.param2:
        raise ValueError("Cannot specify both param1 and param2")
    
    # 检查数值范围
    if not (1 <= args.port <= 65535):
        raise ValueError(f"Port must be between 1 and 65535")
```

## 迁移指南

### 从旧版本迁移

如果你有使用旧版本 `main.py` 的脚本：

1. **直接使用**: 大部分参数保持不变，可以直接使用
2. **检查新参数**: 查看是否有新的性能优化参数可以使用
3. **更新脚本**: 如果需要程序化访问参数，使用新的 `argument_parser` 模块

### 兼容性

- 所有现有的命令行参数都保持兼容
- 参数的默认值保持不变
- 行为保持一致

## 故障排除

### 常见问题

1. **参数冲突**
   ```
   ValueError: Cannot specify both --enable-webmanager and --disable-webmanager
   ```
   解决：只使用其中一个参数

2. **端口范围错误**
   ```
   ValueError: Web port must be between 1 and 65535
   ```
   解决：使用有效的端口号

3. **导入错误**
   ```
   ImportError: No module named 'argument_parser'
   ```
   解决：确保 `argument_parser.py` 在同一目录中

### 调试

1. **查看配置摘要**:
   ```bash
   python3 main.py --enable-webmanager 2>&1 | head -20
   ```

2. **测试参数解析**:
   ```bash
   python3 test_argument_parser.py
   ```

3. **验证参数**:
   ```python
   from argument_parser import parse_arguments
   args = parse_arguments()
   print(vars(args))
   ```

## 总结

新的参数解析器结构提供了：

- ✅ 更好的代码组织
- ✅ 更清晰的参数分组
- ✅ 自动参数验证
- ✅ 详细的配置摘要
- ✅ 更好的可测试性
- ✅ 向后兼容性

这使得 Isaac Sim WebManager 更容易配置、调试和维护。