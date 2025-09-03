# WebManager 资源优化指南

## 问题概述

WebManager 在默认配置下可能会占用较多系统资源，影响 Isaac Sim 的性能。本指南提供了多种优化策略来减少资源占用。

## 🚀 快速解决方案

### 1. 使用轻量级启动脚本（推荐）

```bash
# 最简单的方法 - 使用预配置的轻量级模式
python3 start_webmanager_lightweight.py

# 带资源监控的启动
python3 start_webmanager_lightweight.py --with-monitoring

# 查看所有优化级别
python3 start_webmanager_lightweight.py --show-levels
```

### 2. 手动优化启动参数

```bash
# 低资源占用配置
python3 main.py --enable-webmanager \
    --data-collection-rate 2.0 \
    --max-history 200 \
    --disable-camera-streaming \
    --camera-quality 60 \
    --enable-compression \
    --webmanager-low-impact \
    --log-level WARNING
```

## 📊 优化级别对比

| 级别 | 数据收集 | 图表更新 | 历史记录 | 相机流 | 内存限制 | CPU限制 | 适用场景 |
|------|----------|----------|----------|--------|----------|---------|----------|
| **MINIMAL** | 1.0Hz | 0.2Hz | 50 | 禁用 | 50MB | 5% | 低端系统 |
| **LOW** | 1.5Hz | 0.3Hz | 100 | 禁用 | 75MB | 7.5% | 一般系统 |
| **BALANCED** | 2.0Hz | 0.5Hz | 200 | 禁用 | 100MB | 10% | 推荐配置 |
| **FULL** | 5.0Hz | 1.0Hz | 500 | 启用 | 200MB | 20% | 高端系统 |

## 🔧 详细优化策略

### 1. 数据收集优化

**问题**: 默认 10Hz 的数据收集频率过高

**解决方案**:
```bash
# 降低数据收集频率
--data-collection-rate 2.0  # 从 10Hz 降到 2Hz

# 减少历史数据存储
--max-history 200  # 从 1000 降到 200
```

**效果**: 减少 60-80% 的数据处理开销

### 2. 图表生成优化

**问题**: 实时图表生成消耗大量 CPU

**解决方案**:
- 降低图表更新频率到 0.5Hz
- 减少轨迹点数量
- 在无客户端连接时禁用图表生成

**代码示例**:

```python
from UI.webmanager_resource_optimization import WebManagerResourceOptimizer, OptimizationLevel

optimizer = WebManagerResourceOptimizer()
optimizer.apply_optimization(webmanager_system, OptimizationLevel.LOW)
```

### 3. 相机流优化

**问题**: 相机帧处理和编码占用大量资源

**解决方案**:
```bash
# 完全禁用相机流（推荐）
--disable-camera-streaming

# 或者降低质量和分辨率
--camera-quality 60  # 从 80 降到 60
```

**效果**: 可减少 40-60% 的 CPU 使用

### 4. WebSocket 优化

**问题**: 频繁的 WebSocket 广播消耗网络和 CPU

**解决方案**:
```bash
# 启用数据压缩
--enable-compression

# 限制连接数（在代码中配置）
max_connections = 5
```

### 5. 内存优化

**问题**: 大量历史数据占用内存

**解决方案**:
- 减少历史数据存储量
- 启用定期清理
- 使用更高效的数据结构

## 📈 资源监控

### 实时监控

```python
from UI.webmanager_resource_monitor import create_resource_monitor_for_webmanager

# 创建资源监控器
monitor = create_resource_monitor_for_webmanager(
    webmanager_system,
    memory_limit_mb=100,
    cpu_limit_percent=15
)

# 启动监控
monitor.start_monitoring()

# 获取资源使用报告
summary = monitor.get_resource_summary()
print(f"Memory: {summary['current']['memory_mb']}MB")
print(f"CPU: {summary['current']['cpu_percent']}%")
```

### 自动优化

```python
# 启用自动优化
monitor.enable_auto_optimization()

# 系统会根据资源使用情况自动调整优化级别
```

## 🎯 针对不同场景的推荐配置

### 开发调试场景
```bash
python3 start_webmanager_lightweight.py
```
- 低资源占用
- 保留核心功能
- 适合日常开发

### 演示展示场景
```bash
python3 main.py --enable-webmanager \
    --data-collection-rate 3.0 \
    --max-history 300 \
    --camera-quality 70 \
    --enable-compression
```
- 平衡性能和功能
- 适度的视觉效果
- 稳定的性能

### 生产环境场景
```bash
python3 main.py --enable-webmanager \
    --data-collection-rate 1.0 \
    --max-history 100 \
    --disable-camera-streaming \
    --log-level ERROR
```
- 最小资源占用
- 只保留必要功能
- 高稳定性

## 🔍 故障排除

### 问题 1: WebManager 启动后 Isaac Sim 变慢

**原因**: 数据收集频率过高或相机流占用资源

**解决方案**:
```bash
# 立即使用最小配置重启
python3 start_webmanager_lightweight.py

# 或者手动设置最低参数
python3 main.py --enable-webmanager \
    --data-collection-rate 1.0 \
    --disable-camera-streaming \
    --webmanager-low-impact
```

### 问题 2: 内存使用持续增长

**原因**: 历史数据积累或内存泄漏

**解决方案**:
1. 减少历史数据存储: `--max-history 100`
2. 启用资源监控检查泄漏
3. 定期重启 WebManager

### 问题 3: CPU 使用率过高

**原因**: 图表生成或数据处理频率过高

**解决方案**:
1. 降低数据收集频率: `--data-collection-rate 1.0`
2. 禁用图表生成（在代码中）
3. 使用 MINIMAL 优化级别

### 问题 4: 网络延迟高

**原因**: WebSocket 数据传输量大

**解决方案**:
1. 启用压缩: `--enable-compression`
2. 减少数据更新频率
3. 限制客户端连接数

## 📋 资源使用检查清单

### 启动前检查
- [ ] 选择合适的优化级别
- [ ] 确认是否需要相机流
- [ ] 设置合理的数据收集频率
- [ ] 启用压缩和低影响模式

### 运行时监控
- [ ] 监控内存使用 (< 100MB)
- [ ] 监控 CPU 使用 (< 15%)
- [ ] 检查 Isaac Sim 性能影响
- [ ] 观察网络连接数

### 优化调整
- [ ] 根据实际需求调整参数
- [ ] 启用自动优化
- [ ] 定期检查资源使用趋势
- [ ] 记录最佳配置参数

## 🛠️ 高级优化技巧

### 1. 条件性功能启用

```python
# 只在有客户端连接时启用图表生成
if webmanager.web_server.websocket_manager.is_connected():
    webmanager.data_collector.enable_chart_generation()
else:
    webmanager.data_collector.disable_chart_generation()
```

### 2. 动态频率调整

```python
# 根据系统负载动态调整收集频率
current_cpu = psutil.cpu_percent()
if current_cpu > 20:
    webmanager.data_collector.collection_rate = 1.0
elif current_cpu < 10:
    webmanager.data_collector.collection_rate = 3.0
```

### 3. 智能缓存策略

```python
# 只在数据变化时更新缓存
if has_significant_change(new_data, cached_data):
    update_cache(new_data)
    broadcast_update(new_data)
```

## 📞 获取帮助

如果遇到资源使用问题：

1. **查看日志**: 检查 `isaac_sim_webmanager.log` 中的警告信息
2. **运行诊断**: 使用 `webmanager_resource_monitor.py` 进行资源分析
3. **尝试不同配置**: 从 MINIMAL 级别开始逐步提升
4. **监控系统**: 使用系统监控工具观察整体资源使用

## 📈 性能基准

### 典型资源使用（BALANCED 配置）
- **内存**: 80-120MB
- **CPU**: 5-15%
- **网络**: 1-5 Mbps（取决于客户端数量）
- **磁盘**: 最小（主要是日志文件）

### 优化后资源使用（LOW 配置）
- **内存**: 40-80MB
- **CPU**: 3-8%
- **网络**: 0.5-2 Mbps
- **磁盘**: 最小

通过合理的配置和优化，WebManager 可以在保持核心功能的同时，将资源占用降低到最小，确保 Isaac Sim 的流畅运行。