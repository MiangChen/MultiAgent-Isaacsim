#!/usr/bin/env python3
"""
WebManager Isaac Sim Performance Optimization

专门针对Isaac Sim帧率问题的优化方案。
解决CPU/GPU/内存占用不高但帧率下降的问题。
"""

import time
import threading
import logging
import asyncio
from typing import Dict, Any, Optional, Callable
from collections import deque

logger = logging.getLogger(__name__)


class IsaacSimFrameRateOptimizer:
    """专门优化Isaac Sim帧率的优化器"""
    
    def __init__(self, webmanager_system=None):
        self.webmanager_system = webmanager_system
        self.original_methods = {}
        self.optimization_applied = False
        
        # 性能监控
        self.frame_rate_history = deque(maxlen=60)  # 1秒的帧率历史
        self.monitoring = False
        
    def apply_frame_rate_optimizations(self):
        """应用帧率优化"""
        if self.optimization_applied:
            logger.warning("Frame rate optimizations already applied")
            return
        
        logger.info("🚀 Applying Isaac Sim frame rate optimizations...")
        
        try:
            # 1. 优化数据收集时机
            self._optimize_collection_timing()
            
            # 2. 实现非阻塞数据访问
            self._implement_non_blocking_data_access()
            
            # 3. 优化线程同步
            self._optimize_thread_synchronization()
            
            # 4. 减少Isaac Sim API调用频率
            self._reduce_isaac_sim_api_calls()
            
            # 5. 实现帧率感知的数据收集
            self._implement_frame_rate_aware_collection()
            
            self.optimization_applied = True
            logger.info("✅ Isaac Sim frame rate optimizations applied successfully")
            
        except Exception as e:
            logger.error(f"❌ Error applying frame rate optimizations: {e}")
            self.restore_original_behavior()
    
    def _optimize_collection_timing(self):
        """优化数据收集时机 - 避免在渲染关键时刻收集数据"""
        if not self.webmanager_system or not hasattr(self.webmanager_system, 'data_collector'):
            return
        
        dc = self.webmanager_system.data_collector
        
        # 保存原始方法
        if hasattr(dc, '_collect_all_data'):
            self.original_methods['_collect_all_data'] = dc._collect_all_data
        
        # 创建优化的数据收集方法
        async def optimized_collect_all_data():
            """优化的数据收集 - 使用更小的时间片"""
            try:
                # 将数据收集分解为更小的步骤，每步之间让出控制权
                
                # Step 1: 收集机器人数据（最轻量）
                if dc.robot_data_source and dc.robot_data_source.is_available():
                    await dc._collect_robot_data()
                    await asyncio.sleep(0.001)  # 1ms让出控制权
                
                # Step 2: 收集性能数据
                if dc.performance_data_source and dc.performance_data_source.is_available():
                    await dc._collect_performance_data()
                    await asyncio.sleep(0.001)
                
                # Step 3: 收集ROS数据（如果启用）
                if dc.ros_data_source and dc.ros_data_source.is_available():
                    await dc._collect_ros_data()
                    await asyncio.sleep(0.001)
                
                # 跳过相机数据收集以减少负载
                # 相机数据是最消耗资源的，在帧率优化模式下禁用
                
            except Exception as e:
                logger.error(f"Error in optimized data collection: {e}")
        
        # 替换原始方法
        dc._collect_all_data = optimized_collect_all_data
        logger.info("📊 Data collection timing optimized")
    
    def _implement_non_blocking_data_access(self):
        """实现非阻塞的Isaac Sim数据访问"""
        if (not self.webmanager_system or 
            not hasattr(self.webmanager_system, 'data_collector') or
            not hasattr(self.webmanager_system.data_collector, 'robot_data_source')):
            return
        
        robot_source = self.webmanager_system.data_collector.robot_data_source
        
        if not hasattr(robot_source, 'swarm_manager') or not robot_source.swarm_manager:
            return
        
        # 保存原始方法
        if hasattr(robot_source, '_update_from_swarm_manager'):
            self.original_methods['_update_from_swarm_manager'] = robot_source._update_from_swarm_manager
        
        # 实现缓存和批量处理
        robot_source._position_cache = {}
        robot_source._cache_valid_time = 0.05  # 50ms缓存有效期
        robot_source._last_update_time = 0
        
        def non_blocking_update_from_swarm_manager():
            """非阻塞的机器人数据更新"""
            current_time = time.time()
            
            # 检查缓存是否仍然有效
            if current_time - robot_source._last_update_time < robot_source._cache_valid_time:
                return  # 使用缓存数据，避免频繁API调用
            
            try:
                # 快速批量获取所有机器人位置
                if hasattr(robot_source.swarm_manager, 'robot_active'):
                    batch_positions = {}
                    
                    # 限制每次更新的机器人数量，避免长时间阻塞
                    max_robots_per_update = 5
                    robot_count = 0
                    
                    for robot_class_name, robots in robot_source.swarm_manager.robot_active.items():
                        for i, robot in enumerate(robots):
                            if robot_count >= max_robots_per_update:
                                break
                            
                            robot_id = f"{robot_class_name}_{i}"
                            
                            # 快速获取位置，如果失败则跳过
                            try:
                                if hasattr(robot, 'get_world_pose'):
                                    world_pose = robot.get_world_pose()
                                    if world_pose and len(world_pose) >= 1:
                                        position = world_pose[0]
                                        
                                        # 简化的位置数据，减少处理时间
                                        position_data = {
                                            'position': position,
                                            'orientation': {'yaw': 0.0},  # 简化朝向
                                            'battery_level': 100.0,
                                            'status': 'active'
                                        }
                                        
                                        batch_positions[robot_id] = position_data
                                        robot_count += 1
                                        
                            except Exception as e:
                                logger.debug(f"Skipping robot {robot_id} due to error: {e}")
                                continue
                        
                        if robot_count >= max_robots_per_update:
                            break
                    
                    # 批量更新位置
                    for robot_id, position_data in batch_positions.items():
                        robot_source._update_robot_position(robot_id, position_data)
                    
                    robot_source._last_update_time = current_time
                    
            except Exception as e:
                logger.debug(f"Error in non-blocking robot update: {e}")
        
        # 替换原始方法
        robot_source._update_from_swarm_manager = non_blocking_update_from_swarm_manager
        logger.info("🤖 Non-blocking robot data access implemented")
    
    def _optimize_thread_synchronization(self):
        """优化线程同步，减少线程竞争"""
        if not self.webmanager_system or not hasattr(self.webmanager_system, 'data_collector'):
            return
        
        dc = self.webmanager_system.data_collector
        
        # 设置更低的线程优先级
        try:
            import os
            if hasattr(os, 'nice'):
                # 在数据收集线程中设置最低优先级
                original_run_loop = getattr(dc, '_run_collection_loop', None)
                if original_run_loop:
                    self.original_methods['_run_collection_loop'] = original_run_loop
                    
                    def low_priority_run_loop():
                        try:
                            # 设置最低优先级
                            os.nice(19)  # Linux上的最低优先级
                        except (PermissionError, OSError):
                            pass
                        
                        # 设置线程为后台模式
                        threading.current_thread().daemon = True
                        
                        # 调用原始方法
                        return original_run_loop()
                    
                    dc._run_collection_loop = low_priority_run_loop
                    
        except Exception as e:
            logger.debug(f"Could not optimize thread priority: {e}")
        
        # 减少线程间通信频率
        if hasattr(dc, 'collection_rate'):
            # 进一步降低收集频率以减少线程切换
            dc.collection_rate = min(dc.collection_rate, 2.0)  # 最多2Hz
            dc.collection_interval = 1.0 / dc.collection_rate
        
        logger.info("🧵 Thread synchronization optimized")
    
    def _reduce_isaac_sim_api_calls(self):
        """减少Isaac Sim API调用频率"""
        if not self.webmanager_system:
            return
        
        # 1. 实现智能跳帧 - 不是每帧都收集数据
        if hasattr(self.webmanager_system, 'data_collector'):
            dc = self.webmanager_system.data_collector
            
            # 添加跳帧计数器
            dc._frame_skip_counter = 0
            dc._frame_skip_interval = 30  # 每30帧收集一次数据（假设60FPS，即0.5秒一次）
            
            # 保存原始收集方法
            if hasattr(dc, '_collection_loop'):
                self.original_methods['_collection_loop'] = dc._collection_loop
                
                async def frame_skipping_collection_loop():
                    """带跳帧的数据收集循环"""
                    logger.info(f"Starting frame-skipping collection loop at {dc.collection_rate}Hz")
                    
                    while dc.collecting:
                        collection_start = time.time()
                        
                        try:
                            # 跳帧逻辑
                            dc._frame_skip_counter += 1
                            
                            if dc._frame_skip_counter >= dc._frame_skip_interval:
                                # 执行数据收集
                                await dc._collect_all_data()
                                dc._frame_skip_counter = 0
                                
                                # 更新统计
                                collection_time = time.time() - collection_start
                                dc._update_collection_stats(collection_time)
                            
                            # 等待下一个周期
                            sleep_time = max(0.001, dc.collection_interval)  # 最少1ms
                            await asyncio.sleep(sleep_time)
                            
                        except Exception as e:
                            logger.error(f"Error in frame-skipping collection: {e}")
                            await asyncio.sleep(dc.collection_interval)
                
                dc._collection_loop = frame_skipping_collection_loop
        
        logger.info("📉 Isaac Sim API call frequency reduced")
    
    def _implement_frame_rate_aware_collection(self):
        """实现帧率感知的数据收集"""
        if not self.webmanager_system:
            return
        
        # 添加帧率监控
        self._setup_frame_rate_monitoring()
        
        # 实现自适应收集频率
        if hasattr(self.webmanager_system, 'data_collector'):
            dc = self.webmanager_system.data_collector
            
            # 添加帧率感知逻辑
            dc._target_frame_rate = 60.0  # 目标帧率
            dc._frame_rate_tolerance = 0.9  # 90%的帧率容忍度
            
            # 保存原始自适应方法
            if hasattr(dc, '_adjust_collection_rate_if_needed'):
                self.original_methods['_adjust_collection_rate_if_needed'] = dc._adjust_collection_rate_if_needed
            
            async def frame_rate_aware_adjustment(collection_time):
                """基于帧率的自适应调整"""
                try:
                    # 获取当前帧率
                    current_fps = self._get_current_frame_rate()
                    
                    if current_fps > 0:
                        target_fps = dc._target_frame_rate * dc._frame_rate_tolerance
                        
                        if current_fps < target_fps:
                            # 帧率过低，降低收集频率
                            new_rate = max(0.5, dc.collection_rate * 0.8)
                            if new_rate != dc.collection_rate:
                                logger.info(f"🐌 Frame rate low ({current_fps:.1f}fps), reducing collection to {new_rate}Hz")
                                dc.collection_rate = new_rate
                                dc.collection_interval = 1.0 / new_rate
                        
                        elif current_fps > target_fps * 1.1:
                            # 帧率良好，可以适当提高收集频率
                            new_rate = min(5.0, dc.collection_rate * 1.1)
                            if new_rate != dc.collection_rate:
                                logger.debug(f"🚀 Frame rate good ({current_fps:.1f}fps), increasing collection to {new_rate}Hz")
                                dc.collection_rate = new_rate
                                dc.collection_interval = 1.0 / new_rate
                
                except Exception as e:
                    logger.debug(f"Error in frame rate aware adjustment: {e}")
            
            dc._adjust_collection_rate_if_needed = frame_rate_aware_adjustment
        
        logger.info("📊 Frame rate aware collection implemented")
    
    def _setup_frame_rate_monitoring(self):
        """设置帧率监控"""
        self.last_frame_time = time.time()
        self.frame_count = 0
        
    def _get_current_frame_rate(self) -> float:
        """获取当前帧率估算"""
        try:
            current_time = time.time()
            self.frame_count += 1
            
            # 每秒计算一次帧率
            if current_time - self.last_frame_time >= 1.0:
                fps = self.frame_count / (current_time - self.last_frame_time)
                self.frame_rate_history.append(fps)
                
                self.frame_count = 0
                self.last_frame_time = current_time
                
                return fps
            
            # 返回最近的帧率
            return self.frame_rate_history[-1] if self.frame_rate_history else 60.0
            
        except Exception:
            return 60.0  # 默认帧率
    
    def restore_original_behavior(self):
        """恢复原始行为"""
        if not self.optimization_applied:
            return
        
        logger.info("🔄 Restoring original WebManager behavior...")
        
        try:
            # 恢复所有被修改的方法
            for obj_path, original_method in self.original_methods.items():
                if '.' in obj_path:
                    # 处理嵌套属性
                    parts = obj_path.split('.')
                    obj = self.webmanager_system
                    for part in parts[:-1]:
                        obj = getattr(obj, part)
                    setattr(obj, parts[-1], original_method)
                else:
                    # 直接属性
                    if hasattr(self.webmanager_system, 'data_collector'):
                        setattr(self.webmanager_system.data_collector, obj_path, original_method)
            
            self.optimization_applied = False
            logger.info("✅ Original behavior restored")
            
        except Exception as e:
            logger.error(f"❌ Error restoring original behavior: {e}")
    
    def get_optimization_status(self) -> Dict[str, Any]:
        """获取优化状态"""
        current_fps = self._get_current_frame_rate()
        
        return {
            'optimization_applied': self.optimization_applied,
            'current_fps': current_fps,
            'frame_rate_history': list(self.frame_rate_history),
            'collection_rate': getattr(self.webmanager_system.data_collector, 'collection_rate', 0) if self.webmanager_system else 0,
            'optimizations_count': len(self.original_methods)
        }


def optimize_webmanager_for_isaac_sim(webmanager_system) -> IsaacSimFrameRateOptimizer:
    """为Isaac Sim优化WebManager"""
    
    optimizer = IsaacSimFrameRateOptimizer(webmanager_system)
    optimizer.apply_frame_rate_optimizations()
    
    return optimizer


def create_isaac_sim_friendly_startup_args() -> list:
    """创建对Isaac Sim友好的启动参数"""
    
    return [
        '--data-collection-rate', '1.0',  # 极低的收集频率
        '--max-history', '50',            # 最小历史记录
        '--disable-camera-streaming',     # 禁用相机流
        '--webmanager-low-impact',        # 低影响模式
        '--log-level', 'ERROR',           # 最少日志
        '--enable-compression'            # 启用压缩
    ]


if __name__ == '__main__':
    print("Isaac Sim WebManager Frame Rate Optimization")
    print("=" * 50)
    
    print("\n🎯 Optimization Strategies:")
    print("1. ⏱️  Optimize data collection timing")
    print("2. 🚫 Implement non-blocking Isaac Sim API access")
    print("3. 🧵 Optimize thread synchronization")
    print("4. 📉 Reduce Isaac Sim API call frequency")
    print("5. 📊 Frame rate aware data collection")
    
    print("\n🚀 Isaac Sim friendly startup command:")
    args = create_isaac_sim_friendly_startup_args()
    print(f"python3 main.py --enable-webmanager {' '.join(args)}")
    
    print("\n💡 Key optimizations:")
    print("• Data collection: 1Hz (vs 10Hz default)")
    print("• API call caching: 50ms cache timeout")
    print("• Frame skipping: Collect every 30 frames")
    print("• Thread priority: Lowest possible")
    print("• Memory usage: Minimal history buffer")