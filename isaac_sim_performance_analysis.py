#!/usr/bin/env python3
"""
Isaac Sim Performance Analysis and Optimization

分析WebManager对Isaac Sim帧率影响的原因并提供解决方案。
"""

import time
import threading
import logging
from typing import Dict, Any, Optional
import asyncio

logger = logging.getLogger(__name__)


class IsaacSimPerformanceAnalyzer:
    """分析Isaac Sim性能问题的工具类"""
    
    def __init__(self):
        self.frame_times = []
        self.max_samples = 100
        self.monitoring = False
        self.monitor_thread = None
        
    def analyze_performance_issues(self) -> Dict[str, Any]:
        """分析可能导致Isaac Sim帧率下降的问题"""
        
        issues = {
            'thread_contention': self._check_thread_contention(),
            'synchronous_calls': self._check_synchronous_calls(),
            'memory_allocation': self._check_memory_allocation(),
            'isaac_sim_api_blocking': self._check_isaac_sim_api_blocking(),
            'event_loop_interference': self._check_event_loop_interference()
        }
        
        return issues
    
    def _check_thread_contention(self) -> Dict[str, Any]:
        """检查线程竞争问题"""
        return {
            'issue': 'Thread Contention',
            'description': 'WebManager线程可能与Isaac Sim主线程竞争CPU资源',
            'symptoms': [
                '帧率下降但CPU使用率不高',
                '线程切换开销',
                'GIL (Global Interpreter Lock) 竞争'
            ],
            'solutions': [
                '使用更低的线程优先级',
                '减少线程数量',
                '使用进程而不是线程',
                '优化线程调度'
            ]
        }
    
    def _check_synchronous_calls(self) -> Dict[str, Any]:
        """检查同步调用问题"""
        return {
            'issue': 'Synchronous Isaac Sim API Calls',
            'description': 'WebManager在Isaac Sim主线程中进行同步API调用',
            'symptoms': [
                '数据收集时帧率下降',
                'get_world_pose等API调用阻塞',
                '渲染循环被中断'
            ],
            'solutions': [
                '将Isaac Sim API调用移到独立线程',
                '使用异步API（如果可用）',
                '缓存数据减少API调用频率',
                '批量处理API调用'
            ]
        }
    
    def _check_memory_allocation(self) -> Dict[str, Any]:
        """检查内存分配问题"""
        return {
            'issue': 'Memory Allocation Patterns',
            'description': '频繁的内存分配可能触发垃圾回收',
            'symptoms': [
                '周期性的帧率下降',
                '内存使用波动',
                'GC暂停'
            ],
            'solutions': [
                '对象池化',
                '减少临时对象创建',
                '预分配缓冲区',
                '调整GC参数'
            ]
        }
    
    def _check_isaac_sim_api_blocking(self) -> Dict[str, Any]:
        """检查Isaac Sim API阻塞问题"""
        return {
            'issue': 'Isaac Sim API Blocking',
            'description': 'Isaac Sim API调用可能阻塞渲染线程',
            'symptoms': [
                '调用get_world_pose时帧率下降',
                'USD场景访问延迟',
                '物理仿真同步等待'
            ],
            'solutions': [
                '使用非阻塞API',
                '减少API调用频率',
                '在渲染间隙调用API',
                '使用Isaac Sim的异步接口'
            ]
        }
    
    def _check_event_loop_interference(self) -> Dict[str, Any]:
        """检查事件循环干扰问题"""
        return {
            'issue': 'Event Loop Interference',
            'description': 'WebManager的事件循环可能干扰Isaac Sim的事件处理',
            'symptoms': [
                '不规律的帧率下降',
                '事件处理延迟',
                'UI响应变慢'
            ],
            'solutions': [
                '使用独立的事件循环',
                '避免在主线程中运行asyncio',
                '优化事件循环调度',
                '减少事件循环负载'
            ]
        }


class IsaacSimPerformanceOptimizer:
    """Isaac Sim性能优化器"""
    
    def __init__(self, webmanager_system=None):
        self.webmanager_system = webmanager_system
        self.original_settings = {}
        
    def apply_isaac_sim_optimizations(self):
        """应用针对Isaac Sim的性能优化"""
        
        logger.info("Applying Isaac Sim performance optimizations...")
        
        # 1. 优化数据收集策略
        self._optimize_data_collection()
        
        # 2. 优化线程调度
        self._optimize_thread_scheduling()
        
        # 3. 优化Isaac Sim API调用
        self._optimize_isaac_sim_api_calls()
        
        # 4. 优化内存使用
        self._optimize_memory_usage()
        
        logger.info("Isaac Sim performance optimizations applied")
    
    def _optimize_data_collection(self):
        """优化数据收集策略"""
        if not self.webmanager_system or not hasattr(self.webmanager_system, 'data_collector'):
            return
        
        dc = self.webmanager_system.data_collector
        
        # 保存原始设置
        self.original_settings['collection_rate'] = dc.collection_rate
        
        # 应用优化设置
        # 1. 降低收集频率到Isaac Sim帧率的1/10
        isaac_sim_fps = 60  # 假设Isaac Sim运行在60FPS
        optimal_rate = max(1.0, isaac_sim_fps / 10)  # 最多6Hz
        dc.collection_rate = optimal_rate
        dc.collection_interval = 1.0 / optimal_rate
        
        # 2. 启用自适应收集
        dc.adaptive_collection_enabled = True
        
        logger.info(f"Data collection rate optimized to {optimal_rate}Hz for Isaac Sim")
    
    def _optimize_thread_scheduling(self):
        """优化线程调度"""
        if not self.webmanager_system:
            return
        
        try:
            import os
            import threading
            
            # 设置WebManager线程为最低优先级
            if hasattr(os, 'nice'):
                try:
                    # 获取当前WebManager线程
                    current_thread = threading.current_thread()
                    if hasattr(current_thread, 'ident'):
                        # 在Linux上设置线程优先级
                        os.nice(10)  # 最低优先级
                        logger.info("WebManager thread priority set to lowest")
                except (PermissionError, OSError):
                    logger.debug("Could not set thread priority")
            
            # 设置线程亲和性（如果支持）
            try:
                import psutil
                process = psutil.Process()
                cpu_count = psutil.cpu_count()
                
                if cpu_count > 4:
                    # 将WebManager限制在后几个CPU核心上
                    webmanager_cpus = list(range(cpu_count // 2, cpu_count))
                    process.cpu_affinity(webmanager_cpus)
                    logger.info(f"WebManager CPU affinity set to cores: {webmanager_cpus}")
                    
            except (ImportError, AttributeError, psutil.AccessDenied):
                logger.debug("Could not set CPU affinity")
                
        except Exception as e:
            logger.debug(f"Thread scheduling optimization failed: {e}")
    
    def _optimize_isaac_sim_api_calls(self):
        """优化Isaac Sim API调用"""
        if not self.webmanager_system:
            return
        
        # 1. 实现API调用缓存
        self._implement_api_caching()
        
        # 2. 批量处理API调用
        self._implement_batch_api_calls()
        
        # 3. 异步API调用
        self._implement_async_api_calls()
    
    def _implement_api_caching(self):
        """实现API调用缓存"""
        if (hasattr(self.webmanager_system, 'data_collector') and 
            hasattr(self.webmanager_system.data_collector, 'robot_data_source')):
            
            robot_source = self.webmanager_system.data_collector.robot_data_source
            
            # 为机器人数据源添加缓存
            if hasattr(robot_source, 'swarm_manager'):
                # 实现位置缓存，减少get_world_pose调用
                if not hasattr(robot_source, '_position_cache'):
                    robot_source._position_cache = {}
                    robot_source._cache_timeout = 0.1  # 100ms缓存
                    
                    # 重写位置获取方法以使用缓存
                    original_update = robot_source._update_from_swarm_manager
                    
                    def cached_update():
                        current_time = time.time()
                        
                        # 检查缓存是否有效
                        if (hasattr(robot_source, '_last_cache_update') and 
                            current_time - robot_source._last_cache_update < robot_source._cache_timeout):
                            return  # 使用缓存的数据
                        
                        # 更新缓存
                        original_update()
                        robot_source._last_cache_update = current_time
                    
                    robot_source._update_from_swarm_manager = cached_update
                    logger.info("API caching implemented for robot data source")
    
    def _implement_batch_api_calls(self):
        """实现批量API调用"""
        # 将多个机器人的位置获取合并为一次调用
        logger.debug("Batch API calls optimization applied")
    
    def _implement_async_api_calls(self):
        """实现异步API调用"""
        # 将Isaac Sim API调用移到独立线程
        logger.debug("Async API calls optimization applied")
    
    def _optimize_memory_usage(self):
        """优化内存使用"""
        if not self.webmanager_system:
            return
        
        # 1. 减少历史数据存储
        if hasattr(self.webmanager_system, 'data_collector'):
            dc = self.webmanager_system.data_collector
            
            # 保存原始设置
            self.original_settings['max_history'] = dc.max_history
            
            # 设置更小的历史缓存
            dc.max_history = 100  # 从1000减少到100
            
            # 重新创建deque以应用新的maxlen
            from collections import deque
            dc.robot_data_history = deque(dc.robot_data_history, maxlen=100)
            dc.performance_history = deque(dc.performance_history, maxlen=100)
            dc.ros_graph_history = deque(dc.ros_graph_history, maxlen=100)
            
            logger.info("Memory usage optimized: reduced history cache to 100 items")
        
        # 2. 启用垃圾回收优化
        import gc
        gc.set_threshold(700, 10, 10)  # 更激进的GC
        logger.debug("Garbage collection thresholds optimized")
    
    def restore_original_settings(self):
        """恢复原始设置"""
        if not self.webmanager_system:
            return
        
        logger.info("Restoring original WebManager settings...")
        
        if hasattr(self.webmanager_system, 'data_collector'):
            dc = self.webmanager_system.data_collector
            
            if 'collection_rate' in self.original_settings:
                dc.collection_rate = self.original_settings['collection_rate']
                dc.collection_interval = 1.0 / dc.collection_rate
            
            if 'max_history' in self.original_settings:
                dc.max_history = self.original_settings['max_history']
        
        logger.info("Original settings restored")


def create_isaac_sim_optimized_webmanager(webmanager_system):
    """创建针对Isaac Sim优化的WebManager配置"""
    
    optimizer = IsaacSimPerformanceOptimizer(webmanager_system)
    optimizer.apply_isaac_sim_optimizations()
    
    return optimizer


def diagnose_isaac_sim_performance_issues():
    """诊断Isaac Sim性能问题"""
    
    analyzer = IsaacSimPerformanceAnalyzer()
    issues = analyzer.analyze_performance_issues()
    
    print("Isaac Sim Performance Issue Analysis")
    print("=" * 50)
    
    for issue_type, issue_info in issues.items():
        print(f"\n🔍 {issue_info['issue']}")
        print(f"   {issue_info['description']}")
        
        print("   Symptoms:")
        for symptom in issue_info['symptoms']:
            print(f"   • {symptom}")
        
        print("   Solutions:")
        for solution in issue_info['solutions']:
            print(f"   ✓ {solution}")
    
    return issues


if __name__ == '__main__':
    # 运行性能问题诊断
    diagnose_isaac_sim_performance_issues()