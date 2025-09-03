#!/usr/bin/env python3
"""
WebManager Resource Monitor

Real-time monitoring and automatic optimization of WebManager resource usage.
"""

import time
import threading
import logging
import psutil
import os
from typing import Dict, Any, Optional, Callable
from dataclasses import dataclass
from datetime import datetime, timedelta

logger = logging.getLogger(__name__)


@dataclass
class ResourceMetrics:
    """Resource usage metrics."""
    timestamp: float
    memory_mb: float
    cpu_percent: float
    connections: int
    collection_rate: float
    chart_rate: float
    camera_active: bool


class ResourceMonitor:
    """Monitors WebManager resource usage and provides optimization recommendations."""
    
    def __init__(self, webmanager_system=None, check_interval: float = 10.0):
        """
        Initialize resource monitor.
        
        Args:
            webmanager_system: WebManager system to monitor
            check_interval: Monitoring interval in seconds
        """
        self.webmanager_system = webmanager_system
        self.check_interval = check_interval
        self.monitoring = False
        self.monitor_thread: Optional[threading.Thread] = None
        
        # Resource history
        self.metrics_history = []
        self.max_history = 100
        
        # Thresholds
        self.memory_warning_mb = 100
        self.memory_critical_mb = 200
        self.cpu_warning_percent = 15
        self.cpu_critical_percent = 25
        
        # Callbacks
        self.warning_callbacks: list[Callable] = []
        self.critical_callbacks: list[Callable] = []
        
        # Auto-optimization
        self.auto_optimize_enabled = True
        self.last_optimization = 0
        self.optimization_cooldown = 60  # 1 minute between optimizations
        
    def start_monitoring(self):
        """Start resource monitoring in background thread."""
        if self.monitoring:
            logger.warning("Resource monitoring already running")
            return
        
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        logger.info(f"Resource monitoring started (interval: {self.check_interval}s)")
    
    def stop_monitoring(self):
        """Stop resource monitoring."""
        if not self.monitoring:
            return
        
        self.monitoring = False
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=5)
        logger.info("Resource monitoring stopped")
    
    def _monitor_loop(self):
        """Main monitoring loop."""
        while self.monitoring:
            try:
                # Collect metrics
                metrics = self._collect_metrics()
                if metrics:
                    self.metrics_history.append(metrics)
                    
                    # Trim history
                    if len(self.metrics_history) > self.max_history:
                        self.metrics_history.pop(0)
                    
                    # Check thresholds
                    self._check_thresholds(metrics)
                    
                    # Auto-optimize if enabled
                    if self.auto_optimize_enabled:
                        self._auto_optimize(metrics)
                
                # Sleep until next check
                time.sleep(self.check_interval)
                
            except Exception as e:
                logger.error(f"Error in resource monitoring loop: {e}")
                time.sleep(self.check_interval)
    
    def _collect_metrics(self) -> Optional[ResourceMetrics]:
        """Collect current resource metrics."""
        try:
            # System metrics
            process = psutil.Process(os.getpid())
            memory_mb = process.memory_info().rss / 1024 / 1024
            cpu_percent = process.cpu_percent()
            
            # WebManager metrics
            connections = 0
            collection_rate = 0
            chart_rate = 0
            camera_active = False
            
            if self.webmanager_system:
                # Web server metrics
                if hasattr(self.webmanager_system, 'web_server'):
                    ws = self.webmanager_system.web_server
                    connections = len(ws.websocket_manager.active_connections)
                
                # Data collector metrics
                if hasattr(self.webmanager_system, 'data_collector'):
                    dc = self.webmanager_system.data_collector
                    collection_rate = dc.collection_rate
                    chart_rate = getattr(dc, 'chart_update_rate', 0)
                
                # Camera metrics
                if hasattr(self.webmanager_system, 'camera_data_source'):
                    camera_source = self.webmanager_system.camera_data_source
                    camera_active = any(camera_source.active_cameras.values()) if hasattr(camera_source, 'active_cameras') else False
            
            return ResourceMetrics(
                timestamp=time.time(),
                memory_mb=memory_mb,
                cpu_percent=cpu_percent,
                connections=connections,
                collection_rate=collection_rate,
                chart_rate=chart_rate,
                camera_active=camera_active
            )
            
        except Exception as e:
            logger.error(f"Error collecting metrics: {e}")
            return None
    
    def _check_thresholds(self, metrics: ResourceMetrics):
        """Check if metrics exceed warning/critical thresholds."""
        warnings = []
        criticals = []
        
        # Memory checks
        if metrics.memory_mb > self.memory_critical_mb:
            criticals.append(f"Memory usage critical: {metrics.memory_mb:.1f}MB")
        elif metrics.memory_mb > self.memory_warning_mb:
            warnings.append(f"Memory usage high: {metrics.memory_mb:.1f}MB")
        
        # CPU checks
        if metrics.cpu_percent > self.cpu_critical_percent:
            criticals.append(f"CPU usage critical: {metrics.cpu_percent:.1f}%")
        elif metrics.cpu_percent > self.cpu_warning_percent:
            warnings.append(f"CPU usage high: {metrics.cpu_percent:.1f}%")
        
        # Trigger callbacks
        if warnings:
            for warning in warnings:
                logger.warning(f"Resource warning: {warning}")
            for callback in self.warning_callbacks:
                try:
                    callback(warnings, metrics)
                except Exception as e:
                    logger.error(f"Error in warning callback: {e}")
        
        if criticals:
            for critical in criticals:
                logger.error(f"Resource critical: {critical}")
            for callback in self.critical_callbacks:
                try:
                    callback(criticals, metrics)
                except Exception as e:
                    logger.error(f"Error in critical callback: {e}")
    
    def _auto_optimize(self, metrics: ResourceMetrics):
        """Automatically optimize based on current metrics."""
        current_time = time.time()
        
        # Check cooldown
        if current_time - self.last_optimization < self.optimization_cooldown:
            return
        
        # Determine if optimization is needed
        needs_optimization = (
            metrics.memory_mb > self.memory_warning_mb or
            metrics.cpu_percent > self.cpu_warning_percent
        )
        
        if needs_optimization and self.webmanager_system:
            try:
                from webmanager_resource_optimization import WebManagerResourceOptimizer, OptimizationLevel
                
                optimizer = WebManagerResourceOptimizer()
                
                # Determine optimization level based on severity
                if metrics.memory_mb > self.memory_critical_mb or metrics.cpu_percent > self.cpu_critical_percent:
                    level = OptimizationLevel.MINIMAL
                elif metrics.memory_mb > self.memory_warning_mb * 1.5 or metrics.cpu_percent > self.cpu_warning_percent * 1.5:
                    level = OptimizationLevel.LOW
                else:
                    level = OptimizationLevel.BALANCED
                
                logger.info(f"Auto-optimizing to {level.value} level due to resource usage")
                optimizer.apply_optimization(self.webmanager_system, level)
                self.last_optimization = current_time
                
            except Exception as e:
                logger.error(f"Error in auto-optimization: {e}")
    
    def get_current_metrics(self) -> Optional[ResourceMetrics]:
        """Get current resource metrics."""
        return self._collect_metrics()
    
    def get_metrics_history(self, minutes: int = 10) -> list[ResourceMetrics]:
        """Get metrics history for specified time period."""
        cutoff_time = time.time() - (minutes * 60)
        return [m for m in self.metrics_history if m.timestamp >= cutoff_time]
    
    def get_resource_summary(self) -> Dict[str, Any]:
        """Get resource usage summary."""
        current = self.get_current_metrics()
        if not current:
            return {'error': 'Unable to collect metrics'}
        
        # Calculate averages from recent history
        recent_metrics = self.get_metrics_history(5)  # Last 5 minutes
        
        if recent_metrics:
            avg_memory = sum(m.memory_mb for m in recent_metrics) / len(recent_metrics)
            avg_cpu = sum(m.cpu_percent for m in recent_metrics) / len(recent_metrics)
            max_memory = max(m.memory_mb for m in recent_metrics)
            max_cpu = max(m.cpu_percent for m in recent_metrics)
        else:
            avg_memory = current.memory_mb
            avg_cpu = current.cpu_percent
            max_memory = current.memory_mb
            max_cpu = current.cpu_percent
        
        return {
            'current': {
                'memory_mb': round(current.memory_mb, 1),
                'cpu_percent': round(current.cpu_percent, 1),
                'connections': current.connections,
                'collection_rate': current.collection_rate,
                'chart_rate': current.chart_rate,
                'camera_active': current.camera_active
            },
            'averages_5min': {
                'memory_mb': round(avg_memory, 1),
                'cpu_percent': round(avg_cpu, 1)
            },
            'peaks_5min': {
                'memory_mb': round(max_memory, 1),
                'cpu_percent': round(max_cpu, 1)
            },
            'thresholds': {
                'memory_warning_mb': self.memory_warning_mb,
                'memory_critical_mb': self.memory_critical_mb,
                'cpu_warning_percent': self.cpu_warning_percent,
                'cpu_critical_percent': self.cpu_critical_percent
            },
            'status': self._get_status_level(current),
            'monitoring': self.monitoring,
            'auto_optimize': self.auto_optimize_enabled
        }
    
    def _get_status_level(self, metrics: ResourceMetrics) -> str:
        """Get overall status level based on metrics."""
        if (metrics.memory_mb > self.memory_critical_mb or 
            metrics.cpu_percent > self.cpu_critical_percent):
            return 'critical'
        elif (metrics.memory_mb > self.memory_warning_mb or 
              metrics.cpu_percent > self.cpu_warning_percent):
            return 'warning'
        else:
            return 'normal'
    
    def add_warning_callback(self, callback: Callable):
        """Add callback for warning threshold breaches."""
        self.warning_callbacks.append(callback)
    
    def add_critical_callback(self, callback: Callable):
        """Add callback for critical threshold breaches."""
        self.critical_callbacks.append(callback)
    
    def set_thresholds(self, memory_warning_mb: int = None, memory_critical_mb: int = None,
                      cpu_warning_percent: float = None, cpu_critical_percent: float = None):
        """Set resource usage thresholds."""
        if memory_warning_mb is not None:
            self.memory_warning_mb = memory_warning_mb
        if memory_critical_mb is not None:
            self.memory_critical_mb = memory_critical_mb
        if cpu_warning_percent is not None:
            self.cpu_warning_percent = cpu_warning_percent
        if cpu_critical_percent is not None:
            self.cpu_critical_percent = cpu_critical_percent
        
        logger.info(f"Updated thresholds: Memory {self.memory_warning_mb}/{self.memory_critical_mb}MB, "
                   f"CPU {self.cpu_warning_percent}/{self.cpu_critical_percent}%")
    
    def enable_auto_optimization(self):
        """Enable automatic optimization."""
        self.auto_optimize_enabled = True
        logger.info("Auto-optimization enabled")
    
    def disable_auto_optimization(self):
        """Disable automatic optimization."""
        self.auto_optimize_enabled = False
        logger.info("Auto-optimization disabled")


def create_resource_monitor_for_webmanager(webmanager_system, 
                                         memory_limit_mb: int = 100,
                                         cpu_limit_percent: float = 15) -> ResourceMonitor:
    """Create and configure resource monitor for WebManager."""
    
    monitor = ResourceMonitor(webmanager_system, check_interval=10.0)
    
    # Set conservative thresholds
    monitor.set_thresholds(
        memory_warning_mb=memory_limit_mb * 0.8,  # 80% of limit
        memory_critical_mb=memory_limit_mb,
        cpu_warning_percent=cpu_limit_percent * 0.8,  # 80% of limit
        cpu_critical_percent=cpu_limit_percent
    )
    
    # Add logging callbacks
    def warning_callback(warnings, metrics):
        logger.warning(f"Resource usage warnings: {', '.join(warnings)}")
    
    def critical_callback(criticals, metrics):
        logger.error(f"Critical resource usage: {', '.join(criticals)}")
        logger.error(f"Current usage: {metrics.memory_mb:.1f}MB memory, {metrics.cpu_percent:.1f}% CPU")
    
    monitor.add_warning_callback(warning_callback)
    monitor.add_critical_callback(critical_callback)
    
    return monitor


if __name__ == '__main__':
    # Test the resource monitor
    import sys
    
    # Setup logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    
    print("WebManager Resource Monitor Test")
    print("=" * 40)
    
    # Create a mock WebManager system for testing
    class MockWebManagerSystem:
        def __init__(self):
            self.web_server = type('WebServer', (), {
                'websocket_manager': type('WSManager', (), {
                    'active_connections': []
                })()
            })()
            self.data_collector = type('DataCollector', (), {
                'collection_rate': 2.0,
                'chart_update_rate': 0.5
            })()
    
    mock_system = MockWebManagerSystem()
    monitor = create_resource_monitor_for_webmanager(mock_system)
    
    # Get current metrics
    metrics = monitor.get_current_metrics()
    if metrics:
        print(f"Current metrics:")
        print(f"  Memory: {metrics.memory_mb:.1f}MB")
        print(f"  CPU: {metrics.cpu_percent:.1f}%")
        print(f"  Connections: {metrics.connections}")
    
    # Get summary
    summary = monitor.get_resource_summary()
    print(f"\nResource summary:")
    for key, value in summary.items():
        print(f"  {key}: {value}")
    
    print(f"\nResource monitor ready. Use Ctrl+C to exit.")
    
    # Start monitoring for a short test
    try:
        monitor.start_monitoring()
        time.sleep(30)  # Monitor for 30 seconds
    except KeyboardInterrupt:
        print("\nStopping monitor...")
    finally:
        monitor.stop_monitoring()
        print("Monitor stopped.")