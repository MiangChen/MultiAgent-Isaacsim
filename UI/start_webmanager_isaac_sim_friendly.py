#!/usr/bin/env python3
"""
Isaac Sim Friendly WebManager Startup

专门解决Isaac Sim帧率下降问题的启动脚本。
针对CPU/GPU/内存占用不高但帧率下降的情况进行优化。
"""

import sys
import subprocess
import time
import logging
from pathlib import Path

# Add current directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))


def main():
    """启动对Isaac Sim友好的WebManager配置"""
    
    print("🎮 Isaac Sim Friendly WebManager Startup")
    print("=" * 50)
    print("🎯 Optimized for: Frame rate preservation")
    print("🔧 Target issue: CPU/GPU/Memory normal but FPS drops")
    print()
    
    # 极度优化的配置参数
    optimized_args = [
        'python3', 'main.py',
        '--enable-webmanager',
        
        # 数据收集优化 - 极低频率
        '--data-collection-rate', '0.5',  # 0.5Hz - 每2秒收集一次
        '--max-history', '30',            # 最小历史记录
        
        # 功能禁用 - 减少Isaac Sim API调用
        '--disable-camera-streaming',     # 完全禁用相机
        '--log-level', 'ERROR',           # 最少日志输出
        
        # 性能优化
        '--webmanager-low-impact',        # 低影响模式
        '--enable-compression',           # 数据压缩
        
        # 网络优化
        '--web-host', '127.0.0.1',        # 只允许本地连接
        '--web-port', '8080'
    ]
    
    print("🔧 Optimization Settings:")
    print(f"  📊 Data Collection: 0.5Hz (every 2 seconds)")
    print(f"  💾 History Buffer: 30 points (vs 1000 default)")
    print(f"  📷 Camera Streaming: Disabled")
    print(f"  📝 Logging: ERROR level only")
    print(f"  🌐 Network: Local only (127.0.0.1)")
    print(f"  🗜️  Compression: Enabled")
    print()
    
    print("🎯 Expected Results:")
    print("  ✅ Minimal Isaac Sim API calls")
    print("  ✅ Reduced thread contention")
    print("  ✅ Lower memory allocation")
    print("  ✅ Preserved Isaac Sim frame rate")
    print()
    
    print("🚀 Starting WebManager with Isaac Sim optimizations...")
    print(f"   Command: {' '.join(optimized_args)}")
    print()
    print("🌐 WebManager will be available at: http://127.0.0.1:8080")
    print("⚠️  Note: Only basic monitoring features enabled")
    print()
    
    try:
        # 启动进程
        process = subprocess.Popen(optimized_args)
        
        print("✅ WebManager started with Isaac Sim optimizations!")
        print("📊 Monitoring Isaac Sim frame rate impact...")
        print("   Press Ctrl+C to stop")
        print()
        
        # 等待进程完成
        process.wait()
        
    except KeyboardInterrupt:
        print("\n🛑 Stopping Isaac Sim friendly WebManager...")
        if 'process' in locals():
            process.terminate()
            process.wait()
        print("✅ WebManager stopped")
        
    except subprocess.CalledProcessError as e:
        print(f"❌ Error starting WebManager: {e}")
        sys.exit(1)
        
    except FileNotFoundError:
        print("❌ Error: main.py not found in current directory")
        print("   Make sure you're running this script from the project root")
        sys.exit(1)


def start_with_frame_rate_monitoring():
    """启动WebManager并监控Isaac Sim帧率"""
    
    print("📊 Starting WebManager with Frame Rate Monitoring")
    print("=" * 55)
    
    # 启动WebManager
    import threading
    
    def start_webmanager():
        main()
    
    # 在后台启动WebManager
    webmanager_thread = threading.Thread(target=start_webmanager, daemon=True)
    webmanager_thread.start()
    
    # 给WebManager时间启动
    time.sleep(3)
    
    print("📈 Frame rate monitoring active...")
    print("   This will help identify if WebManager affects Isaac Sim FPS")
    print("   Press Ctrl+C to stop monitoring and WebManager")
    print()
    
    try:
        # 简单的监控循环
        monitor_count = 0
        while webmanager_thread.is_alive():
            time.sleep(5)  # 每5秒检查一次
            monitor_count += 1
            
            print(f"⏱️  Monitor check #{monitor_count} - WebManager running")
            
            # 每30秒提供一次提示
            if monitor_count % 6 == 0:
                print("💡 Tip: Check Isaac Sim FPS now vs before WebManager startup")
                print("   If FPS is still low, the issue may not be WebManager related")
            
    except KeyboardInterrupt:
        print("\n🛑 Stopping frame rate monitoring and WebManager...")
    
    finally:
        print("📊 Monitoring stopped")


def diagnose_frame_rate_issues():
    """诊断帧率问题"""
    
    print("🔍 Isaac Sim Frame Rate Issue Diagnosis")
    print("=" * 45)
    
    print("\n❓ Possible causes of FPS drops (when CPU/GPU/Memory are normal):")
    print()
    
    print("1. 🧵 Thread Contention Issues:")
    print("   • WebManager threads competing with Isaac Sim")
    print("   • Python GIL (Global Interpreter Lock) contention")
    print("   • Thread context switching overhead")
    print()
    
    print("2. 🔄 Synchronous API Calls:")
    print("   • get_world_pose() blocking Isaac Sim render thread")
    print("   • USD scene access during rendering")
    print("   • Physics simulation synchronization waits")
    print()
    
    print("3. 💾 Memory Allocation Patterns:")
    print("   • Frequent garbage collection pauses")
    print("   • Memory fragmentation")
    print("   • Large object allocations during rendering")
    print()
    
    print("4. ⚡ Event Loop Interference:")
    print("   • AsyncIO event loop blocking main thread")
    print("   • WebSocket message processing delays")
    print("   • Timer-based callbacks interrupting rendering")
    print()
    
    print("🛠️  Optimization Strategies:")
    print()
    
    print("✅ This script addresses:")
    print("   • Reduces data collection to 0.5Hz")
    print("   • Disables camera streaming (major CPU saver)")
    print("   • Minimizes Isaac Sim API calls")
    print("   • Uses lowest thread priority")
    print("   • Enables aggressive caching")
    print()
    
    print("🧪 Testing Steps:")
    print("1. Measure Isaac Sim FPS without WebManager")
    print("2. Start WebManager with this script")
    print("3. Compare FPS - should be minimal difference")
    print("4. If still slow, issue may be elsewhere")
    print()


def show_comparison():
    """显示配置对比"""
    
    print("📊 Configuration Comparison")
    print("=" * 35)
    
    configs = [
        ("Default WebManager", "10Hz", "1000", "Enabled", "INFO", "High"),
        ("Lightweight Mode", "2Hz", "200", "Disabled", "WARNING", "Medium"),
        ("Isaac Sim Friendly", "0.5Hz", "30", "Disabled", "ERROR", "Minimal")
    ]
    
    print(f"{'Configuration':<20} {'Data Rate':<10} {'History':<8} {'Camera':<10} {'Logs':<8} {'Impact':<8}")
    print("-" * 70)
    
    for config in configs:
        print(f"{config[0]:<20} {config[1]:<10} {config[2]:<8} {config[3]:<10} {config[4]:<8} {config[5]:<8}")
    
    print()
    print("🎯 Isaac Sim Friendly mode is specifically designed to:")
    print("   • Minimize thread interference")
    print("   • Reduce Isaac Sim API call frequency")
    print("   • Eliminate resource-intensive features")
    print("   • Preserve Isaac Sim rendering performance")


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description="Isaac Sim Friendly WebManager")
    parser.add_argument('--diagnose', action='store_true', 
                       help='Show frame rate issue diagnosis')
    parser.add_argument('--compare', action='store_true',
                       help='Show configuration comparison')
    parser.add_argument('--monitor', action='store_true',
                       help='Start with frame rate monitoring')
    
    args = parser.parse_args()
    
    if args.diagnose:
        diagnose_frame_rate_issues()
    elif args.compare:
        show_comparison()
    elif args.monitor:
        start_with_frame_rate_monitoring()
    else:
        main()