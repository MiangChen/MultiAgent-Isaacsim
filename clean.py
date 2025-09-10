import gc
import sys
import importlib
from pathlib import Path

# 假设您的容器和World实例可以这样获取
from containers import reset_container, get_container
from isaacsim.core.api import World


def deep_clean_pycache(root_directory: str = "."):
    """
    递归地查找并删除指定目录下的所有 .pyc 文件和 __pycache__ 文件夹。
    这是一个底层的、文件系统级别的清理。

    Args:
        root_directory (str): 开始搜索的根目录。
    """
    print(f"🧹 Starting deep clean of .pyc files and __pycache__ directories in '{root_directory}'...")
    count = 0
    root_path = Path(root_directory)
    for path in root_path.rglob('*.pyc'):
        path.unlink()
        count += 1
    print(f"  - Deleted {count} .pyc files.")

    count = 0
    for path in root_path.rglob('__pycache__'):
        if path.is_dir():
            try:
                # 递归删除文件夹及其内容
                import shutil
                shutil.rmtree(path)
                count += 1
            except OSError as e:
                print(f"  - Error removing directory {path}: {e}")
    print(f"  - Removed {count} __pycache__ directories.")


def full_environment_reset(modules_to_reload: list = None):
    """
    执行一个多层次的环境重置，用于交互式开发。
    警告：这是一个强大的工具，请谨慎使用。

    Args:
        modules_to_reload (list): 一个包含需要被强制重新加载的模块名的字符串列表。
                                  例如: ["robot.swarm_manager", "main"]
    """
    print("\n" + "=" * 50)
    print("🚀 EXECUTING FULL ENVIRONMENT RESET 🚀")
    print("=" * 50)

    # 层次 1: 重置 Isaac Sim 的 World 和 Scene
    print("\n[Level 1] Resetting Isaac Sim World and Scene...")
    world = World.instance()
    if world and world.is_playing():
        world.stop()
    if world:
        # world.clear() 是最彻底的清理，它会移除所有已注册的对象
        world.clear()
        print("  - World cleared of all registered objects.")

    # 层次 2: 重置您自己的应用程序容器
    print("\n[Level 2] Resetting application dependency container...")
    reset_container()
    print("  - Dependency container has been reset.")

    # 层次 3: 强制重新加载指定的Python模块
    if modules_to_reload:
        print("\n[Level 3] Force-reloading specified Python modules...")
        for module_name in modules_to_reload:
            if module_name in sys.modules:
                try:
                    importlib.reload(sys.modules[module_name])
                    print(f"  - Successfully reloaded module: {module_name}")
                except Exception as e:
                    print(f"  - FAILED to reload module {module_name}: {e}")
            else:
                print(f"  - Module {module_name} not in sys.modules, skipping reload.")

    # 层次 4: 强制进行垃圾回收
    print("\n[Level 4] Forcing garbage collection...")
    collected_count = gc.collect()
    print(f"  - Garbage collector removed {collected_count} objects.")

    # 层次 5: (可选，最彻底) 清理文件缓存
    # 警告：这会删除.pyc文件，下次导入会稍慢。
    # print("\n[Level 5] Cleaning filesystem bytecode cache...")
    # deep_clean_pycache()

    print("\n✅ Full environment reset complete. Ready for a clean run.")
    print("=" * 50 + "\n")


modules_i_often_change = [
    "robot.swarm_manager",
    "robot.robot_jetbot",
    "robot.robot_base",
    "main",
    "containers"
]

# full_environment_reset(modules_to_reload=modules_i_often_change)