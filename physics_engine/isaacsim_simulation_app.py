# =============================================================================
# Isaac Sim Simulation App Module - Simulation Application Initialization
# =============================================================================
#
# This module provides functionality for initializing and configuring the
# Isaac Sim SimulationApp with proper settings and configuration management.
#
# =============================================================================

# Standard library imports
import os

# Third-party library imports
from isaacsim import SimulationApp
import omni
import yaml


def start_isaacsim_simulation_app():
    """
    创建 SimulationApp 实例
    """

    # 1. 读取YAML配置文件
    current_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(current_dir, "../config/config_parameter.yaml")
    normalized_config_path = os.path.normpath(config_path)
    with open(normalized_config_path, "r") as f:
        config = yaml.safe_load(f)

    # 2. 获取SimulationApp的构造函数配置
    sim_app_config = config.get("simulation_app_config", {})

    # 3. 拼接 experience 路径
    experience = config.get("experience", "")
    exp_path = os.environ.get("EXP_PATH")
    if not exp_path:
        raise ValueError(
            "EXP_PATH environment variable is not set. Please set it to your Isaac Sim experience path."
        )
    full_experience_path = f"{exp_path}/{experience}"

    # 4. 初始化 SimulationApp
    simulation_app = SimulationApp(sim_app_config, experience=full_experience_path)

    # 5. 管理拓展
    extensions_to_enable = config.get("enabled_extensions", [])
    if extensions_to_enable:
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        for ext_name in extensions_to_enable:
            print(f"Enabling extension: {ext_name}")
            ext_manager.set_extension_enabled_immediate(ext_name, True)

    # 6. 循环设置所有其他渲染和物理参数
    if "settings" in config and config["settings"]:
        for key, value in config["settings"].items():
            print(f"Set setting: {key} = {value}")
            simulation_app.set_setting(key, value)

    # 7. 更新一次simulation_app, 确保能
    simulation_app.update()
    return simulation_app
