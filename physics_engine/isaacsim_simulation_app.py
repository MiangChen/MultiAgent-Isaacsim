import os
import yaml

import argparse

from isaacsim import SimulationApp
import omni


def start_isaacsim_simulation_app(config_path):
    """
    创建 SimulationApp 实例
    """

    # 1. 读取YAML配置文件
    with open(config_path, "r") as f:
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

    # # 7. 根据从配置中读取的 headless 状态来决定是否禁用视口
    # if sim_app_config.get("headless", False):
    #     omni.kit.viewport.utility.get_active_viewport().updates_enabled = False

    return simulation_app

