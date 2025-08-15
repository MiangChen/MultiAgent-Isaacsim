import os
import yaml

from isaacsim import SimulationApp
import omni


def initialize_simulation_app_from_yaml(config_path):
    """
    Initialize SimulationApp with settings from a YAML file.

    Args:
        config_path (str): Path to the YAML configuration file.
    """
    # 1. 读取YAML配置文件
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # 2. 获取SimulationApp的构造函数配置
    # 'headless' 设置现在直接从 YAML 文件中读取
    sim_app_config = config.get("simulation_app_config", {})
    is_headless = sim_app_config.get("headless", False)  # 安全地获取headless值，默认为False

    print(f"Initializing Isaac Sim with headless={is_headless}")

    # 3. 拼接 experience 路径
    experience = config.get("experience", "")
    # 确保 EXP_PATH 环境变量已设置
    exp_path = os.environ.get("EXP_PATH")
    if not exp_path:
        raise ValueError("EXP_PATH environment variable is not set. Please set it to your Isaac Sim experience path.")

    full_experience_path = f'{exp_path}/{experience}'

    # --- 新增逻辑：从YAML读取并启用扩展 ---
    # extensions_to_enable = config.get("enabled_extensions", [])
    # if extensions_to_enable:
    #     ext_manager = omni.kit.app.get_app().get_extension_manager()
    #     for ext_name in extensions_to_enable:
    #         print(f"Enabling extension: {ext_name}")
    #         ext_manager.set_extension_enabled_immediate(ext_name, True)

    # 4. 初始化 SimulationApp
    simulation_app = SimulationApp(sim_app_config, experience=full_experience_path)

    # 5. 循环设置所有其他渲染和物理参数
    if "settings" in config and config["settings"]:
        for key, value in config["settings"].items():
            simulation_app.set_setting(key, value)
            print(f"Set setting: {key} = {value}")

    # 6. 根据从配置中读取的 headless 状态来决定是否禁用视口
    if is_headless:
        import omni.kit.viewport.utility
        try:
            viewport = omni.kit.viewport.utility.get_active_viewport()
            if viewport:
                viewport.updates_enabled = False
                print("Viewport updates disabled for headless mode.")
        except Exception as e:
            print(f"Could not disable viewport: {e}")

    return simulation_app
