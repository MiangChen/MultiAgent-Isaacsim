# ubuntu的用户名字
name_user = 'ubuntu'
# conda环境的名字
name_conda_env = 'env_isaaclab'
# isaacsim的位置, 这里以通过pip安装在conda中的isaacsim为例子
path_isaacsim = f'/home/{name_user}/anaconda3/envs/{name_conda_env}/lib/python3.10/site-packages/isaacsim'
# isaaclab的位置, 这里以本地项目中git clone下来的isaaclab为例子
path_isaaclab = f'/home/{name_user}/PycharmProjects/multiagent-isaacim/IsaacLab'

# 本地解压后的IsaacSim的资产位置
replacement_string = '/home/ubuntu/isaacsim_assets/'
# 替换的目标是amazon网址
string_to_find = 'https://omniverse-content-production.s3-us-west-2.amazonaws.com/'

file_path_list = [
    f'/home/{name_user}/.local/share/ov/data/Kit/Isaac-Sim Full/4.5/user.config.json',  # 这个文件容易被遗漏
    f'{path_isaaclab}/source/isaaclab/isaaclab/utils/assets.py',
    f'{path_isaacsim}/exts/isaacsim.util.clash_detection/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.robot_setup.assembler/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.asset.gen.conveyor.ui/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.robot.wheeled_robots/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.sensors.physx/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.sensors.camera.ui/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.benchmark.services/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.benchmark.examples/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.sensors.physics/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.gui.components/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.gui.menu/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.ros2.bridge/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.asset.gen.omap/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.sensors.physx.examples/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.examples.interactive/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.ros1.bridge/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.sensors.camera/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.replicator.domain_randomization/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.core.utils/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.sensors.rtx.ui/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.storage.native/docs/index.rst',
    f'{path_isaacsim}/exts/isaacsim.storage.native/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.ros2.tf_viewer/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.robot.wheeled_robots.ui/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.sensors.physics.examples/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.robot_setup.grasp_editor/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.core.nodes/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.core.prims/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.robot.policy.examples/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.asset.browser/cache/isaacsim.asset.browser.cache.json',
    f'{path_isaacsim}/exts/isaacsim.asset.browser/docs/index.rst',
    f'{path_isaacsim}/exts/isaacsim.asset.browser/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.core.cloner/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.replicator.behavior/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.sensors.rtx/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.test.collection/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.robot_motion.motion_generation/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.core.api/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.robot.manipulators.examples/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.replicator.examples/config/extension.toml',
    f'{path_isaacsim}/exts/isaacsim.robot.manipulators/config/extension.toml',
    f'{path_isaacsim}/extsDeprecated/omni.isaac.dynamic_control/config/extension.toml',
    f'{path_isaacsim}/extsDeprecated/omni.replicator.isaac/config/extension.toml',
    f'{path_isaacsim}/extscache/omni.kit.browser.asset-1.3.11/config/extension.toml',
]

for file_path in file_path_list:
    try:
        with open(file_path, 'r', encoding='utf-8') as f_read:
            content = f_read.read()
        print(f"加载文件 {file_path}, 开始处理")
    except Exception as e:
        print(f"错误：文件未找到 - {file_path}\n {e}")
        break

    # 执行替换
    new_content = content.replace(string_to_find, replacement_string)

    # 检查是否有内容被替换
    if new_content != content:
        # 写回修改后的内容
        with open(file_path, 'w', encoding='utf-8') as f_write:
            f_write.write(new_content)
        print(f"文件已成功更新: {file_path}")
    else:
        print(f"在文件中未找到需要替换的字符串: {file_path}")
