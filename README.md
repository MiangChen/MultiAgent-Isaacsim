Contributions are welcome! Please **fork** this repository and submit a **Pull Request**.
If you'd like to become a **collaborator**, please leave a comment in the **[Pinned Issue](https://github.com/MiangChen/MultiAgent-Isaacsim/issues/4#issue-3667665696)**
Let's build a better isaacsim multi-robot open source repository.

欢迎贡献！如果您想参与开发，请 **Fork** 本仓库并提交 **Pull Request**。  
如果您希望成为 **协作者 (Collaborator)**，请直接在 **[置顶 Issue](https://github.com/MiangChen/MultiAgent-Isaacsim/issues/4#issue-3667665696)** 中留言申请
让我们打造一个更好的isaacsim多机器人开源仓库

## System Requirements

| Category | Item | Requirement / Version | Notes |
| :--- | :--- | :--- | :--- |
| **Hardware** | **CPU** | 12 Cores+ | e.g., AMD 9900X, Intel Ultra 9 |
| | **GPU** | 16GB+ VRAM (RT Core req.)| e.g., RTX 4090, RTX 5880 Ada |
| | **RAM** | 64GB+ | |
| | **Storage**| 512GB+ | |
| **Software** | **OS** | Ubuntu 22.04 | |
| | **Driver** | NVIDIA 570.169 | |
| | **CUDA** | 12.2 / 12.4 / 12.6 | |
| | **Python** | 3.10 / 3.11 | 3.10 for Isaac Sim 4.5; 3.11 for Isaac Sim 5.x |
| | **ROS2** | Humble | |
| | **GCC** | 12.x | |
> **Note:** Items marked with a `+` indicate **minimum requirements** (higher specifications are acceptable). Items without a `+` require the **specific version** listed.  
> **注意**：末尾带有 `+` 的项表示**最低配置要求**（可以使用更高配置）；未带 `+` 的项表示必须严格使用**特定版本**。

## 配置Isaacsim 4.5 的驱动环境和安装包

可以参考该飞书文档安装好python环境, 该文档也包括了安装ubuntu,NVIDIA Driver, CUDA的步骤, 如果已经安装了isaacsim, 可以跳过这几步:

[技术文档 - 台式机Ubuntu 22.04 双系统+ NV driver + Isaacsim 4.5/5.0 全保姆流程](https://xetk4h23q3.feishu.cn/wiki/C985wlnk5iDnJakICQrcvw06nZg?from=from_copylink)


## 配置工作环境

参考该文档:

[工作空间配置](https://xetk4h23q3.feishu.cn/wiki/Jmhnw6DAyiEVY5kXrULcpX91nme?from=from_copylink)


## 测试案例

运行run_main.bash, 可以用这个文档中的指令测试: 

[仿真部分 效果演示](https://xetk4h23q3.feishu.cn/wiki/LSzqwurKmiF3lqkEYlocTXtYnke?from=from_copylink)


## 架构理解

参考文档: [架构图](https://xetk4h23q3.feishu.cn/wiki/PHEFws9W2iEnAeky4dJcnN05nCK?from=from_copylink)


## 各种BUG报错

可以查阅该文档是否有提到过: [疑难杂症 BUG](https://xetk4h23q3.feishu.cn/wiki/ATyhwSTrmizJWAkCP60cR6yYn4f?from=from_copylink)

如果没有, 可以发起Issue, 后续也会整合到上述的文档中


***
***

[![Lines of Code](https://img.shields.io/badge/LoC-29421-blue)](https://github.com/MiangChen/multiagent-isaacsim)


cloc --exclude-dir=$(tr '\n' ',' < .clocignore) 

cloc .
