开发阶段，使用中文来进行记录, 基于中文发布README，最后发布后再提供一个english翻译


## 配置python环境

可以参考该飞书文档安装好python环境, 该文档也包括了安装ubuntu,NVIDIA Driver, CUDA的步骤, 如果已经安装了, 可以跳过这几步:

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

[![Lines of Code](https://img.shields.io/badge/LoC-30998-blue)](https://github.com/MiangChen/multiagent-isaacsim)


cloc --exclude-dir=$(tr '\n' ',' < .clocignore) 
cloc .
