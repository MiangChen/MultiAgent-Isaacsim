安装方法: 
sudo apt update
sudo apt install default-jre
pip install -r ./pddl/requirements.txt

调用solver_p 中的 plan


plan的数据结构如下：


# 🔧 数据结构说明

```json
{
  "step_0": {
    "robot1": {
      "navigate-to": {
        "src": "place1",
        "dst": "depot"
      }
    },
    "robot2": {
      "navigate-to": {
        "src": "place2",
        "dst": "depot"
      }
    }
  },
  "step_1": {
    "robot1": {
      "pick-up": {
        "it1": "item1",
        "loc1": "depot"
      }
    }
  }
}
# 📑 字段说明

| 层级         | 类型   | 描述                                       |
|--------------|--------|--------------------------------------------|
| `step_k`     | `str`  | 时间步编号，允许并行执行的拓扑分层编号     |
| `robotX`     | `str`  | agent 名称，从 `ActionInstance` 中自动提取 |
| `action_name`| `str`  | 动作名称，如 `pick-up`, `navigate-to`      |
| 参数字典     | `dict` | 动作的命名参数及其目标对象，如 `it1`, `loc1` |
