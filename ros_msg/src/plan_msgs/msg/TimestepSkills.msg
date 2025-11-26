# TimestepSkills.msg
# 全局的一个时间步对应的多个机器人动作
int32 timestep           # 全局时间步索引
RobotSkill[] robots      # 各机器人在此时间步上的技能列表
