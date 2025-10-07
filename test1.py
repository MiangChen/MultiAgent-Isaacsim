class Robot:
    def __init__(self, model):
        self.model = model

    def __str__(self):
        return f"我是一个型号为 {self.model} 的机器人。"


# def upgrade_robot(robot_instance):
#     print(f"进入升级程序... 接收到的机器人型号是：{robot_instance.model}")
#     # 直接修改实例的属性
#     robot_instance.model = "T-1000"
#     print(f"升级完成！现在的型号是：{robot_instance.model}")
#
#
# # 创建一个 Robot 实例
# my_robot = Robot("T-800")
# print(f"函数调用前，我的机器人型号是：{my_robot.model}")
# print("-" * 20)

def replace_robot(robot_instance):
    print(f"进入替换程序... 接收到的机器人型号是：{robot_instance.model}")
    # 这里的操作是让 robot_instance 这个名字指向一个全新的 Robot 对象
    # robot_instance = Robot("R2-D2")
    robot_instance2 = robot_instance
    robot_instance2.model = 'T-1000'
    print(f"替换完成！新机器人的型号是：{robot_instance2.model}")

# 创建一个 Robot 实例
my_robot = Robot("T-800")
print(f"函数调用前，我的机器人型号是：{my_robot.model}")
print("-" * 20)

# 将实例传递给替换函数
replace_robot(my_robot)
print("-" * 20)

# 检查原始实例的属性
print(f"函数调用后，我的机器人型号仍然是：{my_robot.model}")
# 将实例传递给函数
# upgrade_robot(my_robot)
# print("-" * 20)

# 检查原始实例的属性
print(f"函数调用后，我的机器人型号是：{my_robot.model}")