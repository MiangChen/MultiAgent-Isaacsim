int32 index      # 当前正在跟踪的机器人路径点的坐标，应该在0<=index<len（path_list)之间
float64 velocity # 表示机器人前进的速度
float64 omega    # 表示机器人自转的角速度，这里容易弄反正负号
bool complete    # 用于表示之前给出的路径列表是否都跟踪完毕
