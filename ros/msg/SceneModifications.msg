# PrimTransform.msg

# 变更类型枚举
uint8 ADD=0
uint8 DELETE=1
uint8 TRANSFORM_CHANGED=2

uint8 change_type        # 0=新增, 1=删除, 2=Transform 改变
string prim_path
geometry_msgs/Transform transform
