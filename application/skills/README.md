# Application Skills

## Directory Structure

```
application/skills/
├── base/              # 基础技能 (所有机器人通用)
│   ├── navigate_to.py
│   ├── explore.py
│   ├── detect.py
│   ├── track.py
│   ├── take_photo.py
│   └── object_detection.py
├── drone/             # 无人机技能
│   └── take_off.py
├── manipulation/      # 机械臂操作技能
│   ├── pick_up.py
│   └── put_down.py
└── target/            # 目标机器人技能
    └── move.py
```

## Skills by Category

### Base Skills (基础技能)

**navigate_to** - Navigate to target position
- ROS Required: ✅
- Usage: `navigate_to(goal_pos=[x, y, z], goal_quat_wxyz=[w, x, y, z])`

**explore** - Explore area with boundary
- ROS Required: ✅
- Usage: `explore(boundary=[[x1,y1,z1], [x2,y2,z2], ...])`

**detect** - Detect overlap with target
- ROS Required: ❌
- Usage: `detect(target_prim="/World/Package")`

**track** - Track moving target
- ROS Required: ✅
- Usage: `track(track_period=10, target_topic="/target_0/odom")`

**take_photo** - Take photo with camera
- ROS Required: ❌
- Usage: `take_photo(camera_name="front", save_to_file="/path/to/file.jpg")`

**object_detection** - Detect objects in camera view
- ROS Required: ❌
- Usage: `object_detection(target_class="car")`

### Drone Skills (无人机技能)

**take_off** - Take off to altitude
- ROS Required: ❌
- Usage: `take_off(altitude=2.0)`

### Manipulation Skills (机械臂操作技能)

**pick_up** - Pick up object
- ROS Required: ❌
- Usage: `pick_up(target_prim="/World/Package")`

**put_down** - Put down object
- ROS Required: ❌
- Usage: `put_down(target_pos=[x, y, z])`

### Target Skills (目标机器人技能)

**move** - Move along path
- ROS Required: ✅
- Usage: `move(path=[[x1,y1,z1], [x2,y2,z2], ...])`

## Usage

```python
from application import SkillManager
from application.skills import navigate_to, explore, take_off

# Create skill manager
skill_manager = SkillManager(robot)

# Register skills
skill_manager.register_skill('navigate_to', navigate_to)
skill_manager.register_skill('explore', explore)
skill_manager.register_skill('take_off', take_off)

# Execute skills
result = skill_manager.execute_skill('navigate_to', goal_pos=[10, 10, 0])
result = skill_manager.execute_skill('take_off', altitude=2.0)
```

## ROS Action Interface

All skills can be executed via ROS2 actions:

```bash
# Navigate
ros2 action send_goal /jetbot_0/skill_execution plan_msgs/action/SkillExecution \
  '{skill_request: {skill_list: [{skill: "navigate_to", params: [{key: "goal_pos", value: "[3, 3, 0]"}]}]}}' --feedback

# Take off
ros2 action send_goal /cf2x_0/skill_execution plan_msgs/action/SkillExecution \
  '{skill_request: {skill_list: [{skill: "take_off", params: [{key: "altitude", value: "1.0"}]}]}}' --feedback

# Explore
ros2 action send_goal /jetbot_0/skill_execution plan_msgs/action/SkillExecution \
  "{skill_request: {skill_list: [{skill: explore, params: [{key: boundary, value: '[[-4.4, 12, 0], [3.0, 27.4, 0]]'}]}]}}" --feedback
```
