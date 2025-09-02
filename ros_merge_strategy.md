# ROS Integration Merge Strategy

## Current Situation Analysis

You have two versions of main.py:

1. **Current main.py** (main branch): Uses dependency injection architecture with containers, modern Isaac Sim APIs, and semantic camera functionality
2. **with_ros branch**: Has direct ROS integration but uses older architecture without dependency injection

## Key Differences

### Architecture Differences

| Feature | Current main.py | with_ros branch |
|---------|----------------|-----------------|
| Dependency Injection | ✅ Uses containers.py | ❌ Direct imports |
| Isaac Sim Initialization | Modern physics_engine module | Custom function |
| ROS Integration | Optional with error handling | Always enabled |
| Semantic Camera | ✅ Full implementation | ❌ Missing |
| Scene Management | ✅ Advanced scene manager | ❌ Basic |
| Error Handling | ✅ Comprehensive | ❌ Basic |

### ROS Integration Differences

| Component | Current main.py | with_ros branch |
|-----------|----------------|-----------------|
| ROS Initialization | Optional, with fallback | Always required |
| Skill Processing | Dependency injected | Direct access |
| Node Management | Background thread | Background thread |
| Error Recovery | ✅ Graceful degradation | ❌ Hard failure |

## Recommended Merge Strategy

### Option 1: Enhance Current main.py (Recommended)

Keep your current architecture and add the missing ROS features from with_ros branch.

#### Steps:

1. **Verify ROS modules are available** (already done ✅)
2. **Enhance ROS skill processing** (needs improvement)
3. **Add missing ROS message types** (if any)
4. **Test ROS integration** with your current architecture

#### What's Already Working:
- ✅ ROS availability check
- ✅ Background ROS thread
- ✅ Skill queue management
- ✅ Plan message processing
- ✅ Graceful shutdown

#### What Needs Enhancement:
- 🔧 Robot state checking in process_ros_skills
- 🔧 Better error handling for ROS failures
- 🔧 ROS node lifecycle management

### Option 2: Migrate with_ros to Modern Architecture

Take the with_ros branch and add dependency injection + semantic camera features.

#### Pros:
- Proven ROS integration
- Direct control over ROS lifecycle

#### Cons:
- Lose modern architecture benefits
- Lose semantic camera functionality
- More work to modernize

## Implementation Plan for Option 1

### Phase 1: Verify Current ROS Integration

```bash
# Test current ROS functionality
python main.py --ros=True

# Check if ROS nodes are created
ros2 node list

# Check if topics are available
ros2 topic list
```

### Phase 2: Enhance ROS Skill Processing

The current `process_ros_skills` function needs to be more robust:

```python
def process_ros_skills(env, swarm_manager: SwarmManager) -> None:
    """Enhanced ROS skill processing with better error handling"""
    if not ROS_AVAILABLE:
        return

    try:
        # Check if all robots have completed their current skills
        state_skill_complete_all = True
        for robot_class in swarm_manager.robot_class:
            for robot in swarm_manager.robot_active[robot_class]:
                # More robust state checking
                if hasattr(robot, "state_skill_complete"):
                    done = bool(robot.state_skill_complete)
                else:
                    # Default to True if attribute doesn't exist
                    done = True
                state_skill_complete_all = state_skill_complete_all and done

        if state_skill_complete_all:
            # Process next skills for each robot
            with _skill_lock:
                keys = list(_skill_queues.keys())

            for rc, rid in keys:
                # Validate robot exists
                if (rc not in swarm_manager.robot_active or 
                    rid >= len(swarm_manager.robot_active[rc])):
                    logger.warning(f"Robot {rc}_{rid} not found, skipping skill")
                    continue

                with _skill_lock:
                    dq = _skill_queues.get((rc, rid))
                    next_skill = dq.popleft() if (dq and len(dq) > 0) else None
                    if dq is not None and len(dq) == 0:
                        _skill_queues.pop((rc, rid), None)

                if next_skill is not None:
                    name = next_skill.skill.strip().lower()
                    params = _param_dict(next_skill.params)
                    fn = _SKILL_TABLE.get(name)
                    
                    if fn is None:
                        logger.warning(f"[Scheduler] unsupported skill: {name}")
                    else:
                        try:
                            logger.info(f"Executing skill {name} for robot {rc}_{rid}")
                            fn(env, rc, rid, params)
                        except Exception as e:
                            logger.error(f"[Scheduler] skill execution error: {e}")

    except Exception as e:
        logger.error(f"Error in process_ros_skills: {e}")
```

### Phase 3: Add Missing ROS Components

Check if you need additional ROS message types or services from the with_ros branch:

```python
# Add any missing imports from with_ros branch
try:
    from plan_msgs.msg import Parameter, SkillInfo, RobotSkill, Plan as PlanMsg, TimestepSkills
    # Add any other message types that might be missing
except ImportError as e:
    logger.warning(f"Some ROS message types not available: {e}")
```

### Phase 4: Testing Strategy

1. **Unit Tests**: Test ROS integration without full simulation
2. **Integration Tests**: Test with actual robots and ROS nodes
3. **Fallback Tests**: Ensure system works without ROS

## Code Changes Needed

### 1. Enhance Robot State Checking

```python
# In your robot classes, ensure state_skill_complete is properly managed
class RobotBase:
    def __init__(self):
        self.state_skill_complete = True
        
    def execute_skill(self, skill_name, params):
        self.state_skill_complete = False
        # Execute skill
        # Set to True when complete
        self.state_skill_complete = True
```

### 2. Improve ROS Node Management

```python
def build_ros_nodes() -> tuple:
    """Enhanced ROS node building with better error handling"""
    if not ROS_AVAILABLE:
        return None, None

    try:
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        plan_receiver = BaseNode('plan_receiver')

        def _plan_cb(msg: PlanMsg):
            """Enhanced plan callback with validation"""
            try:
                if not msg.steps:
                    logger.warning("Received empty plan")
                    return
                    
                steps = sorted(msg.steps, key=lambda s: s.timestep)
                logger.info(f"Received plan with {len(steps)} timesteps")

                with _skill_lock:
                    for ts in steps:
                        for rs in ts.robots:
                            rc, rid = _parse_robot_id(rs.robot_id)
                            q = _skill_queues[(rc, rid)]

                            for sk in rs.skill_list:
                                q.append(sk)
                                logger.debug(f"Queued skill {sk.skill} for robot {rc}_{rid}")

            except Exception as e:
                logger.error(f"[PlanCB] Error processing plan: {e}")

        plan_receiver.create_subscription(PlanMsg, '/Plan', _plan_cb, qos)
        scene_monitor = SceneMonitorNode()
        
        logger.info("ROS nodes created successfully")
        return plan_receiver, scene_monitor
        
    except Exception as e:
        logger.error(f"Failed to create ROS nodes: {e}")
        return None, None
```

## Migration Commands

```bash
# 1. Create a backup of current main.py
cp main.py main_current_backup.py

# 2. Check differences between branches
git diff main with_ros -- main.py > ros_differences.patch

# 3. Selectively apply changes (manual process)
# Review the patch file and apply relevant changes

# 4. Test the merged version
python main.py --ros=True

# 5. If issues occur, you can always revert
cp main_current_backup.py main.py
```

## Conclusion

**Recommendation**: Stick with your current main.py architecture and enhance the ROS integration. Your current code has:

- ✅ Better architecture (dependency injection)
- ✅ Semantic camera functionality
- ✅ Modern Isaac Sim APIs
- ✅ Better error handling
- ✅ More maintainable code structure

The ROS integration in your current main.py is already quite comprehensive. You mainly need to:

1. **Test** the existing ROS functionality
2. **Enhance** robot state management
3. **Add** any missing ROS message types
4. **Improve** error handling in ROS skill processing

This approach will give you the best of both worlds: modern architecture + robust ROS integration.