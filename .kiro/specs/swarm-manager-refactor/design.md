# Design Document

## Overview

This design refactors the swarm manager initialization logic to improve separation of concerns and code organization. The main changes involve:

1. Moving swarm initialization logic from `Env._async_init()` to a new method in `SwarmManager`
2. Simplifying the `Env` class to focus only on environment concerns
3. Updating `main.py` to handle swarm manager initialization directly

## Architecture

### Current Architecture
```
main.py -> Env.create() -> Env._async_init() -> SwarmManager.load_robot_swarm_cfg()
                                            -> SwarmManager.activate_robot()
```

### New Architecture
```
main.py -> SwarmManager.initialize_async() -> SwarmManager.load_robot_swarm_cfg()
                                           -> SwarmManager.activate_robot()
        -> Env.create() -> Env._async_init() (simplified)
```

## Components and Interfaces

### SwarmManager Class Changes

#### New Method: `initialize_async()`
```python
async def initialize_async(self, 
                          scene: Scene,
                          robot_swarm_cfg_path: str = None,
                          robot_active_flag_path: str = None) -> None:
    """
    Complete async initialization of the swarm manager.
    
    Args:
        scene: The Isaac Sim scene object from env.world.scene
        robot_swarm_cfg_path: Path to robot swarm configuration file
        robot_active_flag_path: Path to robot active flag configuration file
    """
```

This method will:
1. Set `self.scene = scene`
2. Call `await self.load_robot_swarm_cfg(robot_swarm_cfg_path)`
3. Call `self.activate_robot(robot_active_flag_path)`

### Env Class Changes

#### Modified Method: `_async_init()`
```python
async def _async_init(self) -> None:
    """
    Simplified async initialization focusing only on environment concerns.
    """
```

This method will only:
1. Set `self._swarm_manager.scene = self.world.scene` (if swarm manager needs scene reference for other operations)

### Main Application Changes

#### Modified Function: `setup_simulation()`
```python
async def setup_simulation(simulation_app) -> Env:
    # ... existing robot registration code ...
    
    # Create environment first
    env = await Env.create(
        simulation_app=simulation_app,
        physics_dt=cfg['world']['physics_dt'],
        swarm_manager=swarm_manager,
        scene_manager=scene_manager,
        grid_map=map_grid,
    )
    
    # Initialize swarm manager after environment is ready
    await swarm_manager.initialize_async(
        scene=env.world.scene,
        robot_swarm_cfg_path=f"{PATH_PROJECT}/files/robot_swarm_cfg.yaml",
        robot_active_flag_path=f"{PATH_PROJECT}/files/robot_swarm_active_flag.yaml"
    )
    
    return env
```

## Data Models

No changes to existing data models are required. The same configuration files and data structures will be used.

## Error Handling

### SwarmManager.initialize_async()
- Handle file not found errors for configuration files
- Validate scene object is not None before assignment
- Propagate any robot creation errors from existing methods

### Main Application
- Handle initialization failures gracefully
- Ensure proper cleanup if swarm initialization fails

## Testing Strategy

### Unit Tests
1. Test `SwarmManager.initialize_async()` with valid parameters
2. Test `SwarmManager.initialize_async()` with invalid/missing files
3. Test `SwarmManager.initialize_async()` with None scene parameter
4. Verify `Env._async_init()` no longer calls swarm initialization methods

### Integration Tests
1. Test complete initialization sequence in main application
2. Verify robots are properly loaded and activated after refactoring
3. Test that simulation behavior remains unchanged
4. Verify error handling in the complete initialization flow

### Regression Tests
1. Run existing simulation scenarios to ensure no functionality is broken
2. Verify all robot types (jetbot, h1, cf2x) still work correctly
3. Test robot activation and deactivation functionality