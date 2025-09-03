# Design Document

## Overview

This design implements an elegant dependency injection system using the `dependency-injector` library's `DeclarativeContainer` pattern. The solution uses a clean `AppContainer` class that manages all components as singletons with automatic dependency resolution, while keeping the implementation minimal and focused.

The key advantage is leveraging the proven `dependency-injector` framework while maintaining simplicity - we only use the essential features needed for our architecture.

## Architecture

### DeclarativeContainer Pattern

The new architecture uses `containers.DeclarativeContainer` with a clean, declarative approach:

```
┌─────────────────────────────────────────────────────────────┐
│                    AppContainer                             │
│              (DeclarativeContainer)                         │
│                                                             │
│  config = providers.Configuration()                        │
│  viewport_manager = providers.Singleton(ViewportManager)   │
│  grid_map = providers.Singleton(GridMap, ...)              │
│  scene_manager = providers.Singleton(SceneManager, ...)    │
│  swarm_manager = providers.Singleton(SwarmManager, ...)    │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              Automatic Dependency Injection                │
│                                                             │
│  1. Declare providers in container                         │
│  2. Wire modules with @inject decorators                   │
│  3. Use Provide[Container.service] for injection           │
│  4. Framework handles singleton lifecycle                  │
└─────────────────────────────────────────────────────────────┘
```

### Clean Dependency Resolution

The design uses `dependency-injector`'s proven patterns:

1. **Declarative Providers**: All services declared as providers in the container
2. **Automatic Wiring**: Modules are wired to enable injection
3. **Type-safe Injection**: Uses `Provide[Container.service]` annotations
4. **Built-in Singletons**: Framework manages singleton lifecycle automatically

## Components and Interfaces

### AppContainer Class (Clean & Minimal)

```python
from dependency_injector import containers, providers
from dependency_injector.wiring import inject, Provide

class AppContainer(containers.DeclarativeContainer):
    """Clean dependency injection container for Isaac Sim application"""
    
    # Configuration provider
    config = providers.Configuration()
    
    # Core singletons (no dependencies)
    viewport_manager = providers.Singleton(ViewportManager)
    semantic_map = providers.Singleton(MapSemantic)
    
    # GridMap with configuration injection
    grid_map = providers.Singleton(
        GridMap,
        cell_size=config.map.cell_size,
        start_point=config.map.start_point,
        min_bounds=config.map.min_bounds,
        max_bounds=config.map.max_bounds,
        occupied_cell=config.map.occupied_cell,
        empty_cell=config.map.empty_cell,
        invisible_cell=config.map.invisible_cell,
    )
    
    # Managers with dependency injection
    scene_manager = providers.Singleton(
        SceneManager,
        viewport_manager=viewport_manager
    )
    
    swarm_manager = providers.Singleton(
        SwarmManager,
        map_grid=grid_map
    )
```

### Manager Adaptations (Minimal Changes)

Managers need only minor constructor updates:

```python
# SwarmManager - already compatible
class SwarmManager:
    def __init__(self, map_grid: GridMap = None):
        self.map_grid = map_grid
        # ... existing code unchanged

# SceneManager - already compatible  
class SceneManager:
    def __init__(self, viewport_manager: ViewportManager = None):
        self._viewport_manager = viewport_manager
        # ... existing code unchanged

# ViewportManager - no changes needed
class ViewportManager:
    def __init__(self):
        # ... existing code unchanged

# GridMap - no changes needed
class GridMap:
    def __init__(self, cell_size=1, start_point=[0,0,0], ...):
        # ... existing code unchanged

# MapSemantic - no changes needed
class MapSemantic:
    def __init__(self):
        # ... existing code unchanged
```

### Simple Container Setup

```python
def create_container() -> AppContainer:
    """Create and configure the dependency injection container"""
    container = AppContainer()
    
    # Load and set configuration
    with open('./config/env_cfg.yaml', 'r') as f:
        config = yaml.safe_load(f)
    container.config.from_dict(config)
    
    # Wire modules for injection
    container.wire(modules=[__name__])
    
    return container
```

## Data Models

### Business Logic Functions with Injection

```python
@inject
def setup_simulation(
    simulation_app,
    cfg: Dict[str, Any],
    swarm_manager: SwarmManager = Provide[AppContainer.swarm_manager],
    scene_manager: SceneManager = Provide[AppContainer.scene_manager],
    grid_map: GridMap = Provide[AppContainer.grid_map]
) -> Env:
    """Setup simulation with injected dependencies"""
    # Register robot classes
    swarm_manager.register_robot_class("jetbot", RobotJetbot, RobotCfgJetbot)
    swarm_manager.register_robot_class("h1", RobotH1, RobotCfgH1)
    swarm_manager.register_robot_class("cf2x", RobotCf2x, RobotCfgCf2x)
    
    # Create environment
    env = await Env.create(
        simulation_app=simulation_app,
        physics_dt=cfg['world']['physics_dt'],
        swarm_manager=swarm_manager,
        scene_manager=scene_manager,
        grid_map=grid_map,
    )
    
    return env

@inject
def create_car_objects(
    scene_manager: SceneManager = Provide[AppContainer.scene_manager]
) -> list:
    """Create car objects with injected scene manager"""
    # ... existing car creation logic unchanged
    return created_prim_paths

@inject
def process_semantic_detection(
    semantic_camera,
    map_semantic: MapSemantic = Provide[AppContainer.semantic_map]
) -> None:
    """Process semantic detection with injected map"""
    # ... existing semantic processing logic unchanged
```

### Global Container Instance

```python
# Global container instance
_container: AppContainer = None

def get_container() -> AppContainer:
    """Get the global container instance"""
    global _container
    if _container is None:
        _container = create_container()
    return _container
```

## Error Handling

### Framework-Provided Error Handling

The `dependency-injector` framework provides robust error handling:

```python
def safe_container_setup() -> AppContainer:
    """Safely setup container with error handling"""
    try:
        container = create_container()
        return container
    except Exception as e:
        print(f"Failed to setup dependency injection container: {e}")
        raise

# The framework automatically handles:
# - Missing dependencies
# - Circular dependency detection  
# - Provider resolution errors
# - Configuration validation
```

## Testing Strategy

### Easy Testing with Provider Overrides

The framework provides excellent testing support:

```python
class TestSwarmManager:
    def setup_method(self):
        self.container = AppContainer()
        # Override providers with mocks
        self.container.grid_map.override(Mock(spec=GridMap))
        self.container.wire(modules=[__name__])
    
    @inject
    def test_swarm_initialization(
        self,
        swarm_manager: SwarmManager = Provide[AppContainer.swarm_manager]
    ):
        assert swarm_manager is not None
        assert isinstance(swarm_manager.map_grid, Mock)
```

### Integration Testing

```python
def test_integration():
    """Test full integration with real services"""
    container = create_container()
    
    # Get services through container
    scene_manager = container.scene_manager()
    swarm_manager = container.swarm_manager()
    
    assert scene_manager._viewport_manager is not None
    assert swarm_manager.map_grid is not None
    
    # Verify singleton behavior
    assert container.scene_manager() is scene_manager
```

## Performance Considerations

### Framework-Optimized Performance

The `dependency-injector` framework is highly optimized:

```python
# Lazy initialization built-in
container.viewport_manager()  # Creates ViewportManager only when called
container.scene_manager()     # Creates SceneManager with injected ViewportManager  
container.scene_manager()     # Returns cached singleton instance
```

### Minimal Runtime Overhead

- Providers are resolved at call time with minimal overhead
- Singletons are cached efficiently
- No runtime reflection or complex proxy objects
- Framework is battle-tested for performance

## Migration Strategy

### Clean Migration with dependency-injector

The migration is straightforward and elegant:

1. **Install dependency-injector**: `pip install dependency-injector`
2. **Create AppContainer** (~20 lines of clean declarative code)
3. **Update main.py** (~15 lines changed)
4. **Add @inject decorators** to business logic functions

### New main.py Structure

```python
from dependency_injector.wiring import inject, Provide
from containers import AppContainer

@inject
def run_simulation(
    swarm_manager: SwarmManager = Provide[AppContainer.swarm_manager],
    scene_manager: SceneManager = Provide[AppContainer.scene_manager],
    grid_map: GridMap = Provide[AppContainer.grid_map],
    semantic_map: MapSemantic = Provide[AppContainer.semantic_map]
):
    """Main simulation logic with injected dependencies"""
    # All existing simulation logic remains the same
    # Just use the injected managers instead of manually created ones
    
def main():
    """Clean main function"""
    # Setup container
    container = create_container()
    
    # Run simulation with injected dependencies
    run_simulation()

if __name__ == "__main__":
    main()
```

### Backward Compatibility

The design maintains full backward compatibility:
- All existing manager code works unchanged
- Managers can still be instantiated manually if needed
- Gradual migration possible - can inject some services while keeping others manual
- No breaking changes to existing APIs

### Total Code Changes

The entire migration requires:
- **AppContainer class**: ~20 lines of declarative providers
- **Container setup**: ~10 lines
- **Main.py refactoring**: ~20 lines
- **Business logic extraction**: ~30 lines
- **Total**: ~80 lines of clean, maintainable code

This provides professional dependency injection with proven framework reliability and minimal code changes.