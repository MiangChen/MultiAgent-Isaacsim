# =============================================================================
# Omniverse Utilities - Unified Import and Helper Functions
# =============================================================================
#
# This module provides a centralized import system for Omniverse platform APIs
# and commonly used utility functions. It organizes imports by functional areas
# and provides convenient aliases for frequently used classes and functions.
#
# The Omniverse platform provides various APIs for:
# - Core platform services (omni.kit, omni.client)
# - USD scene graph operations (omni.usd)
# - Physics simulation (omni.physx)
# - Graph-based programming (omni.graph)
# - UI and application framework (omni.ui, omni.appwindow)
# - Asset and file management (omni.client)
# - Extensions and plugins (omni.ext)
#
# Module Organization:
# 1. Core Omniverse platform APIs
# 2. USD and scene graph utilities
# 3. Physics and simulation APIs
# 4. Graph and node-based programming
# 5. UI and application framework
# 6. Asset management and client APIs
# 7. Extension and plugin system
# 8. Utility functions and helpers
#
# =============================================================================

# Third-party library imports (Omniverse Platform)
# -----------------------------------------------------------------------------
# 1. Core Platform - Base Omniverse Services
# -----------------------------------------------------------------------------
# omni: Core Omniverse platform namespace
import omni

# -----------------------------------------------------------------------------
# 2. Kit Framework - Application and Service Framework
# -----------------------------------------------------------------------------
# omni.kit: Core application framework and services
try:
    import omni.kit
    from omni.kit.async_engine import run_coroutine
    import omni.kit.app
    import omni.kit.commands
except ImportError:
    pass

# -----------------------------------------------------------------------------
# 3. USD Integration - Universal Scene Description
# -----------------------------------------------------------------------------
# omni.usd: Omniverse USD integration and utilities
try:
    import omni.usd
    from omni.usd import get_context, get_world_transform_matrix
except ImportError:
    pass

# -----------------------------------------------------------------------------
# 4. Physics - PhysX Integration
# -----------------------------------------------------------------------------
# omni.physx: PhysX physics engine integration
try:
    import omni.physx
    from omni.physx import get_physx_scene_query_interface
    from omni.physx import get_physx_simulation_interface
    from omni.physx import get_physx_cooking_interface
except ImportError:
    pass

# -----------------------------------------------------------------------------
# 5. Graph System - Node-Based Programming
# -----------------------------------------------------------------------------
# omni.graph: Action Graph and OmniGraph system
try:
    import omni.graph.core as og
    import omni.graph.tools
    from omni.graph.core import GraphRegistry
except ImportError:
    pass

# -----------------------------------------------------------------------------
# 6. UI Framework - User Interface
# -----------------------------------------------------------------------------
# omni.ui: User interface framework
try:
    import omni.ui
    import omni.appwindow
    from omni.ui import color, alignment
except ImportError:
    pass

# -----------------------------------------------------------------------------
# 7. Client APIs - Asset and File Management
# -----------------------------------------------------------------------------
# omni.client: Nucleus and asset management
try:
    import omni.client
    from omni.client import get_server_info
except ImportError:
    pass

# -----------------------------------------------------------------------------
# 8. Extensions - Plugin System
# -----------------------------------------------------------------------------
# omni.ext: Extension and plugin framework
try:
    import omni.ext
    from omni.ext import IExt
except ImportError:
    pass

# -----------------------------------------------------------------------------
# 9. Timeline and Animation
# -----------------------------------------------------------------------------
# omni.timeline: Timeline and animation control
try:
    import omni.timeline
    from omni.timeline import get_timeline_interface
except ImportError:
    pass

# -----------------------------------------------------------------------------
# 10. Rendering and Viewport
# -----------------------------------------------------------------------------
# omni.kit.viewport: Viewport and rendering utilities
try:
    import omni.kit.viewport.utility
    from omni.kit.viewport.utility import get_active_viewport_window
except ImportError:
    pass

# -----------------------------------------------------------------------------
# 11. Hydra and Rendering
# -----------------------------------------------------------------------------
# omni.hydra: Hydra rendering framework integration
try:
    import omni.hydra
except ImportError:
    pass

# =============================================================================
# Commonly Used Classes and Functions - Direct Access Aliases
# =============================================================================
#
# These aliases provide direct access to frequently used Omniverse classes
# and functions without requiring the full module path. This improves code
# readability and follows the pattern established in other utils modules.
#
# Usage Examples:
#   Instead of: omni.usd.get_context()
#   Use:        get_usd_context()
#
#   Instead of: omni.physx.get_physx_scene_query_interface()
#   Use:        get_physx_query_interface()
#
# =============================================================================

# -----------------------------------------------------------------------------
# USD Context and Scene Management
# -----------------------------------------------------------------------------
try:
    get_usd_context = omni.usd.get_context
    get_world_transform = omni.usd.get_world_transform_matrix
except (ImportError, AttributeError):
    get_usd_context = None
    get_world_transform = None

# -----------------------------------------------------------------------------
# PhysX Physics Interfaces
# -----------------------------------------------------------------------------
try:
    get_physx_query_interface = omni.physx.get_physx_scene_query_interface
    get_physx_simulation_interface = omni.physx.get_physx_simulation_interface
    get_physx_cooking_interface = omni.physx.get_physx_cooking_interface
except (ImportError, AttributeError):
    get_physx_query_interface = None
    get_physx_simulation_interface = None
    get_physx_cooking_interface = None

# -----------------------------------------------------------------------------
# Timeline and Animation Control
# -----------------------------------------------------------------------------
try:
    get_timeline_interface = omni.timeline.get_timeline_interface
except (ImportError, AttributeError):
    get_timeline_interface = None

# -----------------------------------------------------------------------------
# Viewport and Rendering
# -----------------------------------------------------------------------------
try:
    get_active_viewport = omni.kit.viewport.utility.get_active_viewport_window
except (ImportError, AttributeError):
    get_active_viewport = None

# -----------------------------------------------------------------------------
# Graph System
# -----------------------------------------------------------------------------
try:
    GraphRegistry = omni.graph.core.GraphRegistry
    og_core = omni.graph.core
except (ImportError, AttributeError):
    GraphRegistry = None
    og_core = None

# =============================================================================
# Utility Functions - Common Omniverse Operations
# =============================================================================
#
# These utility functions provide high-level operations for common Omniverse
# tasks such as scene management, physics queries, and asset operations.
# They abstract away low-level API details and provide a more convenient
# interface for typical simulation and robotics use cases.
#
# =============================================================================

def get_current_stage():
    """
    Get the current USD stage from the Omniverse context.
    
    Returns:
        Usd.Stage: Current USD stage, or None if no context available
        
    Example:
        >>> stage = get_current_stage()
        >>> if stage:
        ...     print(f"Stage has {len(stage.GetRootLayer().GetPrimAtPath('/'))} root prims")
    """
    if get_usd_context is None:
        return None
    
    context = get_usd_context()
    if context:
        return context.get_stage()
    return None

def get_selection():
    """
    Get the currently selected prims in the Omniverse viewport.
    
    Returns:
        list: List of selected prim paths, or empty list if none selected
        
    Example:
        >>> selected = get_selection()
        >>> for path in selected:
        ...     print(f"Selected: {path}")
    """
    if get_usd_context is None:
        return []
    
    context = get_usd_context()
    if context:
        selection = context.get_selection()
        return selection.get_selected_prim_paths()
    return []

def set_selection(prim_paths):
    """
    Set the selection in the Omniverse viewport.
    
    Args:
        prim_paths (list): List of prim paths to select
        
    Returns:
        bool: True if selection was set successfully, False otherwise
        
    Example:
        >>> success = set_selection(["/World/Cube", "/World/Sphere"])
        >>> print(f"Selection set: {success}")
    """
    if get_usd_context is None:
        return False
    
    context = get_usd_context()
    if context:
        selection = context.get_selection()
        selection.set_selected_prim_paths(prim_paths, True)
        return True
    return False

def play_simulation():
    """
    Start/play the physics simulation.
    
    Returns:
        bool: True if simulation started successfully, False otherwise
    """
    if get_timeline_interface is None:
        return False
    
    timeline = get_timeline_interface()
    if timeline:
        timeline.play()
        return True
    return False

def pause_simulation():
    """
    Pause the physics simulation.
    
    Returns:
        bool: True if simulation paused successfully, False otherwise
    """
    if get_timeline_interface is None:
        return False
    
    timeline = get_timeline_interface()
    if timeline:
        timeline.pause()
        return True
    return False

def stop_simulation():
    """
    Stop the physics simulation and reset to frame 0.
    
    Returns:
        bool: True if simulation stopped successfully, False otherwise
    """
    if get_timeline_interface is None:
        return False
    
    timeline = get_timeline_interface()
    if timeline:
        timeline.stop()
        return True
    return False

def is_simulation_playing():
    """
    Check if the physics simulation is currently playing.
    
    Returns:
        bool: True if simulation is playing, False otherwise
    """
    if get_timeline_interface is None:
        return False
    
    timeline = get_timeline_interface()
    if timeline:
        return timeline.is_playing()
    return False

def get_current_time():
    """
    Get the current simulation time in seconds.
    
    Returns:
        float: Current simulation time, or 0.0 if timeline not available
    """
    if get_timeline_interface is None:
        return 0.0
    
    timeline = get_timeline_interface()
    if timeline:
        return timeline.get_current_time()
    return 0.0

def set_current_time(time_seconds):
    """
    Set the current simulation time.
    
    Args:
        time_seconds (float): Time to set in seconds
        
    Returns:
        bool: True if time was set successfully, False otherwise
    """
    if get_timeline_interface is None:
        return False
    
    timeline = get_timeline_interface()
    if timeline:
        timeline.set_current_time(time_seconds)
        return True
    return False

def raycast_closest(origin, direction, max_distance=1000.0):
    """
    Perform a raycast and return the closest hit.
    
    Args:
        origin (tuple): (x, y, z) ray origin in world coordinates
        direction (tuple): (x, y, z) ray direction (will be normalized)
        max_distance (float): Maximum ray distance
        
    Returns:
        dict: Hit information with keys 'hit', 'distance', 'position', 'normal', 'prim_path'
              Returns {'hit': False} if no hit or interface not available
              
    Example:
        >>> # Raycast downward from (0, 10, 0)
        >>> hit = raycast_closest((0, 10, 0), (0, -1, 0), max_distance=20.0)
        >>> if hit['hit']:
        ...     print(f"Hit at distance {hit['distance']}: {hit['prim_path']}")
    """
    if get_physx_query_interface is None:
        return {'hit': False}
    
    query_interface = get_physx_query_interface()
    if not query_interface:
        return {'hit': False}
    
    # Normalize direction
    import math
    length = math.sqrt(sum(d*d for d in direction))
    if length == 0:
        return {'hit': False}
    
    direction = tuple(d/length for d in direction)
    
    # Perform raycast
    hit_info = query_interface.raycast_closest(origin, direction, max_distance)
    
    if hit_info and hit_info.is_hit:
        return {
            'hit': True,
            'distance': hit_info.distance,
            'position': (hit_info.position.x, hit_info.position.y, hit_info.position.z),
            'normal': (hit_info.normal.x, hit_info.normal.y, hit_info.normal.z),
            'prim_path': hit_info.rigid_body if hasattr(hit_info, 'rigid_body') else None
        }
    
    return {'hit': False}

def execute_command(command_name, **kwargs):
    """
    Execute an Omniverse command with the given parameters.
    
    Args:
        command_name (str): Name of the command to execute
        **kwargs: Command parameters
        
    Returns:
        bool: True if command executed successfully, False otherwise
        
    Example:
        >>> # Create a cube primitive
        >>> success = execute_command(
        ...     "CreatePrimWithDefaultXform",
        ...     prim_type="Cube",
        ...     prim_path="/World/MyCube"
        ... )
    """
    try:
        import omni.kit.commands
        omni.kit.commands.execute(command_name, **kwargs)
        return True
    except Exception as e:
        print(f"Command execution failed: {e}")
        return False

def run_async(coroutine):
    """
    Run an async coroutine in the Omniverse async engine.
    
    Args:
        coroutine: Async coroutine to run
        
    Returns:
        Any: Result of the coroutine, or None if async engine not available
        
    Example:
        >>> async def my_async_function():
        ...     await asyncio.sleep(1)
        ...     return "Done"
        >>> 
        >>> result = run_async(my_async_function())
        >>> print(result)  # "Done"
    """
    if run_coroutine is None:
        return None
    
    try:
        return run_coroutine(coroutine)
    except Exception as e:
        print(f"Async execution failed: {e}")
        return None

# =============================================================================
# Graph Utilities - OmniGraph Helper Functions
# =============================================================================

def create_graph(graph_path):
    """
    Create a new OmniGraph at the specified path.
    
    Args:
        graph_path (str): USD path for the graph (e.g., "/World/ActionGraph")
        
    Returns:
        og.Graph: Created graph object, or None if creation failed
    """
    if og_core is None:
        return None
    
    try:
        return og_core.create_graph(graph_path)
    except Exception as e:
        print(f"Graph creation failed: {e}")
        return None

def create_node(graph, node_type, node_path=None):
    """
    Create a node in the specified graph.
    
    Args:
        graph (og.Graph): Graph to create node in
        node_type (str): Type of node to create (e.g., "omni.graph.nodes.ConstantDouble")
        node_path (str, optional): Path for the node within the graph
        
    Returns:
        og.Node: Created node object, or None if creation failed
    """
    if og_core is None or not graph:
        return None
    
    try:
        if node_path:
            return og_core.create_node(graph, node_type, node_path)
        else:
            return og_core.create_node(graph, node_type)
    except Exception as e:
        print(f"Node creation failed: {e}")
        return None

def connect_attributes(source_attr, target_attr):
    """
    Connect two node attributes in an OmniGraph.
    
    Args:
        source_attr (og.Attribute): Source attribute
        target_attr (og.Attribute): Target attribute
        
    Returns:
        bool: True if connection was successful, False otherwise
    """
    if og_core is None:
        return False
    
    try:
        og_core.connect(source_attr, target_attr)
        return True
    except Exception as e:
        print(f"Attribute connection failed: {e}")
        return False

# =============================================================================
# Usage Examples and Best Practices
# =============================================================================

"""
Common Usage Patterns:

1. Scene Management:
    ```python
    from physics_engine.omni_utils import *
    
    # Get current stage and selection
    stage = get_current_stage()
    selected = get_selection()
    
    # Set selection to specific prims
    set_selection(["/World/Cube", "/World/Sphere"])
    ```

2. Simulation Control:
    ```python
    # Control simulation playback
    play_simulation()
    
    # Check if playing and get current time
    if is_simulation_playing():
        current_time = get_current_time()
        print(f"Simulation time: {current_time}")
    
    # Stop simulation
    stop_simulation()
    ```

3. Physics Queries:
    ```python
    # Perform raycast from camera position
    hit = raycast_closest(
        origin=(0, 5, 0),
        direction=(0, -1, 0),
        max_distance=10.0
    )
    
    if hit['hit']:
        print(f"Hit object at {hit['position']}")
    ```

4. Command Execution:
    ```python
    # Create primitives using commands
    execute_command(
        "CreatePrimWithDefaultXform",
        prim_type="Sphere",
        prim_path="/World/MySphere"
    )
    
    # Delete selected prims
    execute_command("DeletePrims", paths=get_selection())
    ```

5. Graph Programming:
    ```python
    # Create action graph
    graph = create_graph("/World/ActionGraph")
    
    # Create nodes and connect them
    const_node = create_node(graph, "omni.graph.nodes.ConstantDouble")
    print_node = create_node(graph, "omni.graph.nodes.PrintDouble")
    
    # Connect output to input
    connect_attributes(
        const_node.get_attribute("outputs:value"),
        print_node.get_attribute("inputs:value")
    )
    ```

Best Practices:
- Always check if interfaces are available before use (they may be None)
- Use try-catch blocks for operations that might fail
- Cache interface references for performance in tight loops
- Use async functions with run_async() for non-blocking operations
- Prefer high-level utility functions over direct API calls
- Check simulation state before performing physics operations
- Use proper USD paths (starting with '/') for all prim operations
- Clean up resources and stop simulations when done
"""