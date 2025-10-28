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
