"""
Dependency Injection Container for Isaac Sim Application

This module provides a clean dependency injection container using the dependency-injector
library's DeclarativeContainer pattern. It manages all manager instances as singletons
with automatic dependency resolution.
"""

import logging
from typing import Dict, Any

from dependency_injector import containers, providers
import yaml

# Setup logging
logger = logging.getLogger(__name__)

# Import manager classes
from robot.swarm_manager import SwarmManager
from scene.scene_manager import SceneManager
from ui.viewport_manager_enhanced import ViewportManager
from map.map_grid_map import GridMap
from map.map_semantic_map import MapSemantic


class AppContainer(containers.DeclarativeContainer):
    """
    Clean dependency injection container for Isaac Sim application
    
    This container manages all manager instances as singletons with automatic
    dependency resolution. It uses the dependency-injector library's proven
    patterns for professional dependency management.
    """
    
    # Configuration provider for YAML config loading
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


def create_container(config_path: str = './files/env_cfg.yaml', wire_modules: bool = True) -> AppContainer:
    """
    Create and configure the dependency injection container
    
    This function creates the container, loads configuration from YAML,
    and optionally wires modules for dependency injection.
    
    Args:
        config_path: Path to the YAML configuration file
        wire_modules: Whether to wire modules for @inject decorators (default: True)
    
    Returns:
        AppContainer: Configured dependency injection container
        
    Raises:
        FileNotFoundError: If configuration file is not found
        yaml.YAMLError: If configuration file is invalid
        Exception: For other container setup errors
    """
    container = AppContainer()
    
    # Load and set configuration
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        # Validate configuration structure and content
        validate_configuration(config)
        
        container.config.from_dict(config)
        
    except FileNotFoundError as e:
        raise FileNotFoundError(f"Configuration file not found: {e}")
    
    # Wire modules for dependency injection (optional for performance)
    # This enables @inject decorators to work in the specified modules
    if wire_modules:
        try:
            # Wire essential modules - containers module and main module
            modules_to_wire = [__name__]
            
            # Always try to wire main module since it has @inject decorators
            import sys
            if 'main' in sys.modules:
                modules_to_wire.append('main')
            elif '__main__' in sys.modules:
                modules_to_wire.append('__main__')
                
            container.wire(modules=modules_to_wire)
            
        except Exception as e:
            # Don't fail container creation if wiring fails
            # This allows container to work without @inject decorators
            raise Exception(f"Error wiring modules: {e}")
    
    return container


def safe_container_setup(config_path: str = './files/env_cfg.yaml', wire_modules: bool = True) -> AppContainer:
    """
    Safely setup container with comprehensive error handling
    
    This function provides a safe wrapper around container creation with
    detailed error messages and graceful failure handling.
    
    Args:
        config_path: Path to the YAML configuration file
        
    Returns:
        AppContainer: Successfully configured container
        
    Raises:
        RuntimeError: If container setup fails with detailed error message
    """
    try:
        container = create_container(config_path, wire_modules)
        
        # Validate that container was properly configured
        if not hasattr(container, 'config') or container.config is None:
            raise RuntimeError("Container configuration was not properly loaded")

        # Test basic provider resolution without instantiation
        container.viewport_manager.provider
        container.grid_map.provider
        container.scene_manager.provider
        container.swarm_manager.provider
        container.semantic_map.provider

        return container
        
    except Exception as e:
        raise RuntimeError(f"Container setup failed: {e}")


def validate_configuration(config: Dict[str, Any]) -> None:
    """
    Validate configuration dictionary for required fields
    
    Args:
        config: Configuration dictionary to validate
        
    Raises:
        ValueError: If required configuration is missing or invalid
    """
    # Check required top-level sections
    required_sections = ['map']
    for section in required_sections:
        if section not in config:
            raise ValueError(f"Required configuration section '{section}' is missing")
    
    # Validate map configuration
    map_config = config['map']
    required_map_fields = [
        'cell_size', 'start_point', 'min_bounds', 'max_bounds',
        'occupied_cell', 'empty_cell', 'invisible_cell'
    ]
    
    for field in required_map_fields:
        if field not in map_config:
            raise ValueError(f"Required map configuration field '{field}' is missing")
    
    # Validate data types
    if not isinstance(map_config['cell_size'], (int, float)) or map_config['cell_size'] <= 0:
        raise ValueError("Map cell_size must be a positive number")
    
    if not isinstance(map_config['start_point'], list) or len(map_config['start_point']) != 3:
        raise ValueError("Map start_point must be a list of 3 coordinates")
    
    if not isinstance(map_config['min_bounds'], list) or len(map_config['min_bounds']) != 3:
        raise ValueError("Map min_bounds must be a list of 3 coordinates")
    
    if not isinstance(map_config['max_bounds'], list) or len(map_config['max_bounds']) != 3:
        raise ValueError("Map max_bounds must be a list of 3 coordinates")


# Global container instance
_container: AppContainer = None


def get_container() -> AppContainer:
    """
    Get the global container instance with safe initialization
    
    Returns:
        AppContainer: The global container instance
        
    Raises:
        RuntimeError: If container initialization fails
    """
    global _container
    if _container is None:
        _container = safe_container_setup()
    return _container


def reset_container() -> None:
    """
    Reset the global container instance
    
    This is useful for testing or when configuration changes require
    a fresh container instance.
    """
    global _container
    if _container is not None:
        try:
            _container.unwire()
        except Exception:
            pass  # Ignore unwiring errors during reset
    _container = None
