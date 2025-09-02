"""
Configuration management for the code cleanup system.
"""
import yaml
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional
from pathlib import Path
import logging


@dataclass
class CleanupConfig:
    """Configuration for code cleanup operations."""
    # Entry points for dependency analysis
    entry_points: List[str] = field(default_factory=lambda: ["main.py"])
    
    # Critical files that should never be moved
    critical_files: List[str] = field(default_factory=lambda: [
        "main.py", 
        "containers.py", 
        "__init__.py",
        "setup.py",
        "pyproject.toml",
        "requirements.txt"
    ])
    
    # Patterns to exclude from analysis
    exclude_patterns: List[str] = field(default_factory=lambda: [
        "*/test/*",
        "*/__pycache__/*", 
        "*/trash/*",
        "*.pyc",
        "*.pyo",
        "*/.git/*",
        "*/.pytest_cache/*",
        "*/build/*",
        "*/dist/*"
    ])
    
    # Directory to move unused files to
    trash_directory: str = "trash"
    
    # Whether to create backup before moving
    create_backup: bool = True
    
    # Dry run mode - analyze only, don't move files
    dry_run: bool = False
    
    # Logging configuration
    log_level: str = "INFO"
    log_file: Optional[str] = "cleanup.log"
    
    # Project root directory
    project_root: str = "."
    
    def __post_init__(self):
        """Validate configuration after initialization."""
        self._validate_config()
    
    def _validate_config(self):
        """Validate configuration parameters."""
        # Validate entry points
        if not self.entry_points:
            raise ValueError("At least one entry point must be specified")
        
        # Validate project root
        project_path = Path(self.project_root)
        if not project_path.exists():
            raise ValueError(f"Project root does not exist: {self.project_root}")
        
        # Validate entry points exist
        for entry_point in self.entry_points:
            entry_path = project_path / entry_point
            if not entry_path.exists():
                logging.warning(f"Entry point does not exist: {entry_point}")
        
        # Validate log level
        valid_levels = ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]
        if self.log_level.upper() not in valid_levels:
            raise ValueError(f"Invalid log level: {self.log_level}")
        
        # Normalize paths
        self.project_root = str(project_path.resolve())
        # Keep trash_directory as relative path for easier testing and configuration
        if not Path(self.trash_directory).is_absolute():
            self.trash_directory = self.trash_directory  # Keep as relative
    
    @classmethod
    def from_yaml(cls, config_path: str) -> 'CleanupConfig':
        """Load configuration from YAML file."""
        config_file = Path(config_path)
        if not config_file.exists():
            raise FileNotFoundError(f"Configuration file not found: {config_path}")
        
        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                config_data = yaml.safe_load(f) or {}
        except yaml.YAMLError as e:
            raise ValueError(f"Invalid YAML in configuration file: {e}")
        
        return cls(**config_data) 
   
    def to_yaml(self, config_path: str):
        """Save configuration to YAML file."""
        config_data = {
            'entry_points': self.entry_points,
            'critical_files': self.critical_files,
            'exclude_patterns': self.exclude_patterns,
            'trash_directory': self.trash_directory,
            'create_backup': self.create_backup,
            'dry_run': self.dry_run,
            'log_level': self.log_level,
            'log_file': self.log_file,
            'project_root': self.project_root
        }
        
        config_file = Path(config_path)
        config_file.parent.mkdir(parents=True, exist_ok=True)
        
        try:
            with open(config_file, 'w', encoding='utf-8') as f:
                yaml.dump(config_data, f, default_flow_style=False, indent=2)
        except Exception as e:
            raise ValueError(f"Failed to save configuration: {e}")
    
    def is_critical_file(self, file_path: str) -> bool:
        """Check if a file is marked as critical."""
        file_path = str(Path(file_path).resolve())
        project_root = Path(self.project_root)
        
        for pattern in self.critical_files:
            # Handle absolute paths
            if Path(pattern).is_absolute():
                if file_path == str(Path(pattern).resolve()):
                    return True
            else:
                # Handle relative paths and patterns
                if pattern.endswith('*'):
                    # Simple wildcard matching
                    pattern_base = pattern[:-1]
                    if file_path.endswith(pattern_base):
                        return True
                else:
                    # Exact match (relative to project root)
                    full_pattern_path = str((project_root / pattern).resolve())
                    if file_path == full_pattern_path:
                        return True
        
        return False
    
    def should_exclude(self, file_path: str) -> bool:
        """Check if a file should be excluded from analysis."""
        file_path = str(Path(file_path).resolve())
        
        for pattern in self.exclude_patterns:
            # Simple pattern matching - can be enhanced with fnmatch if needed
            if '*' in pattern:
                # Remove leading/trailing wildcards for contains check
                pattern_clean = pattern.strip('*/')
                if pattern_clean in file_path:
                    return True
            else:
                if pattern in file_path:
                    return True
        
        return False


def create_default_config(project_root: str = ".") -> CleanupConfig:
    """Create a default configuration for the project."""
    return CleanupConfig(
        project_root=project_root,
        entry_points=["main.py"],
        critical_files=[
            "main.py",
            "containers.py", 
            "__init__.py",
            "setup.py",
            "pyproject.toml",
            "requirements.txt"
        ]
    )