"""
Configuration management for the code standards formatter.

This module handles loading and validating configuration from YAML files.
"""

import yaml
from pathlib import Path
from typing import Dict, Any, Optional
from .models import FormatterConfig


class ConfigLoader:
    """Handles loading and parsing configuration files."""
    
    DEFAULT_CONFIG_NAME = "code_standards_config.yaml"
    
    @staticmethod
    def load_config(config_path: Optional[str] = None) -> FormatterConfig:
        """
        Load configuration from a YAML file.
        
        Args:
            config_path: Path to the configuration file. If None, looks for default config.
            
        Returns:
            FormatterConfig object with loaded settings.
        """
        if config_path is None:
            config_path = ConfigLoader._find_default_config()
        
        if config_path and Path(config_path).exists():
            return ConfigLoader._load_from_file(config_path)
        else:
            return ConfigLoader._create_default_config()
    
    @staticmethod
    def _find_default_config() -> Optional[str]:
        """Find the default configuration file in common locations."""
        possible_locations = [
            Path.cwd() / ConfigLoader.DEFAULT_CONFIG_NAME,
            Path.cwd() / "config" / ConfigLoader.DEFAULT_CONFIG_NAME,
            Path.cwd() / ".config" / ConfigLoader.DEFAULT_CONFIG_NAME,
        ]
        
        for location in possible_locations:
            if location.exists():
                return str(location)
        
        return None
    
    @staticmethod
    def _load_from_file(config_path: str) -> FormatterConfig:
        """Load configuration from a specific YAML file."""
        try:
            with open(config_path, 'r', encoding='utf-8') as file:
                config_data = yaml.safe_load(file) or {}
            
            return ConfigLoader._parse_config_data(config_data)
        
        except (yaml.YAMLError, FileNotFoundError, PermissionError) as e:
            print(f"Warning: Could not load config from {config_path}: {e}")
            print("Using default configuration.")
            return ConfigLoader._create_default_config()
    
    @staticmethod
    def _parse_config_data(config_data: Dict[str, Any]) -> FormatterConfig:
        """Parse configuration data from loaded YAML."""
        return FormatterConfig(
            exclude_patterns=config_data.get('exclude_patterns', []),
            spaces_before_inline_comment=config_data.get('spaces_before_inline_comment', 2),
            blank_lines_after_imports=config_data.get('blank_lines_after_imports', 2),
            blank_lines_between_import_blocks=config_data.get('blank_lines_between_import_blocks', 1),
            create_backups=config_data.get('create_backups', True),
            standard_library_modules=config_data.get('standard_library_modules'),
            isaacsim_prefixes=config_data.get('isaacsim_prefixes')
        )
    
    @staticmethod
    def _create_default_config() -> FormatterConfig:
        """Create a default configuration when no config file is found."""
        return FormatterConfig(exclude_patterns=[])
    
    @staticmethod
    def create_sample_config(output_path: str = None) -> str:
        """
        Create a sample configuration file.
        
        Args:
            output_path: Where to save the sample config. If None, uses default name.
            
        Returns:
            Path to the created sample configuration file.
        """
        if output_path is None:
            output_path = ConfigLoader.DEFAULT_CONFIG_NAME
        
        sample_config = {
            'exclude_patterns': [
                '__pycache__',
                '.git',
                'venv',
                '.venv',
                '*.pyc',
                '.pytest_cache',
                'build',
                'dist',
                '.egg-info'
            ],
            'spaces_before_inline_comment': 2,
            'blank_lines_after_imports': 2,
            'blank_lines_between_import_blocks': 1,
            'create_backups': True,
            'standard_library_modules': [
                'os', 'sys', 'yaml', 'json', 're', 'time', 'datetime',
                'pathlib', 'typing', 'dataclasses', 'enum', 'abc',
                'collections', 'itertools', 'functools', 'operator'
            ],
            'isaacsim_prefixes': [
                'isaacsim', 'omni', 'pxr'
            ]
        }
        
        with open(output_path, 'w', encoding='utf-8') as file:
            yaml.dump(sample_config, file, default_flow_style=False, indent=2)
        
        return output_path