"""
Code cleanup system for identifying and safely moving unused files.
"""

from .models import FileAnalysis, FileType, SafetyLevel, CleanupStats
from .config import CleanupConfig, create_default_config
from .dependency_analyzer import DependencyAnalyzer, ImportInfo
from .file_mover import FileMover

__version__ = "1.0.0"
__all__ = [
    "FileAnalysis",
    "FileType", 
    "SafetyLevel",
    "CleanupStats",
    "CleanupConfig",
    "create_default_config",
    "DependencyAnalyzer",
    "ImportInfo",
    "FileMover"
]