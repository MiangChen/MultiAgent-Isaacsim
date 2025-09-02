"""
Core data models for the code cleanup system.
"""
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Dict, Any, Optional
from pathlib import Path


class FileType(Enum):
    """Classification of file types in the project."""
    CORE = "core"           # Core files, cannot be moved
    ROBOT = "robot"         # Robot-related files
    CONTROLLER = "controller"  # Controller files
    UTILS = "utils"         # Utility files
    MAP = "map"            # Map files
    CAMERA = "camera"       # Camera files
    PATH_PLANNING = "path_planning"  # Path planning files
    OTHER = "other"        # Other files


class SafetyLevel(Enum):
    """Safety level for file operations."""
    CRITICAL = "critical"   # Critical files, absolutely do not move
    SAFE = "safe"          # Safe to move
    REVIEW = "review"      # Requires manual review


@dataclass
class FileAnalysis:
    """Analysis result for a single file."""
    path: str
    is_used: bool
    imported_by: List[str] = field(default_factory=list)
    imports: List[str] = field(default_factory=list)
    file_type: FileType = FileType.OTHER
    safety_level: SafetyLevel = SafetyLevel.REVIEW
    
    def __post_init__(self):
        """Validate and normalize the file path."""
        self.path = str(Path(self.path).resolve())


@dataclass
class CleanupStats:
    """Statistics for cleanup operations."""
    total_files: int = 0
    used_files: int = 0
    unused_files: int = 0
    moved_files: int = 0
    skipped_files: int = 0
    errors: int = 0
    
    @property
    def usage_rate(self) -> float:
        """Calculate file usage rate."""
        if self.total_files == 0:
            return 0.0
        return self.used_files / self.total_files