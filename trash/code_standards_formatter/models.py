"""
Data models for the code standards formatter.

This module contains the core data classes used throughout the formatting system.
"""

from dataclasses import dataclass
from typing import List, Optional
from enum import Enum


class ImportCategory(Enum):
    """Categories for organizing import statements."""
    STANDARD_LIBRARY = "standard_library"
    THIRD_PARTY = "third_party"
    ISAACSIM_RELATED = "isaacsim_related"
    LOCAL_IMPORTS = "local_imports"


@dataclass
class ImportInfo:
    """Information about a single import statement."""
    module_name: str
    import_type: str  # 'import' or 'from'
    imported_items: List[str]
    comment: Optional[str] = None
    original_line: str = ""
    category: Optional[ImportCategory] = None


@dataclass
class ImportBlocks:
    """Container for organized import statements by category."""
    standard_library: List[str]
    third_party: List[str]
    isaacsim_related: List[str]
    local_imports: List[str]
    
    def __init__(self):
        self.standard_library = []
        self.third_party = []
        self.isaacsim_related = []
        self.local_imports = []
    
    def add_import(self, import_line: str, category: ImportCategory) -> None:
        """Add an import line to the appropriate category block."""
        if category == ImportCategory.STANDARD_LIBRARY:
            self.standard_library.append(import_line)
        elif category == ImportCategory.THIRD_PARTY:
            self.third_party.append(import_line)
        elif category == ImportCategory.ISAACSIM_RELATED:
            self.isaacsim_related.append(import_line)
        elif category == ImportCategory.LOCAL_IMPORTS:
            self.local_imports.append(import_line)
    
    def get_all_blocks(self) -> List[List[str]]:
        """Get all import blocks as a list of lists."""
        return [
            self.standard_library,
            self.third_party,
            self.isaacsim_related,
            self.local_imports
        ]


@dataclass
class FormatResult:
    """Result of formatting a single file."""
    file_path: str
    changes_made: List[str]
    warnings: List[str]
    success: bool
    backup_created: bool = False
    
    def add_change(self, change_description: str) -> None:
        """Add a change description to the result."""
        self.changes_made.append(change_description)
    
    def add_warning(self, warning_message: str) -> None:
        """Add a warning message to the result."""
        self.warnings.append(warning_message)
    
    def has_changes(self) -> bool:
        """Check if any changes were made to the file."""
        return len(self.changes_made) > 0
    
    def has_warnings(self) -> bool:
        """Check if any warnings were generated."""
        return len(self.warnings) > 0


@dataclass
class FormatterConfig:
    """Configuration settings for the code formatter."""
    exclude_patterns: List[str]
    spaces_before_inline_comment: int = 2
    blank_lines_after_imports: int = 2
    blank_lines_between_import_blocks: int = 1
    create_backups: bool = True
    
    # Import categorization settings
    standard_library_modules: List[str] = None
    isaacsim_prefixes: List[str] = None
    
    def __post_init__(self):
        """Initialize default values after dataclass creation."""
        if self.standard_library_modules is None:
            self.standard_library_modules = [
                "os", "sys", "yaml", "json", "re", "time", "datetime",
                "pathlib", "typing", "dataclasses", "enum", "abc",
                "collections", "itertools", "functools", "operator"
            ]
        
        if self.isaacsim_prefixes is None:
            self.isaacsim_prefixes = ["isaacsim", "omni", "pxr"]
        
        if not self.exclude_patterns:
            self.exclude_patterns = [
                "__pycache__", ".git", "venv", ".venv", "*.pyc",
                ".pytest_cache", "build", "dist", ".egg-info"
            ]