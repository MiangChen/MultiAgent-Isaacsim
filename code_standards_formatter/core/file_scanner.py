"""
File scanning and discovery functionality for the code standards formatter.

This module provides the FileScanner class for discovering Python files
in a project directory with support for exclusion patterns.
"""

import os
import fnmatch
from pathlib import Path
from typing import List, Set, Generator
from ..models import FormatterConfig


class FileScanner:
    """
    Scans directories to discover Python files for formatting.
    
    Supports recursive directory traversal with configurable exclusion patterns
    to skip non-Python files and excluded directories.
    """
    
    def __init__(self, config: FormatterConfig = None):
        """
        Initialize the FileScanner with configuration.
        
        Args:
            config: FormatterConfig object with exclusion patterns.
                   If None, uses default exclusion patterns.
        """
        self.config = config or FormatterConfig(exclude_patterns=[])
        self._python_extensions = {'.py', '.pyw'}
    
    def scan_python_files(self, root_path: str, exclude_patterns: List[str] = None) -> List[str]:
        """
        Recursively scan for Python files in the given directory.
        
        Args:
            root_path: Root directory to start scanning from.
            exclude_patterns: Additional exclusion patterns to use.
                            If None, uses patterns from config.
        
        Returns:
            List of absolute paths to Python files found.
            
        Raises:
            ValueError: If root_path doesn't exist or isn't a directory.
        """
        root_path = Path(root_path).resolve()
        
        if not root_path.exists():
            raise ValueError(f"Root path does not exist: {root_path}")
        
        if not root_path.is_dir():
            raise ValueError(f"Root path is not a directory: {root_path}")
        
        # Use provided patterns or fall back to config patterns
        patterns = exclude_patterns if exclude_patterns is not None else self.config.exclude_patterns
        
        python_files = []
        
        for file_path in self._walk_directory(root_path, patterns):
            if self.is_python_file(str(file_path)):
                python_files.append(str(file_path))
        
        return sorted(python_files)  # Return sorted for consistent ordering
    
    def is_python_file(self, file_path: str) -> bool:
        """
        Check if a file is a Python file based on its extension.
        
        Args:
            file_path: Path to the file to check.
            
        Returns:
            True if the file has a Python extension (.py or .pyw).
        """
        path = Path(file_path)
        return path.suffix.lower() in self._python_extensions and path.is_file()
    
    def should_exclude(self, file_path: str, exclude_patterns: List[str]) -> bool:
        """
        Check if a file or directory should be excluded based on patterns.
        
        Args:
            file_path: Path to check for exclusion.
            exclude_patterns: List of glob patterns to match against.
            
        Returns:
            True if the file/directory should be excluded.
        """
        path = Path(file_path)
        
        # Check the file/directory name and all parent directory names
        path_parts = [path.name] + [parent.name for parent in path.parents]
        
        for pattern in exclude_patterns:
            # Check against the full path
            if fnmatch.fnmatch(str(path), pattern):
                return True
            
            # Check against just the filename
            if fnmatch.fnmatch(path.name, pattern):
                return True
            
            # Check against any part of the path
            for part in path_parts:
                if fnmatch.fnmatch(part, pattern):
                    return True
        
        return False
    
    def _walk_directory(self, root_path: Path, exclude_patterns: List[str]) -> Generator[Path, None, None]:
        """
        Walk through directory tree, yielding files that aren't excluded.
        
        Args:
            root_path: Root directory to walk.
            exclude_patterns: Patterns to exclude.
            
        Yields:
            Path objects for files that should be processed.
        """
        try:
            for item in root_path.iterdir():
                # Skip if this item matches any exclusion pattern
                if self.should_exclude(str(item), exclude_patterns):
                    continue
                
                if item.is_file():
                    yield item
                elif item.is_dir():
                    # Recursively walk subdirectories
                    yield from self._walk_directory(item, exclude_patterns)
                    
        except (PermissionError, OSError) as e:
            # Log the error but continue processing other directories
            print(f"Warning: Could not access directory {root_path}: {e}")
    
    def get_file_count_estimate(self, root_path: str, exclude_patterns: List[str] = None) -> int:
        """
        Get an estimate of how many Python files will be found.
        
        This is useful for progress reporting without doing a full scan.
        
        Args:
            root_path: Root directory to estimate from.
            exclude_patterns: Exclusion patterns to use.
            
        Returns:
            Estimated number of Python files.
        """
        try:
            files = self.scan_python_files(root_path, exclude_patterns)
            return len(files)
        except (ValueError, OSError):
            return 0
    
    def validate_paths(self, file_paths: List[str]) -> List[str]:
        """
        Validate a list of file paths, returning only valid Python files.
        
        Args:
            file_paths: List of file paths to validate.
            
        Returns:
            List of valid Python file paths.
        """
        valid_paths = []
        
        for file_path in file_paths:
            try:
                path = Path(file_path).resolve()
                if path.exists() and self.is_python_file(str(path)):
                    valid_paths.append(str(path))
            except (OSError, ValueError):
                # Skip invalid paths
                continue
        
        return valid_paths
    
    def find_python_files_by_pattern(self, root_path: str, name_pattern: str, 
                                   exclude_patterns: List[str] = None) -> List[str]:
        """
        Find Python files matching a specific name pattern.
        
        Args:
            root_path: Root directory to search in.
            name_pattern: Glob pattern to match filenames against.
            exclude_patterns: Additional exclusion patterns.
            
        Returns:
            List of matching Python file paths.
        """
        all_files = self.scan_python_files(root_path, exclude_patterns)
        matching_files = []
        
        for file_path in all_files:
            filename = Path(file_path).name
            if fnmatch.fnmatch(filename, name_pattern):
                matching_files.append(file_path)
        
        return matching_files