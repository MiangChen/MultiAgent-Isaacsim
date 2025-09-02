"""
Safety checker component for the code cleanup system.
Provides multi-layer safety checks to ensure critical files are protected
and code integrity is maintained during cleanup operations.
"""
import ast
import sys
import importlib.util
import subprocess
from pathlib import Path
from typing import List, Set, Dict, Optional, Tuple
import logging
from fnmatch import fnmatch

from .config import CleanupConfig
from .models import FileAnalysis, SafetyLevel


class SafetyChecker:
    """
    Multi-layer safety checker for code cleanup operations.
    
    Provides comprehensive safety checks including:
    - Critical file protection
    - Syntax validation
    - Import validation
    - Dependency integrity checks
    """
    
    def __init__(self, config: CleanupConfig):
        """
        Initialize the safety checker.
        
        Args:
            config: Cleanup configuration containing critical files and settings
        """
        self.config = config
        self.project_root = Path(config.project_root)
        self.logger = logging.getLogger(__name__)
        
        # Build comprehensive critical files list
        self._critical_files = self._build_critical_files_list()
        
        # Cache for syntax validation results
        self._syntax_cache: Dict[str, bool] = {}
        
    def _build_critical_files_list(self) -> Set[str]:
        """
        Build a comprehensive list of critical files that should never be moved.
        
        Returns:
            Set of absolute paths to critical files
        """
        critical_files = set()
        
        for pattern in self.config.critical_files:
            if Path(pattern).is_absolute():
                # Absolute path
                critical_files.add(str(Path(pattern).resolve()))
            else:
                # Relative path or pattern
                if '*' in pattern or '?' in pattern:
                    # Pattern matching
                    for file_path in self.project_root.rglob('*'):
                        if file_path.is_file() and fnmatch(str(file_path), pattern):
                            critical_files.add(str(file_path.resolve()))
                        # Also check relative path matching
                        try:
                            rel_path = file_path.relative_to(self.project_root)
                            if fnmatch(str(rel_path), pattern):
                                critical_files.add(str(file_path.resolve()))
                        except ValueError:
                            # Path is not relative to project root
                            pass
                else:
                    # Exact path
                    full_path = self.project_root / pattern
                    if full_path.exists():
                        critical_files.add(str(full_path.resolve()))
        
        # Always include common critical files
        common_critical = [
            "__init__.py",
            "setup.py", 
            "pyproject.toml",
            "requirements.txt",
            "main.py",
            "containers.py"
        ]
        
        for critical_file in common_critical:
            for file_path in self.project_root.rglob(critical_file):
                if file_path.is_file():
                    critical_files.add(str(file_path.resolve()))
        
        self.logger.debug(f"Built critical files list with {len(critical_files)} files")
        return critical_files
    
    def is_safe_to_move(self, file_path: str) -> Tuple[bool, str]:
        """
        Perform comprehensive safety check to determine if a file can be safely moved.
        
        Args:
            file_path: Path to the file to check
            
        Returns:
            Tuple of (is_safe, reason) where is_safe is boolean and reason explains the decision
        """
        file_path = str(Path(file_path).resolve())
        
        # Check 1: Critical file protection
        if self._is_critical_file(file_path):
            return False, "File is marked as critical and cannot be moved"
        
        # Check 2: File extension validation
        if not self._is_moveable_file_type(file_path):
            return False, "File type is not safe to move"
        
        # Check 3: Special directory protection
        if self._is_in_protected_directory(file_path):
            return False, "File is in a protected directory"
        
        # Check 4: Syntax validation (for Python files)
        if file_path.endswith('.py'):
            is_valid, syntax_reason = self._validate_syntax(file_path)
            if not is_valid:
                return False, f"Syntax validation failed: {syntax_reason}"
        
        # Check 5: Import dependency validation
        if self._has_critical_imports(file_path):
            return False, "File contains critical imports that may break the system"
        
        return True, "File passed all safety checks"
    
    def _is_critical_file(self, file_path: str) -> bool:
        """
        Check if a file is in the critical files list.
        
        Args:
            file_path: Absolute path to the file
            
        Returns:
            True if the file is critical, False otherwise
        """
        file_path = str(Path(file_path).resolve())
        
        # Direct match
        if file_path in self._critical_files:
            return True
        
        # Check against config patterns
        return self.config.is_critical_file(file_path)
    
    def _is_moveable_file_type(self, file_path: str) -> bool:
        """
        Check if the file type is safe to move.
        
        Args:
            file_path: Path to the file
            
        Returns:
            True if the file type can be safely moved
        """
        file_path = Path(file_path)
        
        # Safe to move file extensions
        moveable_extensions = {'.py', '.txt', '.md', '.yaml', '.yml', '.json', '.cfg'}
        
        # Never move these extensions
        protected_extensions = {'.so', '.dll', '.exe', '.bin'}
        
        suffix = file_path.suffix.lower()
        
        if suffix in protected_extensions:
            return False
        
        if suffix in moveable_extensions:
            return True
        
        # For unknown extensions, be conservative
        return False
    
    def _is_in_protected_directory(self, file_path: str) -> bool:
        """
        Check if the file is in a directory that should be protected.
        
        Args:
            file_path: Path to the file
            
        Returns:
            True if the file is in a protected directory
        """
        file_path = Path(file_path)
        
        protected_dirs = {
            '.git', '.pytest_cache', '__pycache__', 
            'build', 'dist', '.egg-info',
            '.vscode', '.idea'
        }
        
        # Check if any parent directory is protected
        for parent in file_path.parents:
            if parent.name in protected_dirs:
                return True
        
        return False
    
    def _validate_syntax(self, file_path: str) -> Tuple[bool, str]:
        """
        Validate Python file syntax.
        
        Args:
            file_path: Path to the Python file
            
        Returns:
            Tuple of (is_valid, error_message)
        """
        # Check cache first
        if file_path in self._syntax_cache:
            return self._syntax_cache[file_path], "Cached result"
        
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                source_code = f.read()
            
            # Parse the AST to check syntax
            ast.parse(source_code, filename=file_path)
            
            # Cache successful result
            self._syntax_cache[file_path] = True
            return True, "Syntax is valid"
            
        except SyntaxError as e:
            error_msg = f"Syntax error at line {e.lineno}: {e.msg}"
            self._syntax_cache[file_path] = False
            return False, error_msg
            
        except UnicodeDecodeError as e:
            error_msg = f"Encoding error: {e}"
            return False, error_msg
            
        except Exception as e:
            error_msg = f"Unexpected error during syntax validation: {e}"
            return False, error_msg
    
    def _has_critical_imports(self, file_path: str) -> bool:
        """
        Check if the file contains imports that are critical to system operation.
        
        Args:
            file_path: Path to the file to check
            
        Returns:
            True if the file has critical imports
        """
        if not file_path.endswith('.py'):
            return False
        
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                source_code = f.read()
            
            tree = ast.parse(source_code)
            
            # Critical import patterns that suggest the file might be important
            critical_patterns = {
                'sys', 'os', 'logging', 'argparse', 'main',
                'app', 'server', 'config', 'settings'
            }
            
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    for alias in node.names:
                        if any(pattern in alias.name.lower() for pattern in critical_patterns):
                            return True
                            
                elif isinstance(node, ast.ImportFrom):
                    if node.module:
                        if any(pattern in node.module.lower() for pattern in critical_patterns):
                            return True
            
            return False
            
        except Exception as e:
            self.logger.warning(f"Could not analyze imports in {file_path}: {e}")
            # If we can't analyze, be conservative
            return True
    
    def check_syntax_after_move(self, moved_files: List[str]) -> Tuple[bool, List[str]]:
        """
        Check if the remaining codebase has valid syntax after files are moved.
        
        Args:
            moved_files: List of files that were moved
            
        Returns:
            Tuple of (all_valid, list_of_files_with_errors)
        """
        errors = []
        
        # Find all remaining Python files
        for py_file in self.project_root.rglob('*.py'):
            if str(py_file.resolve()) not in moved_files:
                is_valid, error_msg = self._validate_syntax(str(py_file))
                if not is_valid:
                    errors.append(f"{py_file}: {error_msg}")
        
        return len(errors) == 0, errors
    
    def validate_imports(self, entry_point: str) -> Tuple[bool, List[str]]:
        """
        Validate that imports in the entry point are still valid.
        
        Args:
            entry_point: Path to the entry point file (e.g., main.py)
            
        Returns:
            Tuple of (all_imports_valid, list_of_failed_imports)
        """
        entry_path = Path(entry_point)
        if not entry_path.is_absolute():
            entry_path = self.project_root / entry_point
        
        if not entry_path.exists():
            return False, [f"Entry point does not exist: {entry_point}"]
        
        failed_imports = []
        
        try:
            # Use Python's import system to validate imports
            spec = importlib.util.spec_from_file_location("__main__", entry_path)
            if spec is None:
                return False, [f"Could not create module spec for {entry_point}"]
            
            # Try to load the module (this will trigger import validation)
            module = importlib.util.module_from_spec(spec)
            
            # Add the project root to sys.path temporarily
            original_path = sys.path.copy()
            if str(self.project_root) not in sys.path:
                sys.path.insert(0, str(self.project_root))
            
            try:
                spec.loader.exec_module(module)
            except ImportError as e:
                failed_imports.append(f"Import error in {entry_point}: {e}")
            except Exception as e:
                # Other errors during module execution
                self.logger.warning(f"Error executing {entry_point}: {e}")
            finally:
                # Restore original sys.path
                sys.path = original_path
        
        except Exception as e:
            failed_imports.append(f"Failed to validate {entry_point}: {e}")
        
        return len(failed_imports) == 0, failed_imports
    
    def validate_project_integrity(self, moved_files: List[str]) -> Tuple[bool, Dict[str, List[str]]]:
        """
        Comprehensive validation of project integrity after files are moved.
        
        Args:
            moved_files: List of files that were moved
            
        Returns:
            Tuple of (is_valid, dict_of_validation_results)
        """
        results = {
            'syntax_errors': [],
            'import_errors': [],
            'missing_dependencies': []
        }
        
        # Check syntax of remaining files
        syntax_valid, syntax_errors = self.check_syntax_after_move(moved_files)
        results['syntax_errors'] = syntax_errors
        
        # Check imports for each entry point
        for entry_point in self.config.entry_points:
            imports_valid, import_errors = self.validate_imports(entry_point)
            if not imports_valid:
                results['import_errors'].extend(import_errors)
        
        # Overall validation result
        is_valid = (
            len(results['syntax_errors']) == 0 and
            len(results['import_errors']) == 0 and
            len(results['missing_dependencies']) == 0
        )
        
        return is_valid, results
    
    def get_safety_level(self, file_analysis: FileAnalysis) -> SafetyLevel:
        """
        Determine the safety level for a file based on analysis results.
        
        Args:
            file_analysis: Analysis results for the file
            
        Returns:
            SafetyLevel indicating how safe it is to move the file
        """
        # Critical files are never safe to move
        if self._is_critical_file(file_analysis.path):
            return SafetyLevel.CRITICAL
        
        # Files that are used should be reviewed carefully
        if file_analysis.is_used:
            return SafetyLevel.REVIEW
        
        # Files with many imports might be important
        if len(file_analysis.imports) > 10:
            return SafetyLevel.REVIEW
        
        # Files imported by many others might be important
        if len(file_analysis.imported_by) > 5:
            return SafetyLevel.REVIEW
        
        # Check if file can be safely moved
        is_safe, reason = self.is_safe_to_move(file_analysis.path)
        if not is_safe:
            return SafetyLevel.CRITICAL
        
        # If all checks pass, it's safe to move
        return SafetyLevel.SAFE