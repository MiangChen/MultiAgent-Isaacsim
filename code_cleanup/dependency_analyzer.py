"""
Dependency analyzer for Python code cleanup system.

This module provides functionality to analyze Python file dependencies using AST parsing,
build dependency graphs, and identify which files are actually used in the project.
"""
import ast
import os
import sys
import re
from pathlib import Path
from typing import Dict, List, Set, Optional, Tuple
import logging
from dataclasses import dataclass

from code_cleanup.config import CleanupConfig
from code_cleanup.models import FileAnalysis, FileType, SafetyLevel


@dataclass
class ImportInfo:
    """Information about an import statement."""
    module_name: str
    is_relative: bool
    level: int = 0  # For relative imports (number of dots)
    imported_names: List[str] = None  # For 'from x import y' statements
    alias: Optional[str] = None  # For 'import x as y' statements
    
    def __post_init__(self):
        if self.imported_names is None:
            self.imported_names = []


class DependencyAnalyzer:
    """
    Analyzes Python file dependencies using AST parsing.
    
    This class can scan Python files, extract import statements, resolve import paths,
    and build a dependency graph to identify which files are actually used.
    """
    
    def __init__(self, config: CleanupConfig):
        """
        Initialize the dependency analyzer.
        
        Args:
            config: Cleanup configuration containing project settings
        """
        self.config = config
        self.project_root = Path(config.project_root).resolve()
        self.logger = logging.getLogger(__name__)
        
        # Cache for parsed files to avoid re-parsing
        self._ast_cache: Dict[str, ast.AST] = {}
        self._import_cache: Dict[str, List[ImportInfo]] = {}
        
        # Dependency graph: file_path -> list of files it imports
        self._dependency_graph: Dict[str, List[str]] = {}
        
        # Reverse dependency graph: file_path -> list of files that import it
        self._reverse_dependency_graph: Dict[str, List[str]] = {}
        
        # Set of all Python files in the project
        self._all_python_files: Set[str] = set()
        
        self.logger.info(f"Initialized DependencyAnalyzer for project: {self.project_root}")
    
    def scan_imports(self, file_path: str) -> List[ImportInfo]:
        """
        Extract import statements from a Python file using AST parsing.
        
        Args:
            file_path: Path to the Python file to analyze
            
        Returns:
            List of ImportInfo objects representing the imports in the file
            
        Raises:
            FileNotFoundError: If the file doesn't exist
            SyntaxError: If the file has syntax errors
        """
        file_path = str(Path(file_path).resolve())
        
        # Check cache first
        if file_path in self._import_cache:
            return self._import_cache[file_path]
        
        if not Path(file_path).exists():
            raise FileNotFoundError(f"File not found: {file_path}")
        
        try:
            # Parse the file using AST
            tree = self._parse_file(file_path)
            imports = []
            
            # Walk the AST to find import statements
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    # Handle 'import module' statements
                    for alias in node.names:
                        import_info = ImportInfo(
                            module_name=alias.name,
                            is_relative=False,
                            alias=alias.asname
                        )
                        imports.append(import_info)
                
                elif isinstance(node, ast.ImportFrom):
                    # Handle 'from module import name' statements
                    if node.module is None:
                        # Handle 'from . import name' cases
                        module_name = ""
                    else:
                        module_name = node.module
                    
                    imported_names = [alias.name for alias in node.names]
                    
                    import_info = ImportInfo(
                        module_name=module_name,
                        is_relative=node.level > 0,
                        level=node.level,
                        imported_names=imported_names
                    )
                    imports.append(import_info)
            
            # Cache the results
            self._import_cache[file_path] = imports
            
            self.logger.debug(f"Found {len(imports)} imports in {file_path}")
            return imports
            
        except SyntaxError as e:
            self.logger.warning(f"Syntax error in {file_path}: {e}")
            # Return empty list for files with syntax errors
            return []
        except Exception as e:
            self.logger.error(f"Error parsing {file_path}: {e}")
            return []
    
    def _parse_file(self, file_path: str) -> ast.AST:
        """
        Parse a Python file and return its AST.
        
        Args:
            file_path: Path to the Python file
            
        Returns:
            AST tree for the file
        """
        # Check cache first
        if file_path in self._ast_cache:
            return self._ast_cache[file_path]
        
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            tree = ast.parse(content, filename=file_path)
            self._ast_cache[file_path] = tree
            return tree
            
        except UnicodeDecodeError:
            # Try with different encoding
            with open(file_path, 'r', encoding='latin-1') as f:
                content = f.read()
            
            tree = ast.parse(content, filename=file_path)
            self._ast_cache[file_path] = tree
            return tree
    
    def resolve_import_path(self, import_info: ImportInfo, current_file: str) -> Optional[str]:
        """
        Resolve an import statement to an actual file path.
        
        Args:
            import_info: Information about the import statement
            current_file: Path of the file containing the import
            
        Returns:
            Resolved file path if found, None otherwise
        """
        current_dir = Path(current_file).parent
        
        if import_info.is_relative:
            # Handle relative imports
            return self._resolve_relative_import(import_info, current_dir)
        else:
            # Handle absolute imports
            return self._resolve_absolute_import(import_info, current_dir)
    
    def _resolve_relative_import(self, import_info: ImportInfo, current_dir: Path) -> Optional[str]:
        """
        Resolve a relative import to a file path.
        
        Args:
            import_info: Import information with relative import details
            current_dir: Directory of the current file
            
        Returns:
            Resolved file path if found, None otherwise
        """
        # Go up 'level' directories
        target_dir = current_dir
        for _ in range(import_info.level):
            target_dir = target_dir.parent
        
        if import_info.module_name:
            # from ..module import something
            module_parts = import_info.module_name.split('.')
            for part in module_parts:
                target_dir = target_dir / part
        
        # Try to find the actual file
        return self._find_module_file(target_dir)
    
    def _resolve_absolute_import(self, import_info: ImportInfo, current_dir: Path) -> Optional[str]:
        """
        Resolve an absolute import to a file path.
        
        Args:
            import_info: Import information with absolute import details
            current_dir: Directory of the current file
            
        Returns:
            Resolved file path if found, None otherwise
        """
        module_parts = import_info.module_name.split('.')
        
        # Try resolving from project root first
        target_path = self.project_root
        for part in module_parts:
            target_path = target_path / part
        
        resolved_path = self._find_module_file(target_path)
        if resolved_path:
            return resolved_path
        
        # Try resolving from current directory (for local modules)
        target_path = current_dir
        for part in module_parts:
            target_path = target_path / part
        
        return self._find_module_file(target_path)
    
    def _find_module_file(self, module_path: Path) -> Optional[str]:
        """
        Find the actual Python file for a module path.
        
        Args:
            module_path: Path where the module should be located
            
        Returns:
            Actual file path if found, None otherwise
        """
        # Try as a Python file
        py_file = module_path.with_suffix('.py')
        if py_file.exists() and py_file.is_file():
            return str(py_file.resolve())
        
        # Try as a package (directory with __init__.py)
        if module_path.is_dir():
            init_file = module_path / '__init__.py'
            if init_file.exists():
                return str(init_file.resolve())
        
        return None
    
    def build_dependency_graph(self) -> Dict[str, List[str]]:
        """
        Build a dependency graph for all Python files in the project.
        
        Returns:
            Dictionary mapping file paths to lists of files they depend on
        """
        self.logger.info("Building dependency graph...")
        
        # First, find all Python files in the project
        self._discover_python_files()
        
        # Clear existing graphs
        self._dependency_graph.clear()
        self._reverse_dependency_graph.clear()
        
        # Analyze each file
        for file_path in self._all_python_files:
            if self.config.should_exclude(file_path):
                continue
            
            try:
                imports = self.scan_imports(file_path)
                dependencies = []
                
                for import_info in imports:
                    resolved_path = self.resolve_import_path(import_info, file_path)
                    if resolved_path and resolved_path in self._all_python_files:
                        dependencies.append(resolved_path)
                        
                        # Update reverse dependency graph
                        if resolved_path not in self._reverse_dependency_graph:
                            self._reverse_dependency_graph[resolved_path] = []
                        self._reverse_dependency_graph[resolved_path].append(file_path)
                
                self._dependency_graph[file_path] = dependencies
                
            except Exception as e:
                self.logger.error(f"Error analyzing dependencies for {file_path}: {e}")
                self._dependency_graph[file_path] = []
        
        self.logger.info(f"Built dependency graph with {len(self._dependency_graph)} files")
        return self._dependency_graph.copy()
    
    def _discover_python_files(self):
        """Discover all Python files in the project."""
        self.logger.debug("Discovering Python files...")
        
        self._all_python_files.clear()
        
        for root, dirs, files in os.walk(self.project_root):
            # Skip excluded directories
            dirs[:] = [d for d in dirs if not self.config.should_exclude(os.path.join(root, d))]
            
            for file in files:
                if file.endswith('.py'):
                    file_path = os.path.join(root, file)
                    if not self.config.should_exclude(file_path):
                        self._all_python_files.add(str(Path(file_path).resolve()))
        
        self.logger.debug(f"Discovered {len(self._all_python_files)} Python files")
    
    def find_used_files(self, entry_points: List[str]) -> Set[str]:
        """
        Find all files that are used (directly or indirectly) by the entry points.
        
        This method performs a comprehensive analysis including:
        - Static import analysis
        - Dynamic import detection
        - Conditional import handling
        - Circular dependency detection
        
        Args:
            entry_points: List of entry point file paths
            
        Returns:
            Set of file paths that are used by the entry points
        """
        self.logger.info(f"Finding used files from {len(entry_points)} entry points...")
        
        # Ensure dependency graph is built
        if not self._dependency_graph:
            self.build_dependency_graph()
        
        used_files = set()
        to_visit = []
        visited_stack = []  # For circular dependency detection
        
        # Add entry points to the queue
        for entry_point in entry_points:
            entry_path = self.project_root / entry_point
            if entry_path.exists():
                resolved_path = str(entry_path.resolve())
                if resolved_path in self._all_python_files:
                    to_visit.append(resolved_path)
                    used_files.add(resolved_path)
                    self.logger.debug(f"Added entry point: {entry_point}")
        
        # Perform depth-first search to find all dependencies
        while to_visit:
            current_file = to_visit.pop()
            
            # Check for circular dependencies
            if current_file in visited_stack:
                self.logger.warning(f"Circular dependency detected involving: {current_file}")
                continue
            
            visited_stack.append(current_file)
            
            # Get static dependencies
            static_deps = self._dependency_graph.get(current_file, [])
            
            # Get dynamic and conditional imports
            dynamic_deps = self._find_dynamic_imports(current_file)
            
            # Combine all dependencies
            all_deps = set(static_deps) | set(dynamic_deps)
            
            for dep_file in all_deps:
                if dep_file not in used_files and dep_file in self._all_python_files:
                    used_files.add(dep_file)
                    to_visit.append(dep_file)
                    self.logger.debug(f"Added dependency: {dep_file}")
            
            # Remove from visited stack when done processing
            visited_stack.remove(current_file)
        
        self.logger.info(f"Found {len(used_files)} used files")
        return used_files
    
    def get_dependency_graph(self) -> Dict[str, List[str]]:
        """Get the current dependency graph."""
        return self._dependency_graph.copy()
    
    def get_reverse_dependency_graph(self) -> Dict[str, List[str]]:
        """Get the reverse dependency graph (who imports what)."""
        return self._reverse_dependency_graph.copy()
    
    def get_all_python_files(self) -> Set[str]:
        """Get all discovered Python files."""
        return self._all_python_files.copy()
    
    def _find_dynamic_imports(self, file_path: str) -> List[str]:
        """
        Find dynamic and conditional imports in a Python file.
        
        This method looks for:
        - importlib.import_module() calls
        - __import__() calls
        - exec() and eval() with import statements
        - String-based imports in conditional blocks
        
        Args:
            file_path: Path to the Python file to analyze
            
        Returns:
            List of resolved file paths for dynamic imports
        """
        dynamic_imports = []
        
        try:
            tree = self._parse_file(file_path)
            
            # Walk the AST to find dynamic import patterns
            for node in ast.walk(tree):
                # Look for importlib.import_module calls
                if isinstance(node, ast.Call):
                    if self._is_importlib_call(node):
                        module_name = self._extract_module_name_from_call(node)
                        if module_name:
                            resolved_path = self._resolve_module_name_to_path(module_name, file_path)
                            if resolved_path:
                                dynamic_imports.append(resolved_path)
                    
                    # Look for __import__ calls
                    elif self._is_builtin_import_call(node):
                        module_name = self._extract_module_name_from_call(node)
                        if module_name:
                            resolved_path = self._resolve_module_name_to_path(module_name, file_path)
                            if resolved_path:
                                dynamic_imports.append(resolved_path)
                
                # Look for string literals that might be module names in conditional imports
                elif isinstance(node, ast.If):
                    conditional_imports = self._find_conditional_imports(node, file_path)
                    dynamic_imports.extend(conditional_imports)
            
            # Also scan for string patterns that look like module paths
            string_imports = self._find_string_based_imports(file_path)
            dynamic_imports.extend(string_imports)
            
        except Exception as e:
            self.logger.warning(f"Error finding dynamic imports in {file_path}: {e}")
        
        return dynamic_imports
    
    def _is_importlib_call(self, node: ast.Call) -> bool:
        """Check if a call node is an importlib.import_module call."""
        if isinstance(node.func, ast.Attribute):
            if isinstance(node.func.value, ast.Name):
                return (node.func.value.id == 'importlib' and 
                       node.func.attr == 'import_module')
            elif isinstance(node.func.value, ast.Attribute):
                return (node.func.value.attr == 'importlib' and 
                       node.func.attr == 'import_module')
        return False
    
    def _is_builtin_import_call(self, node: ast.Call) -> bool:
        """Check if a call node is a __import__ call."""
        return (isinstance(node.func, ast.Name) and 
                node.func.id == '__import__')
    
    def _extract_module_name_from_call(self, node: ast.Call) -> Optional[str]:
        """Extract module name from import call arguments."""
        if node.args and isinstance(node.args[0], ast.Constant):
            return node.args[0].value
        elif node.args and isinstance(node.args[0], ast.Str):  # Python < 3.8
            return node.args[0].s
        return None
    
    def _find_conditional_imports(self, if_node: ast.If, file_path: str) -> List[str]:
        """Find imports within conditional blocks."""
        conditional_imports = []
        
        # Look for import statements in the if block
        for node in ast.walk(if_node):
            if isinstance(node, (ast.Import, ast.ImportFrom)):
                # Process the import as if it were at module level
                if isinstance(node, ast.Import):
                    for alias in node.names:
                        import_info = ImportInfo(
                            module_name=alias.name,
                            is_relative=False,
                            alias=alias.asname
                        )
                        resolved_path = self.resolve_import_path(import_info, file_path)
                        if resolved_path:
                            conditional_imports.append(resolved_path)
                
                elif isinstance(node, ast.ImportFrom):
                    module_name = node.module or ""
                    import_info = ImportInfo(
                        module_name=module_name,
                        is_relative=node.level > 0,
                        level=node.level,
                        imported_names=[alias.name for alias in node.names]
                    )
                    resolved_path = self.resolve_import_path(import_info, file_path)
                    if resolved_path:
                        conditional_imports.append(resolved_path)
        
        return conditional_imports
    
    def _find_string_based_imports(self, file_path: str) -> List[str]:
        """
        Find potential module names in string literals that might be used for dynamic imports.
        
        This is a heuristic approach that looks for strings that match common module patterns.
        """
        string_imports = []
        
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # Look for common patterns that might indicate dynamic imports
            import re
            
            # Pattern for strings that look like module paths
            module_patterns = [
                r'["\']([a-zA-Z_][a-zA-Z0-9_]*(?:\.[a-zA-Z_][a-zA-Z0-9_]*)*)["\']',
                r'["\'](\.[a-zA-Z_][a-zA-Z0-9_]*(?:\.[a-zA-Z_][a-zA-Z0-9_]*)*)["\']',
            ]
            
            for pattern in module_patterns:
                matches = re.findall(pattern, content)
                for match in matches:
                    # Only consider strings that look like valid module names
                    if self._looks_like_module_name(match):
                        resolved_path = self._resolve_module_name_to_path(match, file_path)
                        if resolved_path:
                            string_imports.append(resolved_path)
        
        except Exception as e:
            self.logger.debug(f"Error scanning strings in {file_path}: {e}")
        
        return string_imports
    
    def _looks_like_module_name(self, string: str) -> bool:
        """Check if a string looks like a valid module name."""
        if not string:
            return False
        
        # Must start with letter or underscore
        if not (string[0].isalpha() or string[0] == '_' or string[0] == '.'):
            return False
        
        # Should contain only valid module name characters
        import re
        if not re.match(r'^\.?[a-zA-Z_][a-zA-Z0-9_]*(?:\.[a-zA-Z_][a-zA-Z0-9_]*)*$', string):
            return False
        
        # Exclude common non-module strings
        excluded_patterns = [
            'main', 'test', 'debug', 'info', 'error', 'warning',
            'true', 'false', 'none', 'null', 'undefined'
        ]
        
        if string.lower() in excluded_patterns:
            return False
        
        return True
    
    def _resolve_module_name_to_path(self, module_name: str, current_file: str) -> Optional[str]:
        """Resolve a module name string to an actual file path."""
        # Create a temporary ImportInfo object to reuse existing resolution logic
        import_info = ImportInfo(
            module_name=module_name,
            is_relative=module_name.startswith('.'),
            level=len(module_name) - len(module_name.lstrip('.')) if module_name.startswith('.') else 0
        )
        
        # Clean the module name if it's relative
        if import_info.is_relative:
            import_info.module_name = module_name.lstrip('.')
        
        return self.resolve_import_path(import_info, current_file)
    
    def detect_circular_dependencies(self) -> List[List[str]]:
        """
        Detect circular dependencies in the dependency graph.
        
        Returns:
            List of cycles, where each cycle is a list of file paths
        """
        self.logger.info("Detecting circular dependencies...")
        
        if not self._dependency_graph:
            self.build_dependency_graph()
        
        cycles = []
        visited = set()
        rec_stack = set()
        path_stack = []
        
        def dfs(node: str) -> bool:
            """DFS helper function to detect cycles."""
            if node in rec_stack:
                # Found a cycle - extract it from the path stack
                cycle_start = path_stack.index(node)
                cycle = path_stack[cycle_start:] + [node]
                cycles.append(cycle)
                return True
            
            if node in visited:
                return False
            
            visited.add(node)
            rec_stack.add(node)
            path_stack.append(node)
            
            # Visit all dependencies
            for neighbor in self._dependency_graph.get(node, []):
                if dfs(neighbor):
                    return True
            
            rec_stack.remove(node)
            path_stack.pop()
            return False
        
        # Check all nodes for cycles
        for node in self._dependency_graph:
            if node not in visited:
                dfs(node)
        
        if cycles:
            self.logger.warning(f"Found {len(cycles)} circular dependencies")
            for i, cycle in enumerate(cycles):
                self.logger.warning(f"Cycle {i+1}: {' -> '.join(cycle)}")
        else:
            self.logger.info("No circular dependencies found")
        
        return cycles
    
    def classify_file(self, file_path: str) -> FileType:
        """
        Classify a file based on its path and content.
        
        Args:
            file_path: Path to the file to classify
            
        Returns:
            FileType classification for the file
        """
        file_path = Path(file_path)
        path_str = str(file_path).lower()
        
        # Check if it's a critical core file
        if file_path.name in ['main.py', 'containers.py', '__init__.py']:
            return FileType.CORE
        
        # Classify based on directory structure
        if 'robot' in path_str:
            return FileType.ROBOT
        elif 'controller' in path_str:
            return FileType.CONTROLLER
        elif 'utils' in path_str or 'util' in path_str:
            return FileType.UTILS
        elif 'map' in path_str:
            return FileType.MAP
        elif 'camera' in path_str:
            return FileType.CAMERA
        elif 'path_planning' in path_str or 'planning' in path_str:
            return FileType.PATH_PLANNING
        else:
            return FileType.OTHER
    
    def determine_safety_level(self, file_path: str, is_used: bool) -> SafetyLevel:
        """
        Determine the safety level for moving a file.
        
        Args:
            file_path: Path to the file
            is_used: Whether the file is currently used
            
        Returns:
            SafetyLevel for the file
        """
        file_path = Path(file_path)
        
        # Critical files should never be moved
        if file_path.name in self.config.critical_files:
            return SafetyLevel.CRITICAL
        
        # Files in critical directories
        if any(critical_dir in str(file_path).lower() 
               for critical_dir in ['__pycache__', '.git', '.kiro']):
            return SafetyLevel.CRITICAL
        
        # If file is used, it's generally safe to keep
        if is_used:
            return SafetyLevel.SAFE
        
        # Unused files need review based on their type
        file_type = self.classify_file(str(file_path))
        
        if file_type == FileType.CORE:
            return SafetyLevel.CRITICAL
        elif file_type in [FileType.UTILS, FileType.OTHER]:
            return SafetyLevel.SAFE  # Usually safe to move utility files
        else:
            return SafetyLevel.REVIEW  # Domain-specific files need review
    
    def analyze_all_files(self) -> Dict[str, FileAnalysis]:
        """
        Perform comprehensive analysis of all files in the project.
        
        Returns:
            Dictionary mapping file paths to their analysis results
        """
        self.logger.info("Performing comprehensive file analysis...")
        
        # Build dependency graph if not already built
        if not self._dependency_graph:
            self.build_dependency_graph()
        
        # Find used files
        used_files = self.find_used_files(self.config.entry_points)
        
        # Detect circular dependencies
        circular_deps = self.detect_circular_dependencies()
        
        # Analyze each file
        analysis_results = {}
        
        for file_path in self._all_python_files:
            is_used = file_path in used_files
            file_type = self.classify_file(file_path)
            safety_level = self.determine_safety_level(file_path, is_used)
            
            # Get import information
            imports = [dep for dep in self._dependency_graph.get(file_path, [])]
            imported_by = self._reverse_dependency_graph.get(file_path, [])
            
            analysis = FileAnalysis(
                path=file_path,
                is_used=is_used,
                imported_by=imported_by.copy(),
                imports=imports.copy(),
                file_type=file_type,
                safety_level=safety_level
            )
            
            analysis_results[file_path] = analysis
        
        self.logger.info(f"Analyzed {len(analysis_results)} files")
        return analysis_results
    
    def clear_cache(self):
        """Clear all internal caches."""
        self._ast_cache.clear()
        self._import_cache.clear()
        self._dependency_graph.clear()
        self._reverse_dependency_graph.clear()
        self._all_python_files.clear()
        self.logger.debug("Cleared all caches")