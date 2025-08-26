"""
Unit tests for the FileScanner class.

Tests file discovery, exclusion patterns, and Python file detection.
"""

import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch, MagicMock

from code_standards_formatter.core.file_scanner import FileScanner
from code_standards_formatter.models import FormatterConfig


class TestFileScanner(unittest.TestCase):
    """Test cases for the FileScanner class."""
    
    def setUp(self):
        """Set up test fixtures before each test method."""
        self.temp_dir = tempfile.mkdtemp()
        self.temp_path = Path(self.temp_dir)
        
        # Create a test directory structure
        self.create_test_structure()
        
        # Create scanner with default config
        self.scanner = FileScanner()
    
    def tearDown(self):
        """Clean up after each test method."""
        import shutil
        shutil.rmtree(self.temp_dir, ignore_errors=True)
    
    def create_test_structure(self):
        """Create a test directory structure with various file types."""
        # Create directories
        (self.temp_path / "src").mkdir()
        (self.temp_path / "tests").mkdir()
        (self.temp_path / "__pycache__").mkdir()
        (self.temp_path / ".git").mkdir()
        (self.temp_path / "venv").mkdir()
        (self.temp_path / "src" / "submodule").mkdir()
        
        # Create Python files
        (self.temp_path / "main.py").write_text("# Main module")
        (self.temp_path / "setup.py").write_text("# Setup script")
        (self.temp_path / "src" / "module1.py").write_text("# Module 1")
        (self.temp_path / "src" / "module2.py").write_text("# Module 2")
        (self.temp_path / "src" / "submodule" / "submod.py").write_text("# Submodule")
        (self.temp_path / "tests" / "test_main.py").write_text("# Test file")
        
        # Create non-Python files
        (self.temp_path / "README.md").write_text("# README")
        (self.temp_path / "config.yaml").write_text("config: value")
        (self.temp_path / "data.json").write_text('{"key": "value"}')
        
        # Create files in excluded directories
        (self.temp_path / "__pycache__" / "module.pyc").write_text("compiled")
        (self.temp_path / ".git" / "config").write_text("git config")
        (self.temp_path / "venv" / "lib" / "python.py").mkdir(parents=True)
        (self.temp_path / "venv" / "lib" / "python.py").write_text("venv python")
    
    def test_scan_python_files_basic(self):
        """Test basic Python file scanning functionality."""
        files = self.scanner.scan_python_files(str(self.temp_path))
        
        # Should find Python files but exclude those in excluded directories
        expected_files = {
            "main.py", "setup.py", "module1.py", "module2.py", 
            "submod.py", "test_main.py"
        }
        
        found_filenames = {Path(f).name for f in files}
        self.assertEqual(found_filenames, expected_files)
    
    def test_scan_python_files_with_custom_exclusions(self):
        """Test scanning with custom exclusion patterns."""
        exclude_patterns = ["test_*", "setup.py"]
        files = self.scanner.scan_python_files(str(self.temp_path), exclude_patterns)
        
        # Should exclude test files and setup.py
        found_filenames = {Path(f).name for f in files}
        self.assertNotIn("test_main.py", found_filenames)
        self.assertNotIn("setup.py", found_filenames)
        self.assertIn("main.py", found_filenames)
    
    def test_scan_nonexistent_directory(self):
        """Test scanning a directory that doesn't exist."""
        with self.assertRaises(ValueError) as context:
            self.scanner.scan_python_files("/nonexistent/path")
        
        self.assertIn("does not exist", str(context.exception))
    
    def test_scan_file_instead_of_directory(self):
        """Test scanning when given a file path instead of directory."""
        file_path = self.temp_path / "main.py"
        
        with self.assertRaises(ValueError) as context:
            self.scanner.scan_python_files(str(file_path))
        
        self.assertIn("not a directory", str(context.exception))
    
    def test_is_python_file(self):
        """Test Python file detection."""
        # Test valid Python files
        self.assertTrue(self.scanner.is_python_file(str(self.temp_path / "main.py")))
        self.assertTrue(self.scanner.is_python_file(str(self.temp_path / "setup.py")))
        
        # Test non-Python files
        self.assertFalse(self.scanner.is_python_file(str(self.temp_path / "README.md")))
        self.assertFalse(self.scanner.is_python_file(str(self.temp_path / "config.yaml")))
        
        # Test non-existent file
        self.assertFalse(self.scanner.is_python_file("/nonexistent/file.py"))
        
        # Test directory
        self.assertFalse(self.scanner.is_python_file(str(self.temp_path / "src")))
    
    def test_should_exclude_patterns(self):
        """Test exclusion pattern matching."""
        exclude_patterns = ["__pycache__", "*.pyc", "test_*", ".git"]
        
        # Test directory exclusions
        self.assertTrue(self.scanner.should_exclude(
            str(self.temp_path / "__pycache__"), exclude_patterns))
        self.assertTrue(self.scanner.should_exclude(
            str(self.temp_path / ".git"), exclude_patterns))
        
        # Test file pattern exclusions
        self.assertTrue(self.scanner.should_exclude(
            str(self.temp_path / "module.pyc"), exclude_patterns))
        self.assertTrue(self.scanner.should_exclude(
            str(self.temp_path / "test_something.py"), exclude_patterns))
        
        # Test files that should not be excluded
        self.assertFalse(self.scanner.should_exclude(
            str(self.temp_path / "main.py"), exclude_patterns))
        self.assertFalse(self.scanner.should_exclude(
            str(self.temp_path / "src"), exclude_patterns))
    
    def test_should_exclude_nested_paths(self):
        """Test exclusion of files in nested excluded directories."""
        exclude_patterns = ["venv", "__pycache__"]
        
        # Files in excluded directories should be excluded
        nested_file = self.temp_path / "venv" / "lib" / "python.py"
        self.assertTrue(self.scanner.should_exclude(str(nested_file), exclude_patterns))
        
        cache_file = self.temp_path / "__pycache__" / "module.pyc"
        self.assertTrue(self.scanner.should_exclude(str(cache_file), exclude_patterns))
    
    def test_get_file_count_estimate(self):
        """Test file count estimation."""
        count = self.scanner.get_file_count_estimate(str(self.temp_path))
        self.assertGreater(count, 0)
        self.assertIsInstance(count, int)
        
        # Test with nonexistent directory
        count = self.scanner.get_file_count_estimate("/nonexistent")
        self.assertEqual(count, 0)
    
    def test_validate_paths(self):
        """Test path validation functionality."""
        test_paths = [
            str(self.temp_path / "main.py"),  # Valid Python file
            str(self.temp_path / "README.md"),  # Valid file, not Python
            "/nonexistent/file.py",  # Nonexistent file
            str(self.temp_path / "src"),  # Directory
        ]
        
        valid_paths = self.scanner.validate_paths(test_paths)
        
        # Should only return the valid Python file
        self.assertEqual(len(valid_paths), 1)
        self.assertTrue(valid_paths[0].endswith("main.py"))
    
    def test_find_python_files_by_pattern(self):
        """Test finding files by name pattern."""
        # Find all test files
        test_files = self.scanner.find_python_files_by_pattern(
            str(self.temp_path), "test_*")
        
        found_names = {Path(f).name for f in test_files}
        self.assertIn("test_main.py", found_names)
        self.assertNotIn("main.py", found_names)
        
        # Find setup files
        setup_files = self.scanner.find_python_files_by_pattern(
            str(self.temp_path), "setup.py")
        
        self.assertEqual(len(setup_files), 1)
        self.assertTrue(setup_files[0].endswith("setup.py"))
    
    def test_scanner_with_config(self):
        """Test scanner initialization with custom config."""
        config = FormatterConfig(exclude_patterns=["custom_*", "special"])
        scanner = FileScanner(config)
        
        # Create files matching custom patterns
        (self.temp_path / "custom_module.py").write_text("# Custom module")
        (self.temp_path / "special").mkdir()
        (self.temp_path / "special" / "file.py").write_text("# Special file")
        
        files = scanner.scan_python_files(str(self.temp_path))
        found_names = {Path(f).name for f in files}
        
        # Custom exclusions should be applied
        self.assertNotIn("custom_module.py", found_names)
        self.assertNotIn("file.py", found_names)  # In excluded directory
    
    def test_permission_error_handling(self):
        """Test handling of permission errors during directory traversal."""
        with patch('pathlib.Path.iterdir') as mock_iterdir:
            mock_iterdir.side_effect = PermissionError("Access denied")
            
            # Should not raise exception, but print warning
            with patch('builtins.print') as mock_print:
                files = self.scanner.scan_python_files(str(self.temp_path))
                
                # Should handle the error gracefully
                self.assertIsInstance(files, list)
                mock_print.assert_called()
    
    def test_empty_directory(self):
        """Test scanning an empty directory."""
        empty_dir = self.temp_path / "empty"
        empty_dir.mkdir()
        
        files = self.scanner.scan_python_files(str(empty_dir))
        self.assertEqual(files, [])
    
    def test_file_ordering(self):
        """Test that files are returned in sorted order."""
        files = self.scanner.scan_python_files(str(self.temp_path))
        
        # Files should be sorted
        self.assertEqual(files, sorted(files))
    
    def test_python_extensions(self):
        """Test recognition of different Python file extensions."""
        # Create files with different extensions
        (self.temp_path / "script.pyw").write_text("# Windows Python script")
        (self.temp_path / "module.PY").write_text("# Uppercase extension")
        
        # Test .pyw files
        self.assertTrue(self.scanner.is_python_file(str(self.temp_path / "script.pyw")))
        
        # Test case insensitive matching
        self.assertTrue(self.scanner.is_python_file(str(self.temp_path / "module.PY")))


class TestFileScannerIntegration(unittest.TestCase):
    """Integration tests for FileScanner with real project structure."""
    
    def test_scan_current_project(self):
        """Test scanning the current project directory."""
        scanner = FileScanner()
        
        # Scan the current directory (should find this test file)
        current_dir = Path.cwd()
        files = scanner.scan_python_files(str(current_dir))
        
        # Should find Python files including this test file
        self.assertGreater(len(files), 0)
        
        # Verify all returned files are actually Python files
        for file_path in files:
            self.assertTrue(scanner.is_python_file(file_path))
    
    def test_exclusion_patterns_in_real_project(self):
        """Test that common exclusion patterns work in a real project."""
        config = FormatterConfig(exclude_patterns=[
            "__pycache__", ".git", "venv", ".venv", "*.pyc", 
            ".pytest_cache", "build", "dist"
        ])
        scanner = FileScanner(config)
        
        current_dir = Path.cwd()
        files = scanner.scan_python_files(str(current_dir))
        
        # Verify no excluded directories appear in results
        for file_path in files:
            path = Path(file_path)
            path_parts = [part.name for part in path.parents] + [path.name]
            
            # Should not contain excluded directory names
            excluded_dirs = {"__pycache__", ".git", "venv", ".venv", 
                           ".pytest_cache", "build", "dist"}
            
            for part in path_parts:
                if part in excluded_dirs:
                    self.fail(f"Found file in excluded directory: {file_path}")


if __name__ == '__main__':
    unittest.main()