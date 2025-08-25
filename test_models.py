#!/usr/bin/env python3
"""
Test script to verify the core data models work correctly.
"""

from code_standards_formatter.models import (
    ImportBlocks, ImportInfo, FormatResult, FormatterConfig, ImportCategory
)
from code_standards_formatter.config import ConfigLoader


def test_import_blocks():
    """Test ImportBlocks functionality."""
    print("Testing ImportBlocks...")
    
    blocks = ImportBlocks()
    
    # Test adding imports to different categories
    blocks.add_import("import os", ImportCategory.STANDARD_LIBRARY)
    blocks.add_import("import numpy as np", ImportCategory.THIRD_PARTY)
    blocks.add_import("from omni.isaac import core", ImportCategory.ISAACSIM_RELATED)
    blocks.add_import("from .utils import helper", ImportCategory.LOCAL_IMPORTS)
    
    # Verify imports were added correctly
    assert len(blocks.standard_library) == 1
    assert len(blocks.third_party) == 1
    assert len(blocks.isaacsim_related) == 1
    assert len(blocks.local_imports) == 1
    
    print("✓ ImportBlocks test passed")


def test_import_info():
    """Test ImportInfo functionality."""
    print("Testing ImportInfo...")
    
    info = ImportInfo(
        module_name="numpy",
        import_type="import",
        imported_items=["numpy"],
        comment="# Scientific computing",
        original_line="import numpy as np  # Scientific computing"
    )
    
    assert info.module_name == "numpy"
    assert info.import_type == "import"
    assert info.comment == "# Scientific computing"
    
    print("✓ ImportInfo test passed")


def test_format_result():
    """Test FormatResult functionality."""
    print("Testing FormatResult...")
    
    result = FormatResult(
        file_path="test.py",
        changes_made=[],
        warnings=[],
        success=True
    )
    
    result.add_change("Organized imports")
    result.add_warning("Function missing type annotation")
    
    assert result.has_changes()
    assert result.has_warnings()
    assert len(result.changes_made) == 1
    assert len(result.warnings) == 1
    
    print("✓ FormatResult test passed")


def test_config_loading():
    """Test configuration loading."""
    print("Testing configuration loading...")
    
    # Test loading default config
    config = ConfigLoader.load_config()
    
    assert isinstance(config, FormatterConfig)
    assert config.spaces_before_inline_comment == 2
    assert config.blank_lines_after_imports == 2
    assert "os" in config.standard_library_modules
    assert "omni" in config.isaacsim_prefixes
    
    print("✓ Configuration loading test passed")


def test_sample_config_creation():
    """Test sample configuration file creation."""
    print("Testing sample config creation...")
    
    sample_path = ConfigLoader.create_sample_config("test_config.yaml")
    
    # Verify the file was created
    from pathlib import Path
    assert Path(sample_path).exists()
    
    # Clean up
    Path(sample_path).unlink()
    
    print("✓ Sample config creation test passed")


if __name__ == "__main__":
    print("Running tests for code standards formatter models...")
    print()
    
    test_import_blocks()
    test_import_info()
    test_format_result()
    test_config_loading()
    test_sample_config_creation()
    
    print()
    print("All tests passed! ✓")
    print("Project structure and core data models are working correctly.")