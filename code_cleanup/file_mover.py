"""
File mover component for safely moving unused files to trash directory.
"""
import os
import shutil
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict, List, Tuple
import logging
from .config import CleanupConfig
from .models import FileAnalysis


class FileMover:
    """Handles safe file moving operations with conflict resolution and atomic operations."""
    
    def __init__(self, config: CleanupConfig):
        """Initialize FileMover with configuration.
        
        Args:
            config: CleanupConfig instance containing project settings
        """
        self.config = config
        self.project_root = Path(config.project_root).resolve()
        self.trash_root = self.project_root / config.trash_directory
        self.moved_files: List[Tuple[str, str]] = []  # Track moves for rollback
        self.logger = logging.getLogger(__name__)
        
        # Ensure trash directory exists
        self.trash_root.mkdir(parents=True, exist_ok=True)
    
    def move_file(self, source_path: str, preserve_structure: bool = True) -> bool:
        """Move a file to trash directory with conflict resolution.
        
        Args:
            source_path: Path to the source file to move
            preserve_structure: Whether to preserve directory structure in trash
            
        Returns:
            bool: True if file was moved successfully, False otherwise
        """
        try:
            source = Path(source_path).resolve()
            
            # Validate source file exists
            if not source.exists():
                self.logger.warning(f"Source file does not exist: {source}")
                return False
            
            # Validate source is within project root
            if not self._is_within_project(source):
                self.logger.error(f"Source file is outside project root: {source}")
                return False
            
            # Calculate destination path
            if preserve_structure:
                relative_path = source.relative_to(self.project_root)
                destination = self.trash_root / relative_path
            else:
                destination = self.trash_root / source.name
            
            # Handle file conflicts
            destination = self._resolve_conflicts(destination)
            
            # Create destination directory
            destination.parent.mkdir(parents=True, exist_ok=True)
            
            # Create backup if configured
            backup_path = None
            if self.config.create_backup:
                backup_path = self._create_backup(source)
            
            # Perform atomic move
            success = self._atomic_move(source, destination)
            
            if success:
                self.moved_files.append((str(source), str(destination)))
                self.logger.info(f"Moved file: {source} -> {destination}")
                
                # Remove backup if move was successful and backup was created
                if backup_path and backup_path.exists():
                    try:
                        backup_path.unlink()
                    except Exception as e:
                        self.logger.warning(f"Failed to remove backup: {e}")
            else:
                # Restore from backup if move failed
                if backup_path and backup_path.exists():
                    try:
                        shutil.move(str(backup_path), str(source))
                        self.logger.info(f"Restored from backup: {source}")
                    except Exception as e:
                        self.logger.error(f"Failed to restore from backup: {e}")
            
            return success
            
        except Exception as e:
            self.logger.error(f"Error moving file {source_path}: {e}")
            return False
    
    def create_trash_structure(self, relative_path: str) -> str:
        """Create directory structure in trash to match original structure.
        
        Args:
            relative_path: Relative path from project root
            
        Returns:
            str: Full path to the created directory in trash
        """
        try:
            # Ensure relative_path is actually relative
            rel_path = Path(relative_path)
            if rel_path.is_absolute():
                # Convert absolute path to relative
                abs_path = rel_path.resolve()
                if self._is_within_project(abs_path):
                    rel_path = abs_path.relative_to(self.project_root)
                else:
                    raise ValueError(f"Path is outside project root: {relative_path}")
            
            # Create full trash path
            trash_path = self.trash_root / rel_path
            
            # Create directory structure
            if rel_path.suffix:  # It's a file path
                trash_path.parent.mkdir(parents=True, exist_ok=True)
                return str(trash_path.parent)
            else:  # It's a directory path
                trash_path.mkdir(parents=True, exist_ok=True)
                return str(trash_path)
                
        except Exception as e:
            self.logger.error(f"Error creating trash structure for {relative_path}: {e}")
            raise
    
    def move_files_batch(self, file_analyses: List[FileAnalysis]) -> Dict[str, bool]:
        """Move multiple files in batch with rollback capability.
        
        Args:
            file_analyses: List of FileAnalysis objects for files to move
            
        Returns:
            Dict[str, bool]: Mapping of file paths to success status
        """
        results = {}
        moved_in_batch = []
        
        try:
            for analysis in file_analyses:
                if analysis.is_used or analysis.safety_level.value == "critical":
                    self.logger.info(f"Skipping file (used or critical): {analysis.path}")
                    results[analysis.path] = False
                    continue
                
                success = self.move_file(analysis.path)
                results[analysis.path] = success
                
                if success:
                    moved_in_batch.append(analysis.path)
                else:
                    # If any file fails, consider rolling back the batch
                    self.logger.warning(f"Failed to move file in batch: {analysis.path}")
            
            return results
            
        except Exception as e:
            self.logger.error(f"Error in batch move operation: {e}")
            # Rollback files moved in this batch
            self._rollback_batch(moved_in_batch)
            return {path: False for path in [a.path for a in file_analyses]}
    
    def get_moved_files(self) -> List[Tuple[str, str]]:
        """Get list of files that have been moved.
        
        Returns:
            List[Tuple[str, str]]: List of (source, destination) tuples
        """
        return self.moved_files.copy()
    
    def _is_within_project(self, path: Path) -> bool:
        """Check if path is within project root."""
        try:
            path.resolve().relative_to(self.project_root)
            return True
        except ValueError:
            return False
    
    def _resolve_conflicts(self, destination: Path) -> Path:
        """Resolve file name conflicts by adding timestamp suffix.
        
        Args:
            destination: Proposed destination path
            
        Returns:
            Path: Conflict-free destination path
        """
        if not destination.exists():
            return destination
        
        # Generate timestamp suffix
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        stem = destination.stem
        suffix = destination.suffix
        
        # Try with timestamp
        new_name = f"{stem}_{timestamp}{suffix}"
        new_destination = destination.parent / new_name
        
        # If still conflicts, add counter
        counter = 1
        while new_destination.exists():
            new_name = f"{stem}_{timestamp}_{counter}{suffix}"
            new_destination = destination.parent / new_name
            counter += 1
        
        self.logger.info(f"Resolved conflict: {destination.name} -> {new_destination.name}")
        return new_destination
    
    def _create_backup(self, source: Path) -> Optional[Path]:
        """Create backup of source file.
        
        Args:
            source: Source file to backup
            
        Returns:
            Optional[Path]: Path to backup file, None if backup failed
        """
        try:
            backup_dir = self.project_root / ".cleanup_backup"
            backup_dir.mkdir(exist_ok=True)
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            backup_name = f"{source.stem}_{timestamp}{source.suffix}"
            backup_path = backup_dir / backup_name
            
            shutil.copy2(str(source), str(backup_path))
            self.logger.debug(f"Created backup: {backup_path}")
            return backup_path
            
        except Exception as e:
            self.logger.warning(f"Failed to create backup for {source}: {e}")
            return None
    
    def _atomic_move(self, source: Path, destination: Path) -> bool:
        """Perform atomic file move operation.
        
        Args:
            source: Source file path
            destination: Destination file path
            
        Returns:
            bool: True if move was successful
        """
        try:
            # Use os.rename for atomic operation on same filesystem
            # Fall back to shutil.move for cross-filesystem moves
            try:
                os.rename(str(source), str(destination))
                return True
            except OSError:
                # Cross-filesystem move or other OS error
                shutil.move(str(source), str(destination))
                return True
                
        except Exception as e:
            self.logger.error(f"Atomic move failed {source} -> {destination}: {e}")
            return False
    
    def _rollback_batch(self, moved_files: List[str]):
        """Rollback files moved in current batch.
        
        Args:
            moved_files: List of source file paths that were moved
        """
        self.logger.warning("Rolling back batch move operation")
        
        for source_path in moved_files:
            # Find the corresponding move record
            for source, destination in reversed(self.moved_files):
                if source == source_path:
                    try:
                        # Move back from trash to original location
                        shutil.move(destination, source)
                        self.moved_files.remove((source, destination))
                        self.logger.info(f"Rolled back: {destination} -> {source}")
                    except Exception as e:
                        self.logger.error(f"Failed to rollback {destination}: {e}")
                    break