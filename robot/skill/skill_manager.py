# =============================================================================
# Skill Manager Module - Robot Skill Management System
# =============================================================================
#
# This module provides a skill management system for robots.
# It focuses on core functionality with minimal complexity.
#
# =============================================================================

# Standard library imports
import threading
import time
from enum import Enum
from typing import Dict, Any, Optional, Callable

# Local project imports
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class SkillState(Enum):
    """Skill execution state enumeration"""

    IDLE = "idle"  # Idle state, ready to accept new tasks
    RUNNING = "running"  # Currently executing
    FAILED = "failed"  # Execution failed (will automatically return to IDLE)


class SkillManager:
    """Robot skill management system"""

    def __init__(self, robot_instance=None):
        """Initialize skill manager"""
        self.robot = robot_instance
        self.state = SkillState.IDLE
        self.current_skill_name = ""
        self.current_skill_args = {}
        self.current_thread: Optional[threading.Thread] = None
        self.stop_flag = threading.Event()
        self.result_data = {}
        self._lock = threading.Lock()

        # Import and setup skills directly
        from robot.skill.object_detection.object_detection import object_detection_skill
        from robot.skill.navigation.navigate_to import navigate_to_skill
        from robot.skill.navigation.return_home import return_home_skill
        from robot.skill.manipulation.pickup_object import pickup_object_skill
        from robot.skill.manipulation.put_down import put_down_skill
        from robot.skill.detection.detect import detect_skill
        from robot.skill.exploration.explore import explore_skill
        from robot.skill.exploration.plan_exploration_waypoints import (
            plan_exploration_waypoints_skill,
        )

        self.skill_function = {
            "object_detection": object_detection_skill,
            "navigate_to": navigate_to_skill,
            "return_home": return_home_skill,
            "pickup_object": pickup_object_skill,
            "put_down": put_down_skill,
            "detect": detect_skill,
            "explore": explore_skill,
            "plan_exploration_waypoints": plan_exploration_waypoints_skill,
        }

        logger.info(
            f"SkillManager initialized for robot: {robot_instance.namespace if robot_instance else 'Unknown'}"
        )

    def execute_skill(self, skill_name: str, skill_args: Dict[str, Any] = None) -> bool:
        """
        Execute a skill

        Args:
            skill_name: Name of the skill
            skill_args: Skill parameters dictionary

        Returns:
            bool: Whether execution started successfully
        """
        with self._lock:
            # Check if skill exists
            if skill_name not in self.skill_function:
                logger.error(
                    f"Skill '{skill_name}' not found. Available skills: {list(self.skill_function.keys())}"
                )
                return False

            # Check if another skill is already running
            if self.state == SkillState.RUNNING:
                logger.warning(
                    f"Cannot execute skill '{skill_name}': another skill '{self.current_skill_name}' is running"
                )
                return False

            # Prepare for execution
            self.current_skill_name = skill_name
            self.current_skill_args = skill_args
            self.state = SkillState.RUNNING
            self.stop_flag.clear()
            self.result_data = {}

            # Execute skill in new thread
            self.current_thread = threading.Thread(
                target=self._execute_skill_wrapper,
                name=f"Skill_{skill_name}",
                daemon=True,
            )
            self.current_thread.start()

            logger.info(
                f"Skill '{skill_name}' started with args: {self.current_skill_args}"
            )
            return True

    def _execute_skill_wrapper(self):
        """
        Skill execution wrapper, handles exceptions and state management
        """
        skill_name = self.current_skill_name
        skill_function = self.skill_function[skill_name]

        try:
            logger.debug(f"Executing skill '{skill_name}'...")

            # Execute skill function
            result = skill_function(**self.current_skill_args)

            # Check if stopped
            if self.stop_flag.is_set():
                logger.info(f"Skill '{skill_name}' was stopped")
                result = {
                    "success": False,
                    "message": "Skill execution was stopped",
                    "data": None,
                }

            # Validate result format
            if not isinstance(result, dict):
                logger.error(
                    f"Skill '{skill_name}' returned invalid result format: {type(result)}"
                )
                result = {
                    "success": False,
                    "message": "Invalid result format",
                    "data": result,
                }

            # Ensure required fields exist
            if "success" not in result:
                result["success"] = False
            if "message" not in result:
                result["message"] = "No message provided"
            if "data" not in result:
                result["data"] = None

            # Save result
            self.result_data = result

            # Set final state
            if result["success"]:
                logger.info(
                    f"Skill '{skill_name}' completed successfully: {result['message']}"
                )
                self.state = SkillState.IDLE
            else:
                logger.warning(f"Skill '{skill_name}' failed: {result['message']}")
                self.state = SkillState.FAILED
                # Automatically return to idle state after failure
                time.sleep(
                    0.1
                )  # Brief delay to allow external reading of failure state
                self.state = SkillState.IDLE

        except Exception as e:
            logger.error(f"Error executing skill '{skill_name}': {e}")
            self.result_data = {
                "success": False,
                "message": f"Skill execution failed: {str(e)}",
                "data": None,
            }
            self.state = SkillState.FAILED
            # Automatically return to idle state after failure
            time.sleep(0.1)
            self.state = SkillState.IDLE

        finally:
            # Clean up execution context
            with self._lock:
                if not self.stop_flag.is_set():  # Only clean up on normal completion
                    self.current_skill_name = ""
                    self.current_skill_args = {}
                self.current_thread = None
