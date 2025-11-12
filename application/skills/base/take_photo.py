"""Take Photo Skill - CARLA Style

Captures a photo from a robot's camera sensor and saves it to disk.

Flow: Check camera -> Capture image -> Save -> Complete
"""

from application import SkillManager
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


@SkillManager.register()
def take_photo(**kwargs):
    """
    Take a photo from robot's camera (CARLA style)
    
    Parameters:
        camera_type: Camera sensor type (default: 'sensor.camera.rgb')
        save_path: Path to save the photo (required)
        
    Example ROS call:
        ros2 action send_goal /h1_0/skill_execution plan_msgs/action/SkillExecution \
          '{skill_request: {skill_list: [{
            skill: "take_photo",
            params: [
              {key: "camera_type", value: "sensor.camera.rgb"},
              {key: "save_path", value: "/home/user/photo.jpg"}
            ]
          }]}}' --feedback
    """
    robot = kwargs.get("robot")
    skill_manager = kwargs.get("skill_manager")
    skill_name = "take_photo"

    current_state = skill_manager.get_skill_state(skill_name)

    if current_state in [None, "INITIALIZING"]:
        return _init_take_photo(robot, skill_manager, skill_name, kwargs)
    elif current_state == "EXECUTING":
        return _handle_executing(robot, skill_manager, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(robot, skill_manager, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(robot, skill_manager, skill_name)


def _init_take_photo(robot, skill_manager, skill_name, kwargs):
    """Initialize take photo skill"""
    
    # Get parameters (CARLA style)
    camera_type = kwargs.get("camera_type", "sensor.camera.rgb")
    save_path = kwargs.get("save_path", kwargs.get("save_to_file", ""))  # Backward compatibility
    
    # Validate save_path
    if not save_path:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = "save_path parameter is required"
        return skill_manager.form_feedback("failed", "save_path is required")
    
    try:
        # Find camera sensor (CARLA style)
        camera = robot.get_sensor_by_type(camera_type)
        
        if camera is None:
            # List available sensors for debugging
            sensors = robot.get_sensors()
            sensor_types = [s.get_type_id() for s in sensors]
            
            error_msg = f"Camera '{camera_type}' not found. Available: {sensor_types}"
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = error_msg
            return skill_manager.form_feedback("failed", error_msg)
        
        # Store data
        skill_manager.set_skill_data(skill_name, "camera_type", camera_type)
        skill_manager.set_skill_data(skill_name, "save_path", save_path)
        skill_manager.set_skill_data(skill_name, "camera", camera)
        
        skill_manager.set_skill_state(skill_name, "EXECUTING")
        return skill_manager.form_feedback("processing", "Preparing camera", 20)
        
    except Exception as e:
        logger.error(f"Init take_photo failed: {e}")
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = f"Init failed: {str(e)}"
        return skill_manager.form_feedback("failed", f"Init failed: {str(e)}")


def _handle_executing(robot, skill_manager, skill_name):
    """Execute photo capture"""
    try:
        camera = skill_manager.get_skill_data(skill_name, "camera")
        save_path = skill_manager.get_skill_data(skill_name, "save_path")
        camera_type = skill_manager.get_skill_data(skill_name, "camera_type")
        
        # Capture image from sensor
        rgb = camera.sensor.get_rgb()
        
        if rgb is None:
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = "Failed to capture image"
            return skill_manager.form_feedback("failed", "Capture failed")
        
        # Save to file
        try:
            result = camera.sensor.save_rgb_to_file(rgb, save_path)
            logger.info(f"Photo saved to {save_path}")
            
            # Store result
            skill_manager.set_skill_data(skill_name, "image_shape", rgb.shape)
            skill_manager.set_skill_data(skill_name, "saved_path", save_path)
            
            skill_manager.set_skill_state(skill_name, "COMPLETED")
            return skill_manager.form_feedback(
                "completed", 
                f"{result}",
                100
            )
            
        except Exception as e:
            logger.error(f"Failed to save photo: {e}")
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = f"Save failed: {str(e)}"
            return skill_manager.form_feedback("failed", f"Save failed: {str(e)}")
        
    except Exception as e:
        logger.error(f"Execute take_photo failed: {e}")
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = f"Execution failed: {str(e)}"
        return skill_manager.form_feedback("failed", f"Execution failed: {str(e)}")


def _handle_completed(robot, skill_manager, skill_name):
    """Handle completed state"""
    saved_path = skill_manager.get_skill_data(skill_name, "saved_path")
    return skill_manager.form_feedback("completed", f"Photo saved to {saved_path}", 100)


def _handle_failed(robot, skill_manager, skill_name):
    """Handle failed state"""
    error_msg = skill_manager.skill_errors.get(skill_name, "Unknown error")
    return skill_manager.form_feedback("failed", error_msg)
