# Standard library imports
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class CfgCamera:
    type: str = "camera"
    id: Optional[int] = None
    path_prim_relative: str = None
    path_prim_absolute: str = None
    # path_prim_joint_target_relative_to_robot: Optional[str] = None
    # path_prim_joint_target_absolute: Optional[str] = None

    use_existing_camera: bool = False
    name: str = "camera"
    #  Frequency of the sensor (i.e: how often is the data frame updated). Defaults to None.
    frequency: Optional[int] = None
    # dt of the sensor (i.e: period at which a the data frame updated). Defaults to None.
    dt: Optional[str] = None
    # resolution of the camera (width, height).
    resolution: Optional[Tuple[int, int]] = None
    # position in the world frame of the prim. shape is (3, )
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    # translation in the local frame of the prim (with respect to its parent prim). shape is (3, ).
    translation: Optional[Tuple[float, float, float]] = None
    # quaternion orientation in the world/ local frame of the prim (depends if translation or position is specified). quaternion is scalar-first (w, x, y, z). shape is (4, ).
    orientation: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    # Attach the bounding_box_2d_loose annotator to this camera
    enable_semantic_detection: bool = False
    # Longer Lens Lengths Narrower FOV, Shorter Lens Lengths Wider FOV. Unit: mm
    focal_length: int = 20
    # The distance at which perfect sharpness is achieved.
    focus_distance: Optional[float] = None
    # Controls Distance Blurring. Lower Numbers decrease focus range, larger; 0 turns off focusing.
    lens_aperture: float = 0.0
    # Emulates sensor/film width on a camera.
    horizontal_aperture: Optional[float] = None
    # Emulates sensor/film height on a camera.
    vertical_aperture: Optional[float] = None
