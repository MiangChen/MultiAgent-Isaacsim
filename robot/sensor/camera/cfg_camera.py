from typing import Optional, Tuple


from config.cfg_base import CfgBase


class CfgCamera(CfgBase):

    type: str = "camera"
    id: int = None
    path_prim_relative_to_robot: str = "/sensor/camera"
    path_prim_absolute: str = None
    path_prim_joint_target_relative_to_robot: str = None
    path_prim_joint_target_absolute: str = None

    use_existing_camera: bool = False

    # shortname to be used as a key by Scene class. Note: needs to be unique if the object is added to the Scene. Defaults to “camera”.
    name: str = "camera"
    #  Frequency of the sensor (i.e: how often is the data frame updated). Defaults to None.
    frequency: int = None
    # dt of the sensor (i.e: period at which a the data frame updated). Defaults to None.
    dt: str = None
    # resolution of the camera (width, height).
    resolution: Tuple[int, int] = None  # 相机分辨率
    # position in the world frame of the prim. shape is (3, )
    position: Tuple[float, float, float] = (
        0.0,
        0.0,
        0.0,
    )
    # translation in the local frame of the prim (with respect to its parent prim). shape is (3, ).
    translation: Tuple[float, float, float] = None
    # quaternion orientation in the world/ local frame of the prim (depends if translation or position is specified). quaternion is scalar-first (w, x, y, z). shape is (4, ).
    orientation: Tuple[float, float, float, float] = (
        0.0,
        0.0,
        0.0,
        1.0,
    )
    # Attach the bounding_box_2d_loose annotator to this camera
    enable_semantic_detection: bool = False
    # Longer Lens Lengths Narrower FOV, Shorter Lens Lengths Wider FOV. Unit: mm
    focal_length: int = 30
    # The distance at which perfect sharpness is achieved.
    focus_distance: float = None
    # Controls Distance Blurring. Lower Numbers decrease focus range, larger; 0 turns off focusing.
    lens_aperture: float = 0
    # Emulates sensor/film width on a camera.
    horizontal_aperture: float = None
    # Emulates sensor/film height on a camera.
    vertical_aperture: float = None
