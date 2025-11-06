# =============================================================================
# Minimal Camera Module - Fisheye Camera Setup and Configuration
# =============================================================================
#
# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# Modified by: Subhransu Mishra
# Date: 2025-05-15
#
# This script is used to set up the fisheye camera properties with
# Kannala-Brandt distortion model for Isaac Sim simulations.
#
# =============================================================================

# Standard library imports
import math
from typing import Callable, List, Optional, Sequence, Tuple

# Third-party library imports
import numpy as np

# Local project imports
from physics_engine.isaacsim_utils import (
    get_prim_at_path,
    get_prim_type_name,
    is_prim_path_valid,
)
from physics_engine.pxr_utils import Sdf, Vt


def point_to_theta(camera_matrix, x, y):
    """This helper function returns the theta angle of the point."""
    ((fx, _, cx), (_, fy, cy), (_, _, _)) = camera_matrix
    pt_x, pt_y, pt_z = (x - cx) / fx, (y - cy) / fy, 1.0
    r2 = pt_x * pt_x + pt_y * pt_y
    theta = np.arctan2(np.sqrt(r2), 1.0)
    return theta


def distort_point_kannala_brandt(camera_matrix, distortion_model, x, y):
    """This helper function distorts point(s) using Kannala Brandt fisheye model.
    It should be equivalent to the following reference that uses OpenCV:

    def distort_point_kannala_brandt2(camera_matrix, distortion_model, x, y):
        import cv2
        ((fx,_,cx),(_,fy,cy),(_,_,_)) = camera_matrix
        pt_x, pt_y, pt_z  = (x-cx)/fx, (y-cy)/fy, np.full(x.shape, 1.0)
        points3d = np.stack((pt_x, pt_y, pt_z), axis = -1)
        rvecs, tvecs = np.array([0.0,0.0,0.0]), np.array([0.0,0.0,0.0])
        cameraMatrix, distCoeffs = np.array(camera_matrix), np.array(distortion_model)
        points, jac = cv2.fisheye.projectPoints(np.expand_dims(points3d, 1), rvecs, tvecs, cameraMatrix, distCoeffs)
        return np.array([points[:,0,0], points[:,0,1]])
    """
    ((fx, _, cx), (_, fy, cy), (_, _, _)) = camera_matrix
    pt_x, pt_y, pt_z = (x - cx) / fx, (y - cy) / fy, 1.0
    r2 = pt_x * pt_x + pt_y * pt_y
    r = np.sqrt(r2)
    theta = np.arctan2(r, 1.0)

    t3 = theta * theta * theta
    t5 = t3 * theta * theta
    t7 = t5 * theta * theta
    t9 = t7 * theta * theta
    k1, k2, k3, k4 = list(distortion_model[:4])
    theta_d = theta + k1 * t3 + k2 * t5 + k3 * t7 + k4 * t9

    inv_r = 1.0 / r  # if r > 1e-8 else 1.0
    cdist = theta_d * inv_r  # if r > 1e-8 else 1.0

    r_x, r_y = pt_x * cdist, pt_y * cdist
    return np.array([fx * r_x + cx, fy * r_y + cy])


class MinimalCamera:
    def __init__(self, prim_path: str, resolution) -> None:

        if is_prim_path_valid(prim_path):
            self.prim = get_prim_at_path(prim_path)
            if get_prim_type_name(prim_path) != "Camera":
                raise Exception("prim path does not correspond to a Camera prim.")
        else:
            raise Exception("prim path does not correspond to a Camera prim.")

        self._resolution = resolution

        if self.prim.GetAttribute("cameraProjectionType").Get() is None:
            attr = self.prim.CreateAttribute(
                "cameraProjectionType", Sdf.ValueTypeNames.Token
            )
            # The allowed tokens are not set in kit except with the first interaction with the dropdown menu
            # setting it here for now.
            if attr.GetMetadata("allowedTokens") is None:
                attr.SetMetadata(
                    "allowedTokens",
                    [
                        "pinhole",
                        "fisheyeOrthographic",
                        "fisheyeEquidistant",
                        "fisheyeEquisolid",
                        "fisheyePolynomial",
                        "fisheyeSpherical",
                        "fisheyeKannalaBrandtK3",
                        "fisheyeRadTanThinPrism",
                        "omniDirectionalStereo",
                    ],
                )
        properties = [
            "fthetaPolyA",
            "fthetaPolyB",
            "fthetaPolyC",
            "fthetaPolyD",
            "fthetaPolyE",
            "fthetaCx",
            "fthetaCy",
            "fthetaWidth",
            "fthetaHeight",
            "fthetaMaxFov",
        ]
        for property_name in properties:
            if self.prim.GetAttribute(property_name).Get() is None:
                self.prim.CreateAttribute(property_name, Sdf.ValueTypeNames.Float)
        return

    def get_resolution(self) -> Tuple[int, int]:
        return self._resolution

    def get_aspect_ratio(self) -> float:
        """
        Returns:
            float: ratio between width and height
        """
        width, height = self.get_resolution()
        return width / float(height)

    def get_focal_length(self) -> float:
        """
        Returns:
            float: Longer Lens Lengths Narrower FOV, Shorter Lens Lengths Wider FOV
        """
        return self.prim.GetAttribute("focalLength").Get() / 10.0

    def set_focal_length(self, value: float):
        """
        Args:
            value (float): Longer Lens Lengths Narrower FOV, Shorter Lens Lengths Wider FOV
        """
        self.prim.GetAttribute("focalLength").Set(value * 10.0)
        return

    def set_focus_distance(self, value: float):
        """The distance at which perfect sharpness is achieved.

        Args:
            value (float): Distance from the camera to the focus plane (in stage units).
        """
        self.prim.GetAttribute("focusDistance").Set(value)
        return

    def set_lens_aperture(self, value: float):
        """Controls Distance Blurring. Lower Numbers decrease focus range, larger
            numbers increase it.

        Args:
            value (float): controls lens aperture (i.e focusing). 0 turns off focusing.
        """
        self.prim.GetAttribute("fStop").Set(value)
        return

    def get_horizontal_aperture(self) -> float:
        """_
        Returns:
            float:  Emulates sensor/film width on a camera
        """
        aperture = self.prim.GetAttribute("horizontalAperture").Get() / 10.0
        return aperture

    def set_horizontal_aperture(self, value: float) -> None:
        """
        Args:
            value (Optional[float], optional): Emulates sensor/film width on a camera. Defaults to None.
        """
        self.prim.GetAttribute("horizontalAperture").Set(value * 10.0)
        (width, height) = self.get_resolution()
        self.prim.GetAttribute("verticalAperture").Set(
            (value * 10.0) * (float(height) / width)
        )
        return

    def get_vertical_aperture(self) -> float:
        """
        Returns:
            float: Emulates sensor/film height on a camera.
        """
        (width, height) = self.get_resolution()
        aperture = (self.prim.GetAttribute("horizontalAperture").Get() / 10.0) * (
            float(height) / width
        )
        return aperture

    def set_vertical_aperture(self, value: float) -> None:
        """
        Args:
            value (Optional[float], optional): Emulates sensor/film height on a camera. Defaults to None.
        """
        self.prim.GetAttribute("verticalAperture").Set(value * 10.0)
        (width, height) = self.get_resolution()
        self.prim.GetAttribute("horizontalAperture").Set(
            (value * 10.0) * (float(width) / height)
        )
        return

    def get_clipping_range(self) -> Tuple[float, float]:
        """
        Returns:
            Tuple[float, float]: near_distance and far_distance respectively.
        """
        near, far = self.prim.GetAttribute("clippingRange").Get()
        return near, far

    def set_clipping_range(
        self,
        near_distance: Optional[float] = None,
        far_distance: Optional[float] = None,
    ) -> None:
        """Clips the view outside of both near and far range values.

        Args:
            near_distance (Optional[float], optional): value to be used for near clipping. Defaults to None.
            far_distance (Optional[float], optional): value to be used for far clipping. Defaults to None.
        """
        near, far = self.prim.GetAttribute("clippingRange").Get()
        if near_distance:
            near = near_distance
        if far_distance:
            far = far_distance
        self.prim.GetAttribute("clippingRange").Set((near, far))
        return

    def get_projection_type(self) -> str:
        """
        Returns:
            str: pinhole, fisheyeOrthographic, fisheyeEquidistant, fisheyeEquisolid, fisheyePolynomial or fisheyeSpherical
        """
        projection_type = self.prim.GetAttribute("cameraProjectionType").Get()
        if projection_type is None:
            projection_type = "pinhole"
        return projection_type

    def set_projection_type(self, value: str) -> None:
        """
        Args:
            value (str): pinhole: Standard Camera Projection (Disable Fisheye)
                         fisheyeOrthographic: Full Frame using Orthographic Correction
                         fisheyeEquidistant: Full Frame using Equidistant Correction
                         fisheyeEquisolid: Full Frame using Equisolid Correction
                         fisheyePolynomial: 360 Degree Spherical Projection
                         fisheyeSpherical: 360 Degree Full Frame Projection
        """
        self.prim.GetAttribute("cameraProjectionType").Set(Vt.Token(value))
        return

    def get_projection_mode(self) -> str:
        """
        Returns:
            str: perspective or orthographic.
        """
        return self.prim.GetAttribute("projection").Get()

    def set_projection_mode(self, value: str) -> None:
        """Sets camera to perspective or orthographic mode.

        Args:
            value (str): perspective or orthographic.

        """
        self.prim.GetAttribute("projection").Set(value)
        return

    def set_matching_fisheye_polynomial_properties(
        self,
        nominal_width: float,
        nominal_height: float,
        optical_centre_x: float,
        optical_centre_y: float,
        max_fov: Optional[float],
        distortion_model: Sequence[float],
        distortion_fn: Callable,
    ) -> None:
        """Approximates given distortion with ftheta fisheye polynomial coefficients.
        Args:
            nominal_width (float): Rendered Width (pixels)
            nominal_height (float): Rendered Height (pixels)
            optical_centre_x (float): Horizontal Render Position (pixels)
            optical_centre_y (float): Vertical Render Position (pixels)
            max_fov (Optional[float]): maximum field of view (pixels)
            distortion_model (Sequence[float]): distortion model coefficients
            distortion_fn (Callable): distortion function that takes points and returns distorted points
        """
        if "fisheye" not in self.get_projection_type():
            raise Exception(
                "fisheye projection type is not set to allow use set_matching_fisheye_polynomial_properties method."
            )

        cx, cy = optical_centre_x, optical_centre_y
        fx = nominal_width * self.get_focal_length() / self.get_horizontal_aperture()
        fy = nominal_height * self.get_focal_length() / self.get_vertical_aperture()
        camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])

        # Fit the fTheta model for the points on the diagonals.
        X = np.concatenate(
            [
                np.linspace(0, nominal_width, nominal_width),
                np.linspace(0, nominal_width, nominal_width),
            ]
        )
        Y = np.concatenate(
            [
                np.linspace(0, nominal_height, nominal_width),
                np.linspace(nominal_height, 0, nominal_width),
            ]
        )
        theta = point_to_theta(camera_matrix, X, Y)
        r = np.linalg.norm(
            distortion_fn(camera_matrix, distortion_model, X, Y)
            - np.array([[cx], [cy]]),
            axis=0,
        )
        fthetaPoly = np.polyfit(r, theta, deg=4)

        for i, coefficient in enumerate(
            fthetaPoly[::-1]
        ):  # Reverse the order of the coefficients
            self.prim.GetAttribute("fthetaPoly" + (chr(ord("A") + i))).Set(
                float(coefficient)
            )

        self.prim.GetAttribute("fthetaWidth").Set(nominal_width)
        self.prim.GetAttribute("fthetaHeight").Set(nominal_height)
        self.prim.GetAttribute("fthetaCx").Set(optical_centre_x)
        self.prim.GetAttribute("fthetaCy").Set(optical_centre_y)

        if max_fov:
            self.prim.GetAttribute("fthetaMaxFov").Set(max_fov)
        return

    def set_kannala_brandt_properties(
        self,
        nominal_width: float,
        nominal_height: float,
        optical_centre_x: float,
        optical_centre_y: float,
        max_fov: Optional[float],
        distortion_model: Sequence[float],
    ) -> None:
        """Approximates kannala brandt distortion with ftheta fisheye polynomial coefficients.
        Args:
            nominal_width (float): Rendered Width (pixels)
            nominal_height (float): Rendered Height (pixels)
            optical_centre_x (float): Horizontal Render Position (pixels)
            optical_centre_y (float): Vertical Render Position (pixels)
            max_fov (Optional[float]): maximum field of view (pixels)
            distortion_model (Sequence[float]): kannala brandt generic distortion model coefficients (k1, k2, k3, k4)
        """

        self.set_matching_fisheye_polynomial_properties(
            nominal_width,
            nominal_height,
            optical_centre_x,
            optical_centre_y,
            max_fov,
            distortion_model,
            distort_point_kannala_brandt,
        )

        # Store the original distortion model parameters
        K, P = list(distortion_model[:2]) + list(distortion_model[4:]), list(
            distortion_model[2:4]
        )
        self.prim.CreateAttribute(
            "physicalDistortionModel", Sdf.ValueTypeNames.String
        ).Set("kannalaBrandt")
        self.prim.CreateAttribute(
            "physicalDistortionCoefficients", Sdf.ValueTypeNames.FloatArray, False
        ).Set(distortion_model)
        return

    def get_intrinsics_matrix(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: the intrinsics of the camera (used for calibration)
        """
        if "pinhole" not in self.get_projection_type():
            raise Exception(
                "pinhole projection type is not set to be able to use get_intrinsics_matrix method."
            )
        focal_length = self.get_focal_length()
        horizontal_aperture = self.get_horizontal_aperture()
        vertical_aperture = self.get_vertical_aperture()
        (width, height) = self.get_resolution()
        fx = width * focal_length / horizontal_aperture
        fy = height * focal_length / vertical_aperture
        cx = width * 0.5
        cy = height * 0.5
        return self._backend_utils.create_tensor_from_list(
            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
            dtype="float32",
            device=self._device,
        )

    def get_horizontal_fov(self) -> float:
        """
        Returns:
            float: horizontal field of view in pixels
        """
        return 2 * math.atan(
            self.get_horizontal_aperture() / (2 * self.get_focal_length())
        )

    def get_vertical_fov(self) -> float:
        """
        Returns:
            float: vertical field of view in pixels
        """
        width, height = self.get_resolution()
        return self.get_horizontal_fov() * (height / float(width))
