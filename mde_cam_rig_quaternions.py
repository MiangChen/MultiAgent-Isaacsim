"""
Author: Subhransu Mishra

This script generates the camera rig quaternions for the MDE camera rig.

"""

import matplotlib

matplotlib.use("TkAgg")
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

# Rotation matrices lambda functions
Rx = lambda theta: Rotation.from_euler("x", theta, degrees=True)
Ry = lambda theta: Rotation.from_euler("y", theta, degrees=True)
Rz = lambda theta: Rotation.from_euler("z", theta, degrees=True)

# --- FOV Parameters for Frustum Visualization ---
fov_h_deg = 100.0  # Horizontal Field of View (degrees)
fov_v_deg = 100.0  # Vertical Field of View (degrees)
frustum_plot_depth = 0.05  # Depth of the frustum for plotting (meters)

# Define a list of colors for the cameras
camera_colors = ["cyan", "magenta", "yellow", "lime", "orange", "purple", "pink"]

# --- Camera Definitions ---
# Drone coordinate system: X=forward, Y=left, Z=up
# pos and orientations set by looking at the cad drawings
cameras = {}
cameras["HD_LT"] = {
    "pos": np.array([0.09283, 0.03483, 0.00139]),
    "rot": Rz(45) * Ry(-20),
}
cameras["HD_RT"] = {
    "pos": np.array([0.09283, -0.03483, 0.00139]),
    "rot": Rz(-45) * Ry(-20),
}
cameras["HD_UP"] = {"pos": np.array([0.05585, 0.0, 0.01905]), "rot": Ry(-90 + 12)}
cameras["RR_LT"] = {
    "pos": np.array([-0.12283, -0.03083, 0.00739]),
    "rot": Rz(-135) * Ry(-20),
}
cameras["RR_RT"] = {
    "pos": np.array([-0.12283, 0.03083, 0.00739]),
    "rot": Rz(135) * Ry(-20),
}
cameras["DN_LT"] = {"pos": np.array([-0.111, 0.0245, -0.08284]), "rot": Ry(90)}
cameras["DN_RT"] = {"pos": np.array([-0.111, -0.0245, -0.08284]), "rot": Ry(90)}

# --- Output Quaternions ---
print("Camera Definitions (Position [x,y,z] and Quaternion [w,x,y,z]):")
R_FLU_RUB = Rx(90) * Ry(-90)  # to isaac sim camera frame (Right Up Back)
print("RUB camera in FLU body frame")
for name, cam_data in cameras.items():
    quat = (cam_data["rot"] * R_FLU_RUB).as_quat()  # [x, y, z, w]
    pos_str = "[" + ", ".join(f"{x:.5g}" for x in cam_data["pos"]) + "]"
    wxyz = np.array([quat[3], quat[0], quat[1], quat[2]])
    quat_str = "[" + ", ".join(f"{x:.9g}" for x in wxyz) + "]"
    print(f'rig_comps["{name}"] = CamRigComponent(')
    print(f"    pos={pos_str},")
    print(f"    wxyz={quat_str},")
    print(f'    cam_type="fe_mde",')
    print(f'    render_types=["rgb"],')
    print(f")")

R_FRD_RDF = Rz(-90) * Rx(-90)  # to other camera frame (Front Right Down)
R_FRD_FLU = Rx(180)
print("RDF camera in FRD body frame")
for name, cam_data in cameras.items():
    pos_RDF = cam_data["pos"]
    pos_FRD = np.array([pos_RDF[0], -pos_RDF[1], -pos_RDF[2]])
    quat = (R_FRD_FLU * cam_data["rot"] * R_FRD_RDF).as_quat()  # [x, y, z, w]
    pos_str = "[" + ", ".join(f"{x:.5g}" for x in pos_FRD) + "]"
    wxyz = np.array([quat[3], quat[0], quat[1], quat[2]])
    quat_str = "[" + ", ".join(f"{x:.9g}" for x in wxyz) + "]"
    print(f'rig_comps["{name}"] = CamRigComponent(')
    print(f"    pos={pos_str},")
    print(f"    wxyz={quat_str},")
    print(f'    cam_type="fe_mde",')
    print(f'    render_types=["rgb"],')
    print(f")")

# --- 3D Plot Visualization ---
fig = plt.figure(figsize=(12, 12))
ax = fig.add_subplot(111, projection="3d")

# Plot drone body coordinate system at origin
drone_axis_length = 0.1
ax.quiver(
    0, 0, 0, 1, 0, 0, length=drone_axis_length, color="red", label="Drone X (forward)"
)
ax.quiver(
    0, 0, 0, 0, 1, 0, length=drone_axis_length, color="green", label="Drone Y (left)"
)
ax.quiver(
    0, 0, 0, 0, 0, 1, length=drone_axis_length, color="blue", label="Drone Z (up)"
)
ax.text(drone_axis_length, 0, 0, " X_D")
ax.text(0, drone_axis_length, 0, " Y_D")
ax.text(0, 0, drone_axis_length, " Z_D")


# Basis vectors for camera axes (X=Red, Y=Green, Z=Blue)
axis_plot_length = 0.04
cam_x_axis_local = np.array([1, 0, 0]) * axis_plot_length
cam_y_axis_local = np.array([0, 1, 0]) * axis_plot_length
cam_z_axis_local = np.array([0, 0, 1]) * axis_plot_length

for idx, (name, cam_data) in enumerate(cameras.items()):
    pos = cam_data["pos"]
    rot_matrix = cam_data[
        "rot"
    ].as_matrix()  # Transforms from camera frame to drone frame
    color = camera_colors[idx]

    # Plot camera position
    ax.scatter(
        pos[0], pos[1], pos[2], marker="o", s=60, color=color
    )  # Use camera-specific color
    ax.text(pos[0] + 0.01, pos[1] + 0.01, pos[2] + 0.01, name, fontsize=9)

    # Transform camera axes to drone frame and plot
    # X-axis (Red) - Optical axis in local camera frame
    x_axis_drone = rot_matrix @ cam_x_axis_local
    ax.quiver(
        pos[0],
        pos[1],
        pos[2],
        x_axis_drone[0],
        x_axis_drone[1],
        x_axis_drone[2],
        color="r",
        linestyle="-",
    )
    # Y-axis (Green)
    y_axis_drone = rot_matrix @ cam_y_axis_local
    ax.quiver(
        pos[0],
        pos[1],
        pos[2],
        y_axis_drone[0],
        y_axis_drone[1],
        y_axis_drone[2],
        color="g",
        linestyle="-",
    )
    # Z-axis (Blue)
    z_axis_drone = rot_matrix @ cam_z_axis_local
    ax.quiver(
        pos[0],
        pos[1],
        pos[2],
        z_axis_drone[0],
        z_axis_drone[1],
        z_axis_drone[2],
        color="b",
        linestyle="-",
    )

    # --- Frustum plotting ---
    # Define frustum corners in local camera frame (X-optical, Y-left, Z-up)
    half_fov_h_rad = np.deg2rad(fov_h_deg / 2.0)
    half_fov_v_rad = np.deg2rad(fov_v_deg / 2.0)

    # Extents on the far plane
    y_far_half = frustum_plot_depth * np.tan(half_fov_h_rad)
    z_far_half = frustum_plot_depth * np.tan(half_fov_v_rad)

    # Corners of the far plane in local camera coordinates
    # (X=frustum_plot_depth, Y, Z)
    corners_local = [
        np.array(
            [frustum_plot_depth, y_far_half, z_far_half]
        ),  # Top-left on far plane (camera's Y-left, Z-up)
        np.array([frustum_plot_depth, -y_far_half, z_far_half]),  # Top-right
        np.array([frustum_plot_depth, -y_far_half, -z_far_half]),  # Bottom-right
        np.array([frustum_plot_depth, y_far_half, -z_far_half]),  # Bottom-left
    ]

    # Transform corners to drone frame
    corners_drone = [rot_matrix @ c_loc + pos for c_loc in corners_local]

    # Plot lines from camera position (pos) to far plane corners
    for c_drone in corners_drone:
        ax.plot(
            [pos[0], c_drone[0]],
            [pos[1], c_drone[1]],
            [pos[2], c_drone[2]],
            color=color,
            linestyle=":",
            linewidth=0.8,
        )  # Use camera-specific color

    # Plot far plane rectangle (connecting corners_drone in order)
    ax.plot(
        [corners_drone[0][0], corners_drone[1][0]],
        [corners_drone[0][1], corners_drone[1][1]],
        [corners_drone[0][2], corners_drone[1][2]],
        color=color,
        linestyle="-",
        linewidth=0.8,
    )  # Use camera-specific color
    ax.plot(
        [corners_drone[1][0], corners_drone[2][0]],
        [corners_drone[1][1], corners_drone[2][1]],
        [corners_drone[1][2], corners_drone[2][2]],
        color=color,
        linestyle="-",
        linewidth=0.8,
    )  # Use camera-specific color
    ax.plot(
        [corners_drone[2][0], corners_drone[3][0]],
        [corners_drone[2][1], corners_drone[3][1]],
        [corners_drone[2][2], corners_drone[3][2]],
        color=color,
        linestyle="-",
        linewidth=0.8,
    )  # Use camera-specific color
    ax.plot(
        [corners_drone[3][0], corners_drone[0][0]],
        [corners_drone[3][1], corners_drone[0][1]],
        [corners_drone[3][2], corners_drone[0][2]],
        color=color,
        linestyle="-",
        linewidth=0.8,
    )  # Use camera-specific color

# Set plot limits and labels
all_positions = np.array([cam["pos"] for cam in cameras.values()])
all_points_for_limits = np.vstack(
    [np.zeros(3), all_positions, all_positions + drone_axis_length]
)  # Include origin and axis tips for robust limits
ax_min = all_points_for_limits.min(axis=0) - axis_plot_length * 2
ax_max = all_points_for_limits.max(axis=0) + axis_plot_length * 2

ax.set_xlim([min(ax_min[0], -0.2), max(ax_max[0], 0.2)])
ax.set_ylim([min(ax_min[1], -0.2), max(ax_max[1], 0.2)])
ax.set_zlim([min(ax_min[2], -0.15), max(ax_max[2], 0.15)])

ax.set_xlabel("Drone X (m)")
ax.set_ylabel("Drone Y (m)")
ax.set_zlabel("Drone Z (m)")
ax.set_title("Drone and Camera Configuration")

# Create a custom legend for camera axes (since they are plotted per camera)
from matplotlib.lines import Line2D

legend_elements = [
    Line2D([0], [0], color="red", lw=2, label="Drone X / Cam X"),
    Line2D([0], [0], color="green", lw=2, label="Drone Y / Cam Y"),
    Line2D([0], [0], color="blue", lw=2, label="Drone Z / Cam Z"),
    # Scatter for camera position
    plt.scatter([], [], s=60, marker="o", color="black", label="Camera Position"),
]
ax.legend(handles=legend_elements, loc="upper right", fontsize="small")
ax.view_init(elev=25.0, azim=-60)  # Adjust viewing angle
plt.tight_layout()
plt.show()
