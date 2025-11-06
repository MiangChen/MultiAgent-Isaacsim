# =============================================================================
# PXR (Pixar USD) Utilities - Unified Import and Helper Functions
# =============================================================================
#
# This module provides a centralized import system for Pixar USD (Universal Scene Description)
# libraries and commonly used utility functions. It follows the official USD API structure
# and provides Python bindings for USD's C++ API.
#
# References:
# - USD C++ API Documentation: https://openusd.org/release/api/index.html
# - Python/C++ API Naming Notes: https://developer.nvidia.com/usd/apinotes
# - USD Python API follows similar patterns to C++ but with Pythonic conventions
#
# Key Differences between C++ and Python USD APIs:
# - C++ uses camelCase, Python uses PascalCase for class names
# - C++ pointers become Python objects with automatic memory management
# - C++ const references become Python immutable objects where applicable
# - Template specializations in C++ become specific Python classes
#
# Module Organization:
# 1. Core USD modules (Usd, UsdGeom, UsdShade)
# 2. Foundation modules (Sdf, Gf, Tf, Vt)
# 3. Physics extensions (UsdPhysics, PhysxSchema)
# 4. Optional specialized modules (UsdLux, UsdRender, etc.)
# 5. Commonly used class aliases for convenience
# 6. Utility functions for common USD operations
#
# =============================================================================

# Third-party library imports (Pixar USD)
# -----------------------------------------------------------------------------
# 1. Core USD - Universal Scene Description Core
# -----------------------------------------------------------------------------
# Usd: Core USD functionality for stages, prims, attributes, and relationships
# UsdGeom: Geometric primitives and transformations (meshes, cameras, lights, etc.)
# UsdShade: Shading and material networks (materials, shaders, textures)
from pxr import Usd, UsdGeom, UsdShade

# -----------------------------------------------------------------------------
# 2. Sdf - Scene Description Foundation
# -----------------------------------------------------------------------------
# Sdf: Low-level scene description API for layers, paths, and value types
# Provides the foundation for USD's composition and layering system
from pxr import Sdf

# -----------------------------------------------------------------------------
# 3. Gf - Graphics Foundation (Math/Geometry)
# -----------------------------------------------------------------------------
# Gf: Mathematical types and operations for 3D graphics
# Includes vectors, matrices, quaternions, bounding boxes, and geometric utilities
from pxr import Gf

# -----------------------------------------------------------------------------
# 4. Tf - Tools Foundation (Utilities)
# -----------------------------------------------------------------------------
# Tf: Core utilities including error handling, notifications, and type system
# Provides debugging, logging, and notification mechanisms for USD
from pxr import Tf

# -----------------------------------------------------------------------------
# 5. Vt - Value Types
# -----------------------------------------------------------------------------
# Vt: Value types for USD attributes including arrays and dictionaries
# Provides efficient storage and manipulation of attribute values
from pxr import Vt

# -----------------------------------------------------------------------------
# 6. Physics - USD Physics Schema
# -----------------------------------------------------------------------------
# UsdPhysics: Standard USD physics schema for rigid bodies, collisions, joints, and materials
# Provides cross-platform physics representation compatible with multiple physics engines
from pxr import UsdPhysics

# -----------------------------------------------------------------------------
# 7. PhysX - NVIDIA PhysX Schema Extensions
# -----------------------------------------------------------------------------
# PhysxSchema: NVIDIA PhysX-specific extensions to USD Physics
# Provides advanced PhysX features like articulations, vehicles, and GPU acceleration
from pxr import PhysxSchema

# -----------------------------------------------------------------------------
# 8. Additional Specialized Modules - Optional Imports
# -----------------------------------------------------------------------------
try:
    # UsdLux: Lighting schema (lights, light filters, dome lights)
    # UsdRender: Rendering schema (render settings, products, variables)
    # UsdSkel: Skeletal animation schema (skeletons, animations, skinning)
    # UsdUI: User interface schema (UI elements and layout)
    # UsdVol: Volumetric data schema (volume fields, OpenVDB integration)
    from pxr import UsdLux, UsdRender, UsdSkel, UsdUI, UsdVol
except ImportError:
    # These modules may not be available in all USD builds
    pass

# =============================================================================
# Commonly Used Classes and Functions - Direct Access Aliases
# =============================================================================
#
# These aliases provide direct access to frequently used USD classes without
# requiring the full module path. This improves code readability and follows
# the pattern established in Isaac Sim's isaacsim_utils.py.
#
# Usage Examples:
#   Instead of: Gf.Vec3d(1, 2, 3)
#   Use:        Vec3d(1, 2, 3)
#
#   Instead of: UsdGeom.Xformable(prim)
#   Use:        Xformable(prim)
#
# =============================================================================

# -----------------------------------------------------------------------------
# Gf (Graphics Foundation) - Mathematical Types
# -----------------------------------------------------------------------------
# Vector types for 3D and 4D coordinates
Vec3d = Gf.Vec3d  # Double precision 3D vector (x, y, z)
Vec3f = Gf.Vec3f  # Single precision 3D vector (x, y, z)
Vec4d = Gf.Vec4d  # Double precision 4D vector (x, y, z, w)
Vec4f = Gf.Vec4f  # Single precision 4D vector (x, y, z, w)

# Matrix types for transformations
Matrix4d = Gf.Matrix4d  # Double precision 4x4 transformation matrix
Matrix4f = Gf.Matrix4f  # Single precision 4x4 transformation matrix

# Quaternion types for rotations
Quatd = Gf.Quatd  # Double precision quaternion (w, x, y, z)
Quatf = Gf.Quatf  # Single precision quaternion (w, x, y, z)

# Range types for intervals and bounding ranges
Range1d = Gf.Range1d  # 1D range (min, max)
Range1f = Gf.Range1f  # 1D range (min, max) - float
Range2d = Gf.Range2d  # 2D range (min, max) for each axis
Range2f = Gf.Range2f  # 2D range (min, max) for each axis - float
Range3d = Gf.Range3d  # 3D range (min, max) for each axis
Range3f = Gf.Range3f  # 3D range (min, max) for each axis - float

# Bounding box and transformation types
BBox3d = Gf.BBox3d  # 3D axis-aligned bounding box
Transform = Gf.Transform  # Transformation with translation, rotation, scale

# -----------------------------------------------------------------------------
# UsdGeom - Geometric Primitives and Transformations
# -----------------------------------------------------------------------------
# Transformation and hierarchy
Xformable = UsdGeom.Xformable  # Base class for transformable objects
XformOp = UsdGeom.XformOp  # Individual transformation operation
Xform = UsdGeom.Xform  # Transform node (grouping/hierarchy)
Scope = UsdGeom.Scope  # Organizational grouping node

# Geometric primitives
Mesh = UsdGeom.Mesh  # Polygonal mesh geometry
Cube = UsdGeom.Cube  # Box/cube primitive
Sphere = UsdGeom.Sphere  # Sphere primitive
Cylinder = UsdGeom.Cylinder  # Cylinder primitive
Cone = UsdGeom.Cone  # Cone primitive
Plane = UsdGeom.Plane  # Plane primitive

# Camera and imaging
Camera = UsdGeom.Camera  # Camera for rendering and viewport

# -----------------------------------------------------------------------------
# Usd - Core USD Scene Graph
# -----------------------------------------------------------------------------
# Core USD scene graph components
Stage = Usd.Stage  # Root container for USD scene
Prim = Usd.Prim  # Scene graph node/object
Attribute = Usd.Attribute  # Property that holds values
Relationship = Usd.Relationship  # Connection between prims
TimeCode = Usd.TimeCode  # Time sampling for animation

# -----------------------------------------------------------------------------
# Sdf - Scene Description Foundation
# -----------------------------------------------------------------------------
# Low-level USD foundation types
Path = Sdf.Path  # USD path identifier (/World/Cube)
AssetPath = Sdf.AssetPath  # File path reference for assets
ChangeBlock = Sdf.ChangeBlock  # Batched change notifications

# -----------------------------------------------------------------------------
# UsdPhysics - Standard Physics Schema
# -----------------------------------------------------------------------------
# Physics body and collision APIs
RigidBodyAPI = UsdPhysics.RigidBodyAPI  # Rigid body dynamics
CollisionAPI = UsdPhysics.CollisionAPI  # Collision detection
MassAPI = UsdPhysics.MassAPI  # Mass properties
MaterialAPI = UsdPhysics.MaterialAPI  # Physics material properties

# Joint types for articulated bodies
Joint = UsdPhysics.Joint  # Base joint class
FixedJoint = UsdPhysics.FixedJoint  # Fixed/welded joint
RevoluteJoint = UsdPhysics.RevoluteJoint  # Rotational joint (hinge)
PrismaticJoint = UsdPhysics.PrismaticJoint  # Linear sliding joint
SphericalJoint = UsdPhysics.SphericalJoint  # Ball joint (3DOF rotation)
DistanceJoint = UsdPhysics.DistanceJoint  # Distance constraint

# Joint control and actuation
DriveAPI = UsdPhysics.DriveAPI  # Joint drives and motors

# -----------------------------------------------------------------------------
# PhysxSchema - NVIDIA PhysX Extensions
# -----------------------------------------------------------------------------
# PhysX-specific extensions and optimizations
PhysxRigidBodyAPI = PhysxSchema.PhysxRigidBodyAPI  # PhysX rigid body extensions
PhysxCollisionAPI = PhysxSchema.PhysxCollisionAPI  # PhysX collision extensions
PhysxMaterialAPI = PhysxSchema.PhysxMaterialAPI  # PhysX material extensions
PhysxJointAPI = PhysxSchema.PhysxJointAPI  # PhysX joint extensions
PhysxSceneAPI = PhysxSchema.PhysxSceneAPI  # PhysX scene settings

# =============================================================================
# Utility Functions - Common USD Operations
# =============================================================================
#
# These utility functions provide high-level operations for common USD tasks
# such as transformations, coordinate conversions, and prim manipulation.
# They abstract away low-level USD API details and provide a more convenient
# interface for typical robotics and simulation use cases.
#
# =============================================================================


def create_transform_matrix(
    translation=(0, 0, 0), rotation=(0, 0, 0, 1), scale=(1, 1, 1)
):
    """
    Create a 4x4 transformation matrix from translation, rotation (quaternion), and scale.

    This function combines translation, rotation, and scale into a single transformation
    matrix using the standard TRS (Translation * Rotation * Scale) order. The resulting
    matrix can be used with USD's Xformable API or for manual transformation calculations.

    Args:
        translation (tuple): (x, y, z) translation vector in world units
        rotation (tuple): (w, x, y, z) quaternion rotation (w=real, xyz=imaginary)
        scale (tuple): (x, y, z) scale factors (1.0 = no scaling)

    Returns:
        Gf.Matrix4d: 4x4 transformation matrix in column-major order

    Example:
        >>> # Create transform for object at (1,2,3) with 90° Y rotation
        >>> import math
        >>> quat = euler_to_quaternion(0, 90, 0)  # 90° around Y axis
        >>> matrix = create_transform_matrix((1, 2, 3), quat, (1, 1, 1))
        >>> print(matrix)
    """
    # Create translation matrix
    trans_matrix = Matrix4d(1.0)
    trans_matrix.SetTranslateOnly(Vec3d(*translation))

    # Create rotation matrix
    quat = Quatd(rotation[0], Vec3d(rotation[1], rotation[2], rotation[3]))
    rot_matrix = Matrix4d(quat.GetNormalized())

    # Create scale matrix
    scale_matrix = Matrix4d(1.0)
    scale_matrix.SetScale(Vec3d(*scale))

    # Combine: T * R * S
    return trans_matrix * rot_matrix * scale_matrix


def euler_to_quaternion(roll=0.0, pitch=0.0, yaw=0.0, degrees=True):
    """
    Convert Euler angles to quaternion using ZYX rotation order (yaw-pitch-roll).

    This function converts Euler angles to a quaternion representation using the
    aerospace/robotics convention: Z(yaw) * Y(pitch) * X(roll). This is the most
    common convention in robotics and matches Isaac Sim's coordinate system.

    Args:
        roll (float): Rotation around X-axis (bank angle)
        pitch (float): Rotation around Y-axis (elevation angle)
        yaw (float): Rotation around Z-axis (azimuth angle)
        degrees (bool): If True, input angles are in degrees; if False, in radians

    Returns:
        Gf.Quatd: Normalized quaternion (w, x, y, z) where w is the real part

    Example:
        >>> # 90 degree rotation around Z axis (yaw)
        >>> quat = euler_to_quaternion(0, 0, 90, degrees=True)
        >>> print(f"Quaternion: w={quat.GetReal()}, xyz={quat.GetImaginary()}")

    Note:
        - Euler angles suffer from gimbal lock at pitch = ±90°
        - For continuous rotations, consider using quaternions directly
        - USD uses right-handed coordinate system with Y-up convention
    """
    import math

    if degrees:
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)

    # Convert to quaternion
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return Quatd(w, Vec3d(x, y, z))


def quaternion_to_euler(quat, degrees=True):
    """
    Convert quaternion to Euler angles using ZYX rotation order (yaw-pitch-roll).

    This function extracts Euler angles from a quaternion using the aerospace/robotics
    convention. The conversion assumes the quaternion represents a rotation in the
    order Z(yaw) * Y(pitch) * X(roll).

    Args:
        quat (Gf.Quatd): Input quaternion (should be normalized)
        degrees (bool): If True, return angles in degrees; if False, in radians

    Returns:
        tuple: (roll, pitch, yaw) angles in the specified units
            - roll: Rotation around X-axis [-180°, 180°] or [-π, π]
            - pitch: Rotation around Y-axis [-90°, 90°] or [-π/2, π/2]
            - yaw: Rotation around Z-axis [-180°, 180°] or [-π, π]

    Example:
        >>> quat = Quatd(0.707, Vec3d(0, 0, 0.707))  # 90° Z rotation
        >>> roll, pitch, yaw = quaternion_to_euler(quat, degrees=True)
        >>> print(f"Euler: roll={roll}, pitch={pitch}, yaw={yaw}")

    Note:
        - Gimbal lock occurs when pitch ≈ ±90°, causing roll and yaw to be coupled
        - For continuous angle tracking, consider using quaternions directly
        - Input quaternion should be normalized for accurate results
    """
    import math

    # Extract quaternion components
    w = quat.GetReal()
    x, y, z = quat.GetImaginary()

    # Convert to Euler angles
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    if degrees:
        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)

    return roll, pitch, yaw


def get_prim_world_transform(prim):
    """
    Get the world (global) transformation matrix of a USD prim.

    This function computes the accumulated transformation from the prim's local
    transform and all its parent transforms up to the root. It handles the full
    USD transformation stack including translate, rotate, and scale operations.

    Args:
        prim (Usd.Prim): USD prim to get world transform for

    Returns:
        Gf.Matrix4d: 4x4 world transformation matrix, or identity matrix if invalid

    Example:
        >>> stage = Usd.Stage.CreateInMemory()
        >>> cube_prim = UsdGeom.Cube.Define(stage, "/World/Cube").GetPrim()
        >>> world_matrix = get_prim_world_transform(cube_prim)
        >>> translation = world_matrix.ExtractTranslation()
        >>> print(f"World position: {translation}")

    Note:
        - Returns identity matrix for invalid prims
        - Recursively accumulates parent transformations
        - Handles complex transformation stacks with multiple XformOps
        - Result is in USD's right-handed Y-up coordinate system
    """
    if not prim.IsValid():
        return Matrix4d(1.0)

    xformable = Xformable(prim)
    if not xformable:
        return Matrix4d(1.0)

    # Get local transformation
    local_transform = xformable.GetLocalTransformation()

    # Get parent transformation recursively
    parent_prim = prim.GetParent()
    if parent_prim and parent_prim.GetPath() != Sdf.Path.absoluteRootPath:
        parent_transform = get_prim_world_transform(parent_prim)
        return parent_transform * local_transform

    return local_transform


def set_prim_transform(prim, translation=None, rotation=None, scale=None):
    """
    Set the local transformation of a USD prim using standard TRS order.

    This function sets up the transformation stack for a prim using the standard
    Transform-Rotate-Scale order. It clears any existing transformation operations
    and creates new ones in the correct order for predictable behavior.

    Args:
        prim (Usd.Prim): USD prim to transform
        translation (tuple, optional): (x, y, z) translation in world units
        rotation (tuple, optional): Either:
            - (w, x, y, z) quaternion (w=real part)
            - (roll, pitch, yaw) Euler angles in degrees
        scale (tuple, optional): (x, y, z) scale factors (1.0 = no scaling)

    Returns:
        bool: True if transformation was applied successfully, False otherwise

    Example:
        >>> # Position cube at (1,2,3) with 45° Y rotation and 2x scale
        >>> stage = Usd.Stage.CreateInMemory()
        >>> cube = UsdGeom.Cube.Define(stage, "/World/Cube")
        >>> success = set_prim_transform(
        ...     cube.GetPrim(),
        ...     translation=(1, 2, 3),
        ...     rotation=(0, 45, 0),  # Euler angles
        ...     scale=(2, 2, 2)
        ... )

    Note:
        - Clears existing transform operations for clean state
        - Uses standard TRS order: translate, then rotate, then scale
        - Rotation can be specified as quaternion or Euler angles
        - Returns False for invalid prims or non-transformable objects
        - All parameters are optional - only specified transforms are applied
    """
    if not prim.IsValid():
        return False

    xformable = Xformable(prim)
    if not xformable:
        return False

    # Clear existing transform ops
    xformable.ClearXformOpOrder()

    # Add transform operations in order: translate, rotate, scale
    if translation is not None:
        translate_op = xformable.AddTranslateOp()
        translate_op.Set(Vec3d(*translation))

    if rotation is not None:
        if len(rotation) == 3:  # Euler angles
            rotation = euler_to_quaternion(*rotation)
        elif len(rotation) == 4:  # Quaternion
            rotation = Quatd(rotation[0], Vec3d(rotation[1], rotation[2], rotation[3]))

        orient_op = xformable.AddOrientOp()
        orient_op.Set(rotation)

    if scale is not None:
        scale_op = xformable.AddScaleOp()
        scale_op.Set(Vec3d(*scale))

    return True


# =============================================================================
# Additional Utility Functions
# =============================================================================


def get_prim_bounding_box(prim, time_code=None):
    """
    Get the world-space bounding box of a prim and its children.

    Args:
        prim (Usd.Prim): USD prim to compute bounding box for
        time_code (Usd.TimeCode, optional): Time to evaluate at (default: current)

    Returns:
        Gf.BBox3d: World-space bounding box, or empty box if invalid
    """
    if not prim.IsValid():
        return BBox3d()

    if time_code is None:
        time_code = TimeCode.Default()

    # Get bounding box using UsdGeom
    bbox_cache = UsdGeom.BBoxCache(
        time_code, includedPurposes=[UsdGeom.Tokens.default_]
    )
    return bbox_cache.ComputeWorldBound(prim)


def create_physics_material(
    stage, path, static_friction=0.5, dynamic_friction=0.5, restitution=0.0
):
    """
    Create a physics material with specified properties.

    Args:
        stage (Usd.Stage): USD stage to create material in
        path (str): USD path for the material (e.g., "/World/Materials/RubberMat")
        static_friction (float): Static friction coefficient [0.0, inf)
        dynamic_friction (float): Dynamic friction coefficient [0.0, inf)
        restitution (float): Restitution/bounciness [0.0, 1.0]

    Returns:
        UsdPhysics.MaterialAPI: Created physics material API
    """
    # Create material prim
    material_prim = stage.DefinePrim(path, "Material")

    # Apply physics material API
    physics_material = MaterialAPI.Apply(material_prim)

    # Set material properties
    physics_material.CreateStaticFrictionAttr().Set(static_friction)
    physics_material.CreateDynamicFrictionAttr().Set(dynamic_friction)
    physics_material.CreateRestitutionAttr().Set(restitution)

    return physics_material


def apply_rigid_body_physics(prim, mass=1.0, enable_gravity=True):
    """
    Apply rigid body physics to a prim with collision detection.

    Args:
        prim (Usd.Prim): USD prim to make into a rigid body
        mass (float): Mass in kg (default: 1.0)
        enable_gravity (bool): Whether gravity affects this body

    Returns:
        tuple: (RigidBodyAPI, CollisionAPI) if successful, (None, None) if failed
    """
    if not prim.IsValid():
        return None, None

    # Apply rigid body API
    rigid_body = RigidBodyAPI.Apply(prim)
    if not rigid_body:
        return None, None

    # Apply collision API
    collision = CollisionAPI.Apply(prim)
    if not collision:
        return None, None

    # Set mass properties
    mass_api = MassAPI.Apply(prim)
    if mass_api:
        mass_api.CreateMassAttr().Set(mass)

    # Set gravity enable
    rigid_body.CreateRigidBodyEnabledAttr().Set(True)
    if hasattr(rigid_body, "CreateGravityEnabledAttr"):
        rigid_body.CreateGravityEnabledAttr().Set(enable_gravity)

    return rigid_body, collision


# =============================================================================
# Usage Examples and Best Practices
# =============================================================================

"""
Common Usage Patterns:

1. Creating and Transforming Objects:
    ```python
    from physics_engine.pxr_utils import *
    
    # Create stage and cube
    stage = Stage.CreateInMemory()
    cube = UsdGeom.Cube.Define(stage, "/World/Cube")
    
    # Set transform using utility function
    set_prim_transform(
        cube.GetPrim(),
        translation=(1, 2, 3),
        rotation=(0, 45, 0),  # 45° Y rotation
        scale=(2, 1, 1)       # 2x scale on X axis
    )
    ```

2. Physics Setup:
    ```python
    # Make cube a rigid body
    cube_prim = cube.GetPrim()
    rigid_body, collision = apply_rigid_body_physics(cube_prim, mass=5.0)
    
    # Create physics material
    material = create_physics_material(
        stage, "/World/Materials/Metal",
        static_friction=0.8,
        dynamic_friction=0.6,
        restitution=0.1
    )
    ```

3. Coordinate System Conversions:
    ```python
    # Convert between Euler and quaternion
    euler_angles = (30, 45, 60)  # degrees
    quat = euler_to_quaternion(*euler_angles, degrees=True)
    back_to_euler = quaternion_to_euler(quat, degrees=True)
    
    # Create transformation matrix
    transform_matrix = create_transform_matrix(
        translation=(0, 1, 0),
        rotation=quat,
        scale=(1, 1, 1)
    )
    ```

4. Querying Scene Information:
    ```python
    # Get world transform and bounding box
    world_transform = get_prim_world_transform(cube_prim)
    world_position = world_transform.ExtractTranslation()
    bounding_box = get_prim_bounding_box(cube_prim)
    
    print(f"Position: {world_position}")
    print(f"Bounding box: {bounding_box.GetRange()}")
    ```

Best Practices:
- Always check prim.IsValid() before operations
- Use TimeCode.Default() for static scenes, specific time codes for animation
- Normalize quaternions before use in transformations
- Use double precision (Vec3d, Matrix4d) for accuracy in large scenes
- Apply physics APIs in order: RigidBody -> Collision -> Mass -> Material
- Use Sdf.Path for path operations instead of string manipulation
- Cache expensive operations like bounding box calculations
- Use ChangeBlock for batched modifications to improve performance
"""
