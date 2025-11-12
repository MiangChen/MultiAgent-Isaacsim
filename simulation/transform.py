def _to_python_scalar(value):
    """
    将 torch tensor, numpy array 或其他类型转换为 Python 标量
    """
    if hasattr(value, 'item'):
        # torch tensor 或 numpy scalar
        return value.item()
    return float(value)


def _to_python_list(array):
    """
    将 torch tensor, numpy array 或其他类型转换为 Python list
    """
    if hasattr(array, 'tolist'):
        # torch tensor 或 numpy array
        return array.tolist()
    elif hasattr(array, '__iter__'):
        # 可迭代对象，逐个转换
        return [_to_python_scalar(x) for x in array]
    return list(array)


class Location:

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = _to_python_scalar(x)
        self.y = _to_python_scalar(y)
        self.z = _to_python_scalar(z)

    def __repr__(self):
        return f"Location(x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f})"

    def to_list(self):
        return [self.x, self.y, self.z]


class Rotation:

    def __init__(self, quaternion=None, order: str = "xyzw"):
        """
        Initialize rotation with quaternion only.

        Args:
            quaternion: Quaternion as list/array/tensor [x, y, z, w] or [w, x, y, z]
                       If None, defaults to identity quaternion [0, 0, 0, 1]
            order: "xyzw" (default) or "wxyz" to specify quaternion order
        """
        if quaternion is None:
            # 默认单位四元数
            self._quaternion = [0.0, 0.0, 0.0, 1.0]
        else:
            # 统一转换为 Python list
            quaternion = _to_python_list(quaternion)
            
            # 处理顺序转换
            if order == "wxyz":
                w, x, y, z = quaternion
                self._quaternion = [x, y, z, w]
            else:
                self._quaternion = quaternion

    def to_quaternion(self):
        """Return quaternion representation [x, y, z, w]"""
        return self._quaternion

    def __repr__(self):
        return f"Rotation(quaternion={self._quaternion})"


class Transform:

    def __init__(self, location=None, rotation=None, order: str = "xyzw"):
        """
        Initialize Transform with location and rotation.
        
        Args:
            location: Location object, or list/array [x, y, z]
                     Must be provided explicitly (no default)
            rotation: Rotation object, or list/array (quaternion)
                     Must be provided explicitly (no default)
        """
        # 处理 location - 不提供默认值，强制用户显式设置
        if location is None:
            self.location = None
        elif isinstance(location, Location):
            self.location = location
        elif hasattr(location, '__iter__'):
            # list, torch tensor, numpy array
            loc_list = _to_python_list(location)
            self.location = Location(loc_list[0], loc_list[1], loc_list[2])
        else:
            raise ValueError(f"Invalid location type: {type(location)}")
        
        # 处理 rotation - 不提供默认值，强制用户显式设置
        if rotation is None:
            self.rotation = None
        elif isinstance(rotation, Rotation):
            self.rotation = rotation
        elif hasattr(rotation, '__iter__'):
            self.rotation = Rotation(quaternion=rotation, order=order)
        else:
            raise ValueError(f"Invalid rotation type: {type(rotation)}")

    def __repr__(self):
        return f"Transform(location={self.location}, rotation={self.rotation})"


class Vector3D:

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = _to_python_scalar(x)
        self.y = _to_python_scalar(y)
        self.z = _to_python_scalar(z)

    def length(self) -> float:
        return (self.x**2 + self.y**2 + self.z**2) ** 0.5

    def __repr__(self):
        return f"Vector3D(x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f})"

    def to_list(self):
        return [self.x, self.y, self.z]
