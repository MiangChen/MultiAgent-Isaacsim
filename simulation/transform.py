class Location:

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def __repr__(self):
        return f"Location(x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f})"

    def to_list(self):
        return [self.x, self.y, self.z]


class Rotation:

    def __init__(
        self,
        pitch: float = 0.0,
        yaw: float = 0.0,
        roll: float = 0.0,
        quaternion: list = None,
    ):
        """
        Initialize rotation with either Euler angles or quaternion.

        Args:
            pitch: Pitch angle in degrees
            yaw: Yaw angle in degrees
            roll: Roll angle in degrees
            quaternion: Quaternion [x, y, z, w] (overrides Euler angles if provided)
        """
        if quaternion is not None:
            self._quaternion = list(quaternion)
            self.pitch = 0.0
            self.yaw = 0.0
            self.roll = 0.0
        else:
            self.pitch = float(pitch)
            self.yaw = float(yaw)
            self.roll = float(roll)
            self._quaternion = None

    def to_quaternion(self):
        """Return quaternion representation [x, y, z, w]"""
        if self._quaternion is not None:
            return self._quaternion

        # Convert Euler angles to quaternion
        from math import cos, sin, radians

        pitch_rad = radians(self.pitch)
        yaw_rad = radians(self.yaw)
        roll_rad = radians(self.roll)

        cy = cos(yaw_rad * 0.5)
        sy = sin(yaw_rad * 0.5)
        cp = cos(pitch_rad * 0.5)
        sp = sin(pitch_rad * 0.5)
        cr = cos(roll_rad * 0.5)
        sr = sin(roll_rad * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return [x, y, z, w]

    def __repr__(self):
        if self._quaternion is not None:
            return f"Rotation(quaternion={self._quaternion})"
        return f"Rotation(pitch={self.pitch:.2f}, yaw={self.yaw:.2f}, roll={self.roll:.2f})"


class Transform:

    def __init__(self, location: Location = None, rotation: Rotation = None):
        self.location = location if location is not None else Location()
        self.rotation = rotation if rotation is not None else Rotation()

    def __repr__(self):
        return f"Transform({self.location}, {self.rotation})"


class Vector3D:

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def length(self) -> float:
        return (self.x**2 + self.y**2 + self.z**2) ** 0.5

    def __repr__(self):
        return f"Vector3D(x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f})"

    def to_list(self):
        return [self.x, self.y, self.z]
