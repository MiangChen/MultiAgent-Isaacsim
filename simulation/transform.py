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
    
    def __init__(self, pitch: float = 0.0, yaw: float = 0.0, roll: float = 0.0):
        self.pitch = float(pitch)
        self.yaw = float(yaw)
        self.roll = float(roll)
    
    def __repr__(self):
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
        return (self.x**2 + self.y**2 + self.z**2)**0.5
    
    def __repr__(self):
        return f"Vector3D(x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f})"
    
    def to_list(self):
        return [self.x, self.y, self.z]
