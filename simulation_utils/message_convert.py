import numpy as np

import carb

from isaacsim.core.utils import extensions, stage

from sensor_msgs.msg import PointCloud2, PointField, Image


@carb.profiler.profile
def create_pc2_msg(header, points):
    """Create PointCloud2 message from points."""
    assert points.dtype == np.float32
    assert len(points.shape) == 3
    assert points.shape[2] == 3

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    point_step = points.shape[2] * 4
    pc = PointCloud2(
        header=header,
        height=points.shape[0],
        width=points.shape[1],
        is_dense=True,
        is_bigendian=False,
        fields=fields,
        point_step=point_step,
        row_step=points.shape[1] * point_step,
        data=points.tobytes(),
    )
    return pc


@carb.profiler.profile
def create_image_msg(header, depth):
    """Create Image message from depth data."""
    assert depth.dtype == np.float32
    assert len(depth.shape) == 2

    img_msg = Image(
        header=header,
        height=depth.shape[0],
        width=depth.shape[1],
        encoding="32FC1",
        is_bigendian=False,
        data=depth.tobytes(),
        step=(depth.shape[1] * 4),
    )
    return img_msg
