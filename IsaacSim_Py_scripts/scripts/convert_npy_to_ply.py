#!/usr/bin/env python3

import argparse
import numpy as np
import pcl


def main():
    parser = argparse.ArgumentParser(
        description="Convert numpy array to PLY point cloud file"
    )
    parser.add_argument("input", help="Input .npy file")
    parser.add_argument("output", help="Output .ply file")

    args = parser.parse_args()

    points = np.load(args.input)
    pointcloud = pcl._pcl.PointCloud(points.astype("float32"))
    pcl.save(pointcloud, args.output)


if __name__ == "__main__":
    main()
