from pathlib import Path
from typing import Optional

import numpy as np
import open3d as o3d


class PointCloudData:
    """
    Container for a point cloud.

    points : (N,3) float32
    colors : (N,3) float32 or None
    cloud  : Open3D PointCloud
    """

    def __init__(self, cloud: o3d.geometry.PointCloud,path=None):
        self.cloud = cloud
        self.path = path

        self.points = np.asarray(
            cloud.points,
            dtype=np.float32,
        )

        if cloud.has_colors():
            self.colors = np.asarray(
                cloud.colors,
                dtype=np.float32,
            )
        else:
            self.colors: Optional[np.ndarray] = None

    @property
    def num_points(self):
        return len(self.points)


def load_pcd(path):
    """
    Load a PCD file.

    Parameters
    ----------
    path : str | Path

    Returns
    -------
    PointCloudData
    """

    path = Path(path)

    if not path.exists():
        raise FileNotFoundError(path)

    cloud = o3d.io.read_point_cloud(str(path))

    if cloud.is_empty():
        raise RuntimeError(f"Empty point cloud: {path}")

    return PointCloudData(cloud,path)


def load_scene(path):
    return load_pcd(path)


def load_object(path):
    return load_pcd(path)