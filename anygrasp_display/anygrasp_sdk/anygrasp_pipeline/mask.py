import numpy as np
from scipy.spatial import cKDTree


def compute_object_mask(
        scene_points: np.ndarray,
        object_points: np.ndarray,
        threshold: float = 0.005,
):
    """
    Compute a point-wise object mask for a scene point cloud.

    Parameters
    ----------
    scene_points : np.ndarray
        Full scene point cloud.
        Shape: (N,3)
        XYZ coordinates in camera frame.

    object_points : np.ndarray
        Object point cloud.
        Shape: (M,3)
        XYZ coordinates in the same camera frame.

    threshold : float
        Maximum distance (meters) to consider a scene point
        belonging to the object.

    Returns
    -------
    mask : np.ndarray
        Boolean mask of length N.
        True  -> scene point belongs to object
        False -> background
    """

    if scene_points.ndim != 2 or scene_points.shape[1] != 3:
        raise ValueError(
            "scene_points must have shape (N,3)"
        )

    if object_points.ndim != 2 or object_points.shape[1] != 3:
        raise ValueError(
            "object_points must have shape (M,3)"
        )


    # Build nearest-neighbor search structure
    object_tree = cKDTree(object_points)


    # Find closest object point for every scene point
    distances, _ = object_tree.query(
        scene_points,
        k=1
    )


    # Points close to object cloud are considered object points
    mask = distances < threshold


    return mask



def print_mask_statistics(mask):
    """
    Print useful debugging information.
    """

    total = len(mask)
    object_points = np.count_nonzero(mask)

    print("=" * 50)
    print("Object mask statistics")
    print("=" * 50)

    print(f"Total scene points : {total}")
    print(f"Object points      : {object_points}")
    print(
        f"Object ratio       : "
        f"{100*object_points/total:.2f}%"
    )