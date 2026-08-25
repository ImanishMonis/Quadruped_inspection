import numpy as np


def pose_to_matrix(rotation, translation):
    """
    Convert rotation matrix + translation vector into a 4x4 transform.

    Parameters
    ----------
    rotation : (3,3)
    translation : (3,)

    Returns
    -------
    (4,4) homogeneous transform
    """

    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = rotation
    T[:3, 3] = translation

    return T


def matrix_to_pose(T):
    """
    Convert homogeneous transform into rotation + translation.
    """

    rotation = T[:3, :3]
    translation = T[:3, 3]

    return rotation, translation


def transform_grasp(grasp, T_base_camera):
    """
    Transform a grasp from camera frame to robot base frame.

    Parameters
    ----------
    grasp
        One AnyGrasp grasp object.

    T_base_camera : (4,4)

    Returns
    -------
    rotation_base
    translation_base
    """

    T_camera_grasp = pose_to_matrix(
        grasp.rotation_matrix,
        grasp.translation,
    )

    T_base_grasp = T_base_camera @ T_camera_grasp

    rotation, translation = matrix_to_pose(T_base_grasp)

    return rotation, translation


def transform_grasp_group(grasp_group, T_base_camera):
    """
    Transform every grasp in a GraspGroup.

    Returns
    -------
    list of tuples

        [
            (rotation, translation),
            ...
        ]
    """

    transformed = []

    for grasp in grasp_group:
        transformed.append(
            transform_grasp(
                grasp,
                T_base_camera,
            )
        )

    return transformed


def identity_camera_to_base():
    """
    Temporary transform used while ROS TF is unavailable.

    Later this function will be replaced with a TF lookup.
    """

    return np.eye(4, dtype=np.float32)