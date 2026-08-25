#!/usr/bin/env python3
"""
input( from anygrasp in camera coordinate frame ):
{
    "translation": [x, y, z],
    "rotation": [[...],[...],[...]],
    "width": 0.04,
    "depth": 0.02,
    "score": 0.8
}
(exactly what grasp_client.get_grasps()/get_best_grasp() return)

Process:
    convert rotation matrix to quaternion       (tf_utils.rotation_matrix_to_quaternion)
    transform grasp pose to base coordinate     (tf_utils.grasp_to_base_pose)
    add pre-grasp offset                        (tf_utils.offset_along_approach_axis)

output:
    (pregrasp_pose, grasp_pose) - both geometry_msgs/PoseStamped in
    base_frame, which MoveIt understands.
"""

import rospy

import tf_utils


# Meters, pulled back along the approach axis (local +X, see
# tf_utils module docstring) from the final grasp pose. Negative
# because "pre-grasp" means further away, not deeper in.
DEFAULT_PREGRASP_OFFSET = -0.05


def plan_grasp_poses(
    tf_buffer,
    grasp,
    base_frame=tf_utils.DEFAULT_BASE_FRAME,
    camera_frame=tf_utils.DEFAULT_CAMERA_FRAME,
    pregrasp_offset=DEFAULT_PREGRASP_OFFSET,
):
    """
    Parameters
    ----------
    tf_buffer : tf2_ros.Buffer
        Already subscribed via a tf2_ros.TransformListener.
    grasp : dict
        One grasp as returned by grasp_client.get_grasps()/get_best_grasp().
    pregrasp_offset : float
        Meters to pull back along the approach axis for the pre-grasp
        waypoint. Negative = further away from the object.

    Returns
    -------
    (pregrasp_pose, grasp_pose) : tuple of geometry_msgs/PoseStamped
        Both already in base_frame, ready to hand to MoveIt/executor.py.
    """

    grasp_pose = tf_utils.grasp_to_base_pose(
        tf_buffer,
        grasp,
        base_frame=base_frame,
        camera_frame=camera_frame,
    )

    pregrasp_pose = tf_utils.offset_along_approach_axis(
        grasp_pose,
        pregrasp_offset,
    )

    return pregrasp_pose, grasp_pose


if __name__ == "__main__":

    import argparse

    import tf2_ros
    import grasp_client

    parser = argparse.ArgumentParser(
        description="Standalone test: fetch a real grasp from AnyGrasp "
                    "and print the planned pre-grasp/grasp poses."
    )
    parser.add_argument("--scene", required=True)
    parser.add_argument("--object", required=True)
    parser.add_argument("--server-url", default=grasp_client.DEFAULT_SERVER_URL)
    parser.add_argument("--base-frame", default=tf_utils.DEFAULT_BASE_FRAME)
    parser.add_argument("--camera-frame", default=tf_utils.DEFAULT_CAMERA_FRAME)
    parser.add_argument("--pregrasp-offset", type=float,
                         default=DEFAULT_PREGRASP_OFFSET)
    args = parser.parse_args()

    rospy.init_node("grasp_planner_test", anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)

    grasp = grasp_client.get_best_grasp(
        args.scene, args.object, server_url=args.server_url
    )

    if grasp is None:
        rospy.logerr("No grasp returned, nothing to plan.")
        raise SystemExit(1)

    pregrasp_pose, grasp_pose = plan_grasp_poses(
        tf_buffer,
        grasp,
        base_frame=args.base_frame,
        camera_frame=args.camera_frame,
        pregrasp_offset=args.pregrasp_offset,
    )

    rospy.loginfo("Pre-grasp pose:\n%s", pregrasp_pose)
    rospy.loginfo("Grasp pose:\n%s", grasp_pose)
