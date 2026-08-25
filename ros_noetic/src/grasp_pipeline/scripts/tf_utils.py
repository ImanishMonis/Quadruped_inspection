#!/usr/bin/env python3
"""
Camera -> robot-base frame conversion for AnyGrasp grasp poses.

AnyGrasp's own convention (anygrasp_sdk/grasp_detection/USAGE.md, Note 2):
  - gripper local +X axis = approach direction
  - gripper local +Y axis = open/close direction
  - gripper_tip_position = translation + depth * rotation_matrix[:, 0]

This robot's end_effector_link (open_manipulator_x_arm.urdf.xacro):
  - end_effector_joint is FIXED, rpy="0 0 0" relative to link5
    -> end_effector_link has the SAME orientation as link5.
  - end_effector_link and gripper_left_link both sit further out along
    link5's local +X, and gripper_left_joint's prismatic axis is local
    Y -> local +X is this robot's approach direction too, +Y is
    open/close.

Both conventions already agree axis-for-axis (X=approach, Y=open/close),
so ROTATION_OFFSET_QUATERNION below defaults to identity. This is a
reasoned default from reading both spec docs, NOT a substitute for the
AGENT_SESSION.md "known trap" check: verify empirically in RViz (does
the arm approach along the axis you'd expect, not sideways or
backwards?) before trusting it for a real pick.
"""

import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped transforms)
from geometry_msgs.msg import PoseStamped
from tf.transformations import (
    quaternion_from_matrix,
    quaternion_multiply,
    quaternion_matrix,
)


# Fixed rotation applied to every incoming AnyGrasp rotation matrix
# before use, to correct for a gripper-frame-convention mismatch.
# Identity by default - see module docstring for why. Expressed as a
# quaternion (x, y, z, w) so it composes with quaternion_multiply; if
# RViz shows the arm approaching sideways/backwards, this is the one
# constant to change (e.g. a +/-90 deg rotation about the axis that's
# actually wrong).
ROTATION_OFFSET_QUATERNION = (0.0, 0.0, 0.7071068, 0.7071068)

DEFAULT_BASE_FRAME = "link1"
DEFAULT_CAMERA_FRAME = "camera_optical_frame"

# Empirical correction for a residual mount-offset mismatch between the
# ROS URDF's assumed camera mount (camera_joint's xyz="0.04 0.00 0.05"
# relative to link5) and Isaac Sim's actual USD camera prim placement,
# which was authored directly in the scene and never round-tripped back
# into this URDF. Measured once, at one specific observe pose (arm at
# x=0.02,z=0.25,pitch=40deg in link1 frame): a known cube at link1
# (0.18, 0.0, 0.09) showed up in the captured point cloud, transformed
# through TF, centered at (0.192, 0.049, 0.172) - error (0.012, 0.049,
# 0.082). This constant subtracts that measured error in BASE_FRAME.
#
# CAVEAT: this is a base-frame (world-relative) constant, calibrated at
# ONE arm pose. If the true cause is a camera-local mount offset (the
# likely case - Isaac's real mount just differs by a few cm/degrees
# from what the URDF assumes), this correction will drift as the arm's
# orientation changes and camera_joint moves the error vector with it.
# Re-measure (see END_TO_END_TESTING.md) if observe poses change
# significantly, or replace this with a corrected camera_joint xyz/rpy
# once Isaac's actual USD mount transform is read directly - see
# AGENT_SESSION.md for that follow-up.
# GRASP_POSITION_CORRECTION = (-0.012, -0.049, -0.082)
GRASP_POSITION_CORRECTION = (0.0, 0.0, 0.0)


def rotation_matrix_to_quaternion(rotation_matrix):
    """
    3x3 rotation matrix (nested list or ndarray, as returned by
    AnyGrasp) -> quaternion (x, y, z, w).
    """

    r = np.asarray(rotation_matrix, dtype=np.float64)

    m = np.eye(4)
    m[:3, :3] = r

    return quaternion_from_matrix(m)


def grasp_to_camera_pose(grasp, frame_id=DEFAULT_CAMERA_FRAME):
    """
    AnyGrasp grasp dict (score/width/depth/translation/rotation, as
    returned by grasp_client.get_grasps) -> geometry_msgs/PoseStamped
    in the camera frame, with ROTATION_OFFSET_QUATERNION applied.
    """

    quat = rotation_matrix_to_quaternion(grasp["rotation"])
    quat = quaternion_multiply(quat, ROTATION_OFFSET_QUATERNION)

    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = rospy.Time(0)

    tx, ty, tz = grasp["translation"]
    pose.pose.position.x = tx
    pose.pose.position.y = ty
    pose.pose.position.z = tz

    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]

    return pose


def transform_pose(tf_buffer, pose_stamped, target_frame, timeout=None):
    """
    Transform a PoseStamped into target_frame using tf2. Raises the
    usual tf2_ros exceptions (LookupException, ExtrapolationException,
    ConnectivityException) on failure - let the caller decide how to
    handle a missing/late TF rather than swallowing it here.
    """

    if timeout is None:
        timeout = rospy.Duration(1.0)

    transform = tf_buffer.lookup_transform(
        target_frame,
        pose_stamped.header.frame_id,
        pose_stamped.header.stamp,
        timeout,
    )

    return tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)


def grasp_to_base_pose(
    tf_buffer,
    grasp,
    base_frame=DEFAULT_BASE_FRAME,
    camera_frame=DEFAULT_CAMERA_FRAME,
):
    """
    Full conversion: AnyGrasp grasp dict (camera frame) -> PoseStamped
    in base_frame, ready for MoveIt.
    """

    camera_pose = grasp_to_camera_pose(grasp, frame_id=camera_frame)
    base_pose = transform_pose(tf_buffer, camera_pose, base_frame)

    # See GRASP_POSITION_CORRECTION's docstring above for what this is
    # and its caveats - it's an empirical fudge, not a real fix.
    base_pose.pose.position.x += GRASP_POSITION_CORRECTION[0]
    base_pose.pose.position.y += GRASP_POSITION_CORRECTION[1]
    base_pose.pose.position.z += GRASP_POSITION_CORRECTION[2]
    return base_pose


def offset_along_approach_axis(pose_stamped, distance):
    """
    Shift a PoseStamped by `distance` meters along its own local +X
    axis (the approach axis for both AnyGrasp and this robot's
    end_effector_link - see module docstring). Positive distance moves
    further along +X (deeper into the grasp, e.g. using AnyGrasp's own
    `depth`); negative pulls back (pre-grasp / retreat waypoints).
    """

    q = pose_stamped.pose.orientation
    rot = quaternion_matrix([q.x, q.y, q.z, q.w])
    approach_axis = rot[:3, 0]

    offset_pose = PoseStamped()
    offset_pose.header = pose_stamped.header
    offset_pose.pose.orientation = pose_stamped.pose.orientation

    offset_pose.pose.position.x = (
        pose_stamped.pose.position.x + distance * approach_axis[0]
    )
    offset_pose.pose.position.y = (
        pose_stamped.pose.position.y + distance * approach_axis[1]
    )
    offset_pose.pose.position.z = (
        pose_stamped.pose.position.z + distance * approach_axis[2]
    )

    return offset_pose


if __name__ == "__main__":
    # Standalone empirical check: convert a grasp (from a live
    # grasp_client call, or a fixed test grasp) into base_frame and
    # keep broadcasting it as a TF frame so it can be eyeballed in
    # RViz against the real arm, per the module docstring's warning.
    import argparse

    import grasp_client

    parser = argparse.ArgumentParser(
        description="Broadcast a converted grasp pose as a TF frame "
                    "for visual inspection in RViz."
    )
    parser.add_argument("--scene", help="scene.pcd; if omitted, uses a "
                         "fixed identity-ish test grasp instead of AnyGrasp")
    parser.add_argument("--object", help="object.pcd (required with --scene)")
    parser.add_argument("--base-frame", default=DEFAULT_BASE_FRAME)
    parser.add_argument("--camera-frame", default=DEFAULT_CAMERA_FRAME)
    parser.add_argument("--server-url", default=grasp_client.DEFAULT_SERVER_URL)
    args = parser.parse_args()

    rospy.init_node("tf_utils_test", anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    broadcaster = tf2_ros.TransformBroadcaster()

    if args.scene and args.object:
        grasp = grasp_client.get_best_grasp(
            args.scene, args.object, server_url=args.server_url
        )
        if grasp is None:
            rospy.logerr("No grasp returned, nothing to visualize.")
            raise SystemExit(1)
    else:
        rospy.logwarn(
            "No --scene/--object given; broadcasting a fixed test "
            "grasp 0.1m in front of the camera instead of a real one."
        )
        grasp = {
            "translation": [0.0, 0.0, 0.1],
            "rotation": np.eye(3).tolist(),
            "score": 0.0,
            "width": 0.0,
            "depth": 0.0,
        }

    rospy.sleep(1.0)  # give tf2 a moment to fill its buffer

    try:
        base_pose = grasp_to_base_pose(
            tf_buffer, grasp,
            base_frame=args.base_frame,
            camera_frame=args.camera_frame,
        )
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr("TF lookup failed: %s", e)
        raise SystemExit(1)

    rospy.loginfo("Grasp pose in %s:\n%s", args.base_frame, base_pose)

    from geometry_msgs.msg import TransformStamped

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = args.base_frame
        t.child_frame_id = "grasp_pose"
        t.transform.translation.x = base_pose.pose.position.x
        t.transform.translation.y = base_pose.pose.position.y
        t.transform.translation.z = base_pose.pose.position.z
        t.transform.rotation = base_pose.pose.orientation
        broadcaster.sendTransform(t)
        rate.sleep()
