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

import math

import rospy
from geometry_msgs.msg import PoseStamped

import base_teleport
import tf_utils


# Meters, pulled back along the approach axis (local +X, see
# tf_utils module docstring) from the final grasp pose. Negative
# because "pre-grasp" means further away, not deeper in.
DEFAULT_PREGRASP_OFFSET = -0.05

# Degrees. A grasp's approach vector, expressed in the world frame, must
# have a Z-component within sin(this angle) of zero to count as "level" -
# see select_level_grasp below.
DEFAULT_MAX_TILT_DEG = 15.0


def select_level_grasp(
    tf_buffer,
    grasps,
    reference_frame=tf_utils.DEFAULT_BASE_FRAME,
    camera_frame=tf_utils.DEFAULT_CAMERA_FRAME,
    max_tilt_deg=DEFAULT_MAX_TILT_DEG,
):
    """
    From a best-first list of AnyGrasp candidates (grasp_client.get_grasps()),
    return the highest-scoring one whose approach is level - i.e. the
    gripper stays roughly parallel to the ground, reaching in sideways
    rather than tilting up/down or coming in top-down.

    A grasp's approach direction (local +X, see tf_utils module docstring)
    is transformed into reference_frame; "level" means its Z-component is
    close to zero (within sin(max_tilt_deg) of it), since Z is the only
    component that can make the arm pitch up or down to reach it. This is
    checked against a gravity-aligned frame, not the camera frame, because
    "parallel to the ground" is a statement about gravity, not about
    wherever the camera happens to be pointed.

    reference_frame defaults to link1, not world (changed 2026-09-17): the
    base-teleport virtual joint is planar (x, y, yaw only), and a yaw
    rotation about Z leaves any vector's Z-component unchanged, so the test
    gives an identical answer in either frame - but link1 is always in the
    TF tree, whereas world only exists while a BaseTeleporter happens to be
    broadcasting. Using world here meant this raised LookupException in any
    context without an active teleporter. It also matches the up-vector
    reference live_capture.py records for the AnyGrasp side (see
    write_capture_metadata / main.py --level-only), so both containers
    select the same grasp.

    Returns
    -------
    dict or None
        The first (best-scoring) grasp meeting the tilt threshold, or
        None if no candidate in `grasps` qualifies - try again with a
        larger top_k from grasp_client.get_grasps(), or a larger
        max_tilt_deg, rather than assuming no level grasp exists at all.
    """

    max_tilt_z = math.sin(math.radians(max_tilt_deg))

    for grasp in grasps:
        ref_pose = tf_utils.grasp_to_base_pose(
            tf_buffer, grasp, base_frame=reference_frame, camera_frame=camera_frame,
        )
        approach = tf_utils.extract_approach_vector(ref_pose)
        if abs(approach[2]) <= max_tilt_z:
            return grasp

    return None


def plan_grasp_with_base_relocation(
    tf_buffer,
    grasp,
    base_frame=tf_utils.DEFAULT_BASE_FRAME,
    world_frame=tf_utils.DEFAULT_WORLD_FRAME,
    camera_frame=tf_utils.DEFAULT_CAMERA_FRAME,
    pregrasp_offset=DEFAULT_PREGRASP_OFFSET,
    standoff_distance=base_teleport.DEFAULT_STANDOFF_DISTANCE,
    base_topic=base_teleport.DEFAULT_GO1_TOPIC,
    teleporter=None,
    publish_base_cmd=True,
):
    """
    Plans grasp with mobile base relocation:
    1. Converts AnyGrasp pose into global world frame.
    2. Computes optimal mobile base placement (x, y, yaw) and relative
       displacement (dx, dy, dtheta) for Unitree Go1.
    3. Publishes relative displacement to base_topic.
    4. Teleports the arm base (world -> link1) using teleporter.
    5. Transforms grasp and pre-grasp poses into the teleported base_frame
       preserving full 3D orientation.

    Returns
    -------
    (pregrasp_pose, grasp_pose, base_info)
    """
    # 1. Transform grasp to world coordinates
    world_pose = tf_utils.grasp_to_world_pose(
        tf_buffer,
        grasp,
        world_frame=world_frame,
        camera_frame=camera_frame,
    )

    # 2. Get current base position in world if available
    curr_x, curr_y, curr_yaw = 0.0, 0.0, 0.0
    try:
        t_curr = tf_buffer.lookup_transform(world_frame, base_frame, rospy.Time(0), rospy.Duration(0.5))
        curr_x = t_curr.transform.translation.x
        curr_y = t_curr.transform.translation.y
        q = t_curr.transform.rotation
        from tf.transformations import euler_from_quaternion
        _, _, curr_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    except Exception as exc:
        rospy.logwarn("Could not lookup current %s -> %s: %s. Assuming (0,0,0).", world_frame, base_frame, exc)

    # 3. Compute base placement
    base_info = base_teleport.compute_base_placement(
        world_pose,
        standoff_distance=standoff_distance,
        current_base_pos=(curr_x, curr_y),
        current_base_yaw=curr_yaw,
    )

    dx, dy, dtheta = base_info["relative_displacement"]
    tx, ty, tyaw = base_info["target_base_world"]

    # 4. Publish teleport / move command for the teammate's mobile base node
    if publish_base_cmd:
        base_teleport.publish_teleport_params(dx, dy, dtheta, topic=base_topic)
        # Absolute pose for an Isaac Sim-side listener to actually move the
        # robot prim - the TF-only teleport below does not affect planning,
        # see base_teleport.publish_absolute_pose's docstring / BUG-17.
        base_teleport.publish_absolute_pose(tx, ty, tyaw)

    # 5. Teleport arm base in simulation / TF
    if teleporter is not None:
        teleporter.set_pose(tx, ty, tyaw)
        rospy.sleep(0.15)  # Allow TF buffer to register updated transform

    # 6. Transform grasp into the updated arm base frame.
    #
    # Deliberately re-transforms the already-computed, fixed world_pose
    # from step 1 - NOT tf_utils.grasp_to_base_pose(grasp, ...), which
    # would re-derive the pose straight from the raw camera-frame
    # measurement via camera_optical_frame -> link1. That lookup only
    # ever needs the arm's OWN internal kinematic chain (camera_link ->
    # link5 -> ... -> link1), since both frames sit within the same rigid
    # arm assembly - it structurally cannot reflect any base relocation,
    # physical or TF-based, because moving the whole arm as one rigid body
    # can't change the camera's pose *relative to link1*. world_pose, by
    # contrast, is anchored in the fixed "world" frame, which shares no
    # other path to "link1" besides the world -> link1 edge itself - so
    # transforming it into base_frame is the one calculation that's
    # actually forced to pick up wherever BaseTeleporter (or, once it's
    # wired up, Isaac Sim's real base position) says link1 now is.
    #
    # world_pose.header.stamp must be reset to Time(0) ("latest") before
    # this second transform - tf2_geometry_msgs.do_transform_pose() (used
    # inside grasp_to_world_pose above) stamps its OUTPUT with the
    # TRANSFORM's timestamp, not Time(0), so world_pose is now pinned to
    # a specific, frozen, pre-teleport instant. Without this reset,
    # looking it up again queries "what was world -> link1 back then",
    # which is the identity value from before teleporter.set_pose() was
    # ever called - not a TF bug, just the wrong question. Verified live:
    # an isolated BaseTeleporter test confirmed the broadcast itself
    # updates correctly within 0.15s: this stale-stamp reuse was the
    # actual remaining cause of every "Planned Grasp in Teleported Arm
    # Frame" == "Grasp world pos" result throughout BUG-17's
    # investigation, on top of the three earlier, also-real bugs.
    # See AGENT_SESSION.md BUG-17 for the full diagnosis.
    world_pose_now = PoseStamped()
    world_pose_now.header.frame_id = world_pose.header.frame_id
    world_pose_now.header.stamp = rospy.Time(0)
    world_pose_now.pose = world_pose.pose

    grasp_pose = tf_utils.transform_pose(tf_buffer, world_pose_now, base_frame)
    grasp_pose.pose.position.x += tf_utils.GRASP_POSITION_CORRECTION[0]
    grasp_pose.pose.position.y += tf_utils.GRASP_POSITION_CORRECTION[1]
    grasp_pose.pose.position.z += tf_utils.GRASP_POSITION_CORRECTION[2]

    pregrasp_pose = tf_utils.offset_along_approach_axis(
        grasp_pose,
        pregrasp_offset,
    )

    return pregrasp_pose, grasp_pose, base_info


def plan_grasp_poses(
    tf_buffer,
    grasp,
    base_frame=tf_utils.DEFAULT_BASE_FRAME,
    camera_frame=tf_utils.DEFAULT_CAMERA_FRAME,
    pregrasp_offset=DEFAULT_PREGRASP_OFFSET,
    teleport_base=True,
    standoff_distance=base_teleport.DEFAULT_STANDOFF_DISTANCE,
    base_topic=base_teleport.DEFAULT_GO1_TOPIC,
    teleporter=None,
    publish_base_cmd=True,
    return_base_info=True,
):
    """
    High-level entry point for grasp planning.
    If teleport_base is True, executes mobile base relocation and teleportation.
    """
    if teleport_base:
        pregrasp, grasp_p, base_info = plan_grasp_with_base_relocation(
            tf_buffer,
            grasp,
            base_frame=base_frame,
            camera_frame=camera_frame,
            pregrasp_offset=pregrasp_offset,
            standoff_distance=standoff_distance,
            base_topic=base_topic,
            teleporter=teleporter,
            publish_base_cmd=publish_base_cmd,
        )
        if return_base_info:
            return pregrasp, grasp_p, base_info
        return pregrasp, grasp_p

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

    if return_base_info:
        return pregrasp_pose, grasp_pose, None
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
