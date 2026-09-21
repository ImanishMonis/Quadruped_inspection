#!/usr/bin/env python3
"""
Full pick sequence, driven through MoveIt:

    pre-grasp -> open gripper -> cartesian approach -> close gripper
    -> attach -> retreat

MoveIt reaches Isaac Sim via trajectory_bridge (see
X_moveit_config/launch/fake_moveit_controller_manager.launch.xml), so
this only talks moveit_commander - it has no idea Isaac Sim exists.

Group/state names come straight from config/open_manipulator_x.srdf:
    x_arm group  : joint1-joint4
    hand  group  : gripper_left_joint, named states Grip_Open/Grip_Close
    end_effector : parent_link="end_effector_link", group="hand"
"""

import math
import rospy
import moveit_commander
import tf2_ros

import base_teleport
import grasp_planner
import tf_utils


ARM_GROUP = "x_arm"
HAND_GROUP = "hand"

GRIP_OPEN_STATE = "Grip_Open"
GRIP_CLOSE_STATE = "Grip_Close"

# Meters, lifted straight UP (link1 +Z) from the grasp pose after closing
# the gripper. Changed 2026-09-20 at the user's request: this used to be
# a negative offset along the grasp's own approach axis (pulling back the
# way it came in), which for a level/sideways grasp just drags the object
# backward along the surface instead of actually lifting it off.
DEFAULT_RETREAT_OFFSET = 0.08

CARTESIAN_EEF_STEP = 0.01
CARTESIAN_MIN_FRACTION = 0.9


class PickExecutionError(RuntimeError):
    pass


def _run_cartesian_path(arm, waypoints, eef_step=CARTESIAN_EEF_STEP,
                         min_fraction=CARTESIAN_MIN_FRACTION,
                         description="cartesian path"):

    plan, fraction = arm.compute_cartesian_path(waypoints, eef_step)

    if fraction < min_fraction:
        raise PickExecutionError(
            f"{description} only {fraction * 100:.0f}% complete "
            f"(need >= {min_fraction * 100:.0f}%)"
        )

    if not arm.execute(plan, wait=True):
        raise PickExecutionError(f"Failed to execute {description}")

    arm.stop()


def execute_pick(
    arm,
    hand,
    tf_buffer,
    grasp,
    object_name=None,
    pregrasp_offset=grasp_planner.DEFAULT_PREGRASP_OFFSET,
    retreat_offset=DEFAULT_RETREAT_OFFSET,
    standoff_distance=base_teleport.DEFAULT_STANDOFF_DISTANCE,
    base_topic=base_teleport.DEFAULT_GO1_TOPIC,
    teleport_base=True,
    teleporter=None,
    planning_time=10.0,
    planning_attempts=10,
    pose_broadcaster=None,
):
    """
    Parameters
    ----------
    arm, hand : moveit_commander.MoveGroupCommander
        For ARM_GROUP and HAND_GROUP respectively.
    tf_buffer : tf2_ros.Buffer
        Already subscribed via a tf2_ros.TransformListener.
    grasp : dict
        One grasp as returned by grasp_client.get_grasps()/get_best_grasp().
    object_name : str, optional
        Planning-scene object name to attach after closing the gripper.
    standoff_distance : float
        Horizontal distance between arm base and grasp center (meters).
    base_topic : str
        ROS topic to publish Go1 relative displacement commands to.
    teleport_base : bool
        Whether to calculate and teleport the arm base in simulation / TF.
    teleporter : BaseTeleporter, optional
        Active BaseTeleporter instance for dynamic TF broadcasting.
    pose_broadcaster : tf_utils.PoseBroadcaster, optional
        If given, publishes "grasp_pose" and "retreat_pose" as live TF
        frames as they're computed - add them in RViz (Add -> TF, or
        Add -> Axes with that frame name) to see exactly what this run
        actually computed and acted on, not a separately re-run
        diagnostic that might see a different scene by the time you run
        it. Added 2026-09-17 at the user's request.

    Raises
    ------
    PickExecutionError
        On any step failing. Earlier steps are not undone.
    """
    arm.set_planning_time(planning_time)
    arm.set_num_planning_attempts(planning_attempts)

    pregrasp_pose, grasp_pose, base_info = grasp_planner.plan_grasp_poses(
        tf_buffer,
        grasp,
        pregrasp_offset=pregrasp_offset,
        teleport_base=teleport_base,
        standoff_distance=standoff_distance,
        base_topic=base_topic,
        teleporter=teleporter,
        return_base_info=True,
    )

    if pose_broadcaster is not None:
        pose_broadcaster.set_pose("grasp_pose", grasp_pose)

    if base_info is not None:
        dx, dy, dtheta = base_info["relative_displacement"]
        bx, by, byaw = base_info["target_base_world"]
        rospy.loginfo("=" * 50)
        rospy.loginfo("[BaseRelocation] Unitree Go1 Base Teleportation Parameters:")
        rospy.loginfo("  Target World Pose : x=%.4f m, y=%.4f m, yaw=%.2f deg", bx, by, math.degrees(byaw))
        rospy.loginfo("  Command (Pose2D)  : delta_x=%.4f m, delta_y=%.4f m, delta_yaw=%.2f deg", dx, dy, math.degrees(dtheta))
        rospy.loginfo("  Published to topic: %s", base_topic)
        rospy.loginfo("=" * 50)

    print("*" * 20)
    print("Planned Grasp in Teleported Arm Frame (link1):")
    print(f"Target X: {grasp_pose.pose.position.x:.4f} m")
    print(f"Target Y: {grasp_pose.pose.position.y:.4f} m")
    print(f"Target Z: {grasp_pose.pose.position.z:.4f} m")
    print(f"Radial Distance: {(grasp_pose.pose.position.x**2 + grasp_pose.pose.position.y**2)**0.5:.4f} m")
    print(f"Orientation (quat): [{grasp_pose.pose.orientation.x:.3f}, {grasp_pose.pose.orientation.y:.3f}, {grasp_pose.pose.orientation.z:.3f}, {grasp_pose.pose.orientation.w:.3f}]")
    print("*" * 20)

    # 1. Open gripper before moving anywhere near the object.
    hand.set_named_target(GRIP_OPEN_STATE)
    if not hand.go(wait=True):
        raise PickExecutionError("Failed to open gripper")
    hand.stop()

    # 2. Move directly to the grasp pose - no pre-grasp waypoint.
    # Removed 2026-09-17 at the user's request: the pre-grasp -> grasp
    # segment was already reduced to a position-only target (see git
    # history), and with kinematics.yaml's position_only_ik=true (4-DOF
    # arm, can't achieve AnyGrasp's 6-DOF orientation) there's no
    # meaningful orientation control gained by staging through an
    # intermediate pre-grasp point first - it was just adding a second
    # position-only move (and a second chance to fail) ahead of the one
    # that actually matters. Straight to grasp_pose instead.
    arm.set_position_target([
        grasp_pose.pose.position.x,
        grasp_pose.pose.position.y,
        grasp_pose.pose.position.z,
    ])
    if not arm.go(wait=True):
        raise PickExecutionError("Failed to reach grasp pose")
    arm.stop()
    arm.clear_pose_targets()

    # 3. Close the gripper on the object.
    hand.set_named_target(GRIP_CLOSE_STATE)
    if not hand.go(wait=True):
        raise PickExecutionError("Failed to close gripper")
    hand.stop()

    # 4. Attach to the planning scene, if we have something to attach.
    if object_name:
        arm.attach_object(object_name)
    else:
        rospy.logwarn(
            "No object_name given, skipping attach - the planning "
            "scene has no matching collision object to attach yet."
        )

    # 5. Vertical lift, straight up off the surface.
    retreat_pose = tf_utils.lift_pose(
        grasp_pose,
        retreat_offset,
    )
    if pose_broadcaster is not None:
        pose_broadcaster.set_pose("retreat_pose", retreat_pose)
    try:
        _run_cartesian_path(
            arm,
            [grasp_pose.pose, retreat_pose.pose],
            description="retreat",
        )
    except Exception:
        arm.set_position_target([
            retreat_pose.pose.position.x,
            retreat_pose.pose.position.y,
            retreat_pose.pose.position.z,
        ])
        arm.go(wait=True)
    arm.stop()
    arm.clear_pose_targets()

    return True
# def execute_pick(
#     arm,
#     hand,
#     tf_buffer,
#     grasp,
#     object_name=None,
#     pregrasp_offset=grasp_planner.DEFAULT_PREGRASP_OFFSET,
#     retreat_offset=DEFAULT_RETREAT_OFFSET,
#     planning_time=10.0,
#     planning_attempts=10,
# ):
#     arm.set_planning_time(planning_time)
#     arm.set_num_planning_attempts(planning_attempts)

#     # 1. Get the poses intended for the FINGERTIPS
#     fingertip_pregrasp, fingertip_grasp = grasp_planner.plan_grasp_poses(
#         tf_buffer,
#         grasp,
#         pregrasp_offset=pregrasp_offset,
#     )

#     # 2. THE FIX: Shift the target backward by 12.6 cm because MoveIt is driving 
#     # the wrist (link5), not the fingertips.
#     TCP_OFFSET_METERS = 0.126 
#     wrist_pregrasp = tf_utils.offset_along_approach_axis(fingertip_pregrasp, TCP_OFFSET_METERS)
#     wrist_grasp = tf_utils.offset_along_approach_axis(fingertip_grasp, TCP_OFFSET_METERS)

#     print("*"*15)
#     print(f"Fingertip Pre-Grasp X: {fingertip_pregrasp.pose.position.x:.4f}")
#     print(f"Wrist Pre-Grasp X (Reachable): {wrist_pregrasp.pose.position.x:.4f}")
#     print("*"*15)

#     # 3. Open gripper
#     hand.set_named_target(GRIP_OPEN_STATE)
#     if not hand.go(wait=True):
#         raise PickExecutionError("Failed to open gripper")
#     hand.stop()

#     # 4. HOVER POSE: Move 10cm above the pre-grasp to prevent hitting the floor
#     arm.set_position_target([
#         wrist_pregrasp.pose.position.x, 
#         wrist_pregrasp.pose.position.y, 
#         wrist_pregrasp.pose.position.z + 0.10
#     ])
#     if not arm.go(wait=True):
#         raise PickExecutionError("Failed to reach hover pose")
#     arm.stop()
#     arm.clear_pose_targets()

#     # 5. ALIGN WRIST: Force wrist to point forward so fingers don't drag
#     current_joint_values = arm.get_current_joint_values()
#     current_joint_values[3] = 0.0  # joint4 (wrist pitch)
#     arm.set_joint_value_target(current_joint_values)
#     arm.go(wait=True)
#     arm.stop()
#     arm.clear_pose_targets()

#     # 6. DROP TO PRE-GRASP
#     arm.set_position_target([
#         wrist_pregrasp.pose.position.x, 
#         wrist_pregrasp.pose.position.y, 
#         wrist_pregrasp.pose.position.z
#     ])
#     if not arm.go(wait=True):
#         raise PickExecutionError("Failed to reach pre-grasp pose")
#     arm.stop()
#     arm.clear_pose_targets()

#     # 7. MOVE TO GRASP
#     arm.set_position_target([
#         wrist_grasp.pose.position.x, 
#         wrist_grasp.pose.position.y, 
#         wrist_grasp.pose.position.z
#     ])
#     if not arm.go(wait=True):
#         raise PickExecutionError("Failed to reach grasp pose")
#     arm.stop()
#     arm.clear_pose_targets()

#     # 8. Close the gripper
#     hand.set_named_target(GRIP_CLOSE_STATE)
#     if not hand.go(wait=True):
#         raise PickExecutionError("Failed to close gripper")
#     hand.stop()

#     if object_name:
#         arm.attach_object(object_name)

#     # 9. Retreat by lifting straight up
#     arm.set_position_target([
#         wrist_grasp.pose.position.x, 
#         wrist_grasp.pose.position.y, 
#         wrist_grasp.pose.position.z + 0.15
#     ])
#     if not arm.go(wait=True):
#         raise PickExecutionError("Failed to retreat")
#     arm.stop()
#     arm.clear_pose_targets()
 
#     return True


if __name__ == "__main__":

    import argparse
    import sys

    import grasp_client

    parser = argparse.ArgumentParser(
        description="Standalone test: fetch a real grasp from AnyGrasp "
                    "and execute the full pick sequence through MoveIt."
    )
    parser.add_argument("--scene", required=True)
    parser.add_argument("--object", required=True)
    parser.add_argument("--server-url", default=grasp_client.DEFAULT_SERVER_URL)
    parser.add_argument("--pregrasp-offset", type=float,
                         default=grasp_planner.DEFAULT_PREGRASP_OFFSET)
    parser.add_argument("--retreat-offset", type=float,
                         default=DEFAULT_RETREAT_OFFSET)
    parser.add_argument("--object-name", default=None,
                         help="planning-scene object name to attach; "
                              "omit to skip the attach step")
    parser.add_argument("--teleport", action="store_true", default=True,
                         help="Enable mobile base relocation and arm base teleportation")
    parser.add_argument("--no-teleport", dest="teleport", action="store_false",
                         help="Disable mobile base relocation (fixed base mode)")
    parser.add_argument("--standoff-dist", type=float,
                         default=base_teleport.DEFAULT_STANDOFF_DISTANCE,
                         help="Horizontal distance from base to grasp center (m)")
    parser.add_argument("--base-topic", type=str,
                         default=base_teleport.DEFAULT_GO1_TOPIC,
                         help="ROS topic for Unitree Go1 displacement command")
    parser.add_argument("--teleport-only", action="store_true", default=False,
                         help="Only compute base placement, publish command and teleport without executing arm motion")
    parser.add_argument("--level-only", action="store_true", default=False,
                         help="Only accept a grasp whose approach is parallel to the "
                              "ground (see grasp_planner.select_level_grasp) - fetches "
                              "--top-k candidates and picks the highest-scoring level "
                              "one, instead of AnyGrasp's single best-scoring grasp "
                              "regardless of tilt")
    parser.add_argument("--max-tilt-deg", type=float,
                         default=grasp_planner.DEFAULT_MAX_TILT_DEG,
                         help="Max degrees off level a grasp's approach may be to "
                              "still count as level, used with --level-only")
    parser.add_argument("--top-k", type=int, default=5,
                         help="How many candidates to fetch from AnyGrasp when "
                              "--level-only is set")
    parser.add_argument("--no-visualize", dest="visualize",
                         action="store_false", default=True,
                         help="Don't broadcast grasp_pose/retreat_pose as TF "
                              "frames for RViz (Add -> TF, or Add -> Axes with "
                              "that frame name) - on by default, added "
                              "2026-09-17 so you can see exactly what a run "
                              "computed and acted on")
    parser.add_argument("--hold-viz-sec", type=float, default=30.0,
                         help="Seconds to keep broadcasting grasp_pose/"
                              "retreat_pose after the pick sequence ends "
                              "(success or failure), so there's time to look "
                              "in RViz before the process exits")
    args = parser.parse_args(rospy.myargv(sys.argv[1:]))

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("executor_test", anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)

    teleporter = None
    if args.teleport:
        teleporter = base_teleport.BaseTeleporter(rate_hz=20.0)
        teleporter.start()

    pose_broadcaster = None
    if args.visualize:
        pose_broadcaster = tf_utils.PoseBroadcaster(rate_hz=10.0)
        pose_broadcaster.start()

    arm = moveit_commander.MoveGroupCommander(ARM_GROUP)
    hand = moveit_commander.MoveGroupCommander(HAND_GROUP)
    arm.set_end_effector_link("end_effector_link")

    # CRITICAL (found 2026-09-17): every target below is computed in
    # tf_utils.DEFAULT_BASE_FRAME ("link1"), but set_position_target()
    # interprets raw coordinates in the group's *pose reference frame*,
    # which defaults to the planning frame - "world". Without this line,
    # link1-frame numbers were being executed as world-frame coordinates,
    # so the gripper landed off by exactly the world->link1 base-teleport
    # transform. That's why the grasp_pose TF marker looked right in RViz
    # (it's a properly-tagged PoseStamped) while the arm still went
    # somewhere else. Measured: 79.6mm error before, 1.5mm after.
    arm.set_pose_reference_frame(tf_utils.DEFAULT_BASE_FRAME)
    print("End effector link:", arm.get_end_effector_link())
    print("Pose reference frame:", arm.get_pose_reference_frame())
    if args.level_only:
        grasps = grasp_client.get_grasps(
            args.scene, args.object, server_url=args.server_url, top_k=args.top_k
        )
        grasp = grasp_planner.select_level_grasp(
            tf_buffer, grasps, max_tilt_deg=args.max_tilt_deg
        )
        if grasp is None:
            rospy.logerr(
                "No grasp within %.1f deg of level found among top %d "
                "candidates - try a larger --top-k or --max-tilt-deg.",
                args.max_tilt_deg, len(grasps),
            )
            if teleporter:
                teleporter.stop()
            raise SystemExit(1)
    else:
        grasp = grasp_client.get_best_grasp(
            args.scene, args.object, server_url=args.server_url
        )

    if grasp is None:
        rospy.logerr("No grasp returned, nothing to execute.")
        if teleporter:
            teleporter.stop()
        raise SystemExit(1)

    if args.teleport_only:
        rospy.loginfo("Teleport-only mode: computing base placement and publishing...")
        pregrasp_pose, grasp_pose, base_info = grasp_planner.plan_grasp_poses(
            tf_buffer,
            grasp,
            pregrasp_offset=args.pregrasp_offset,
            teleport_base=True,
            standoff_distance=args.standoff_dist,
            base_topic=args.base_topic,
            teleporter=teleporter,
            return_base_info=True,
        )
        if pose_broadcaster is not None:
            pose_broadcaster.set_pose("grasp_pose", grasp_pose)
        rospy.loginfo("Teleportation complete. Waiting for mobile base execution.")
        if pose_broadcaster and args.hold_viz_sec > 0:
            rospy.loginfo(
                "Holding grasp_pose TF broadcast for %.0fs - add it in RViz "
                "now (Add -> TF, or Add -> Axes) to see where it landed.",
                args.hold_viz_sec,
            )
            rospy.sleep(args.hold_viz_sec)
        else:
            rospy.sleep(1.0)
        if teleporter:
            teleporter.stop()
        if pose_broadcaster:
            pose_broadcaster.stop()
        moveit_commander.roscpp_shutdown()
        sys.exit(0)

    pick_failed = False
    try:
        execute_pick(
            arm,
            hand,
            tf_buffer,
            grasp,
            object_name=args.object_name,
            pregrasp_offset=args.pregrasp_offset,
            retreat_offset=args.retreat_offset,
            standoff_distance=args.standoff_dist,
            base_topic=args.base_topic,
            teleport_base=args.teleport,
            teleporter=teleporter,
            pose_broadcaster=pose_broadcaster,
        )
        rospy.loginfo("Pick sequence completed.")
    except PickExecutionError as exc:
        rospy.logerr("Pick sequence failed: %s", exc)
        pick_failed = True
    finally:
        if teleporter:
            teleporter.stop()
        if pose_broadcaster and args.hold_viz_sec > 0:
            rospy.loginfo(
                "Holding grasp_pose/retreat_pose TF broadcast for %.0fs - "
                "add them in RViz now (Add -> TF, or Add -> Axes with the "
                "frame name) to see what this run computed.",
                args.hold_viz_sec,
            )
            rospy.sleep(args.hold_viz_sec)
        if pose_broadcaster:
            pose_broadcaster.stop()

    moveit_commander.roscpp_shutdown()
    if pick_failed:
        raise SystemExit(1)
