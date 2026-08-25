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

import rospy
import moveit_commander
import tf2_ros

import grasp_planner
import tf_utils


ARM_GROUP = "x_arm"
HAND_GROUP = "hand"

GRIP_OPEN_STATE = "Grip_Open"
GRIP_CLOSE_STATE = "Grip_Close"

# Meters, pulled back along the approach axis from the grasp pose after
# closing the gripper - deliberately further than the pre-grasp offset
# so the retreat clears whatever the pre-grasp approach swept through.
DEFAULT_RETREAT_OFFSET = -0.03

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
    planning_time=10.0,
    planning_attempts=10,
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
        If None, the attach step is skipped (nothing published the
        object into the planning scene yet - see scene_receiver.py).

    Raises
    ------
    PickExecutionError
        On any step failing. Earlier steps are not undone - the arm is
        left wherever it stopped, deliberately, so the failure is
        visible instead of silently retreating.
    """

    # A short default planning time/attempt count is what leaves a 4-DOF
    # arm timing out on legitimately-reachable-but-tight grasp poses -
    # set generous values regardless of what the caller configured.
    arm.set_planning_time(planning_time)
    arm.set_num_planning_attempts(planning_attempts)

    pregrasp_pose, grasp_pose = grasp_planner.plan_grasp_poses(
        tf_buffer,
        grasp,
        pregrasp_offset=pregrasp_offset,
    )
    print("*" * 15)
    print(grasp_pose)
    print(f"Target X: {grasp_pose.pose.position.x:.4f}")
    print(f"Target Y: {grasp_pose.pose.position.y:.4f}")
    print(f"Target Z: {grasp_pose.pose.position.z:.4f}")
    print(f"Radial Distance: {(grasp_pose.pose.position.x**2 + grasp_pose.pose.position.y**2)**0.5:.4f}")
    print("*" * 15)

    # retreat_pose = tf_utils.offset_along_approach_axis(
    #     grasp_pose,
    #     retreat_offset,
    # )
    # GRIPPER_LENGTH = 0.13 
    # grasp_pose = tf_utils.offset_along_approach_axis(grasp_pose, GRIPPER_LENGTH)
    # pregrasp_pose = tf_utils.offset_along_approach_axis(pregrasp_pose, GRIPPER_LENGTH)

    # 1. Open gripper before moving anywhere near the object.
    hand.set_named_target(GRIP_OPEN_STATE)
    if not hand.go(wait=True):
        raise PickExecutionError("Failed to open gripper")
    hand.stop()

    # 2. Move to the pre-grasp waypoint (free planning, not cartesian -
    #    this leg can be far from the object, cartesian is only needed
    #    for the final straight-line approach in step 3).
    # arm.set_pose_target(pregrasp_pose)
    # if not arm.go(wait=True):
    #     raise PickExecutionError("Failed to reach pre-grasp pose")
    # arm.stop()
    # arm.clear_pose_targets()

    # arm.set_position_target([
    #     pregrasp_pose.pose.position.x, 
    #     pregrasp_pose.pose.position.y, 
    #     pregrasp_pose.pose.position.z
    # ])
    # if not arm.go(wait=True):
    #     raise PickExecutionError("Failed to reach pre-grasp pose")
    # arm.stop()
    # arm.clear_pose_targets()

    # 3. Straight-line cartesian approach from pre-grasp into the grasp.
    # _run_cartesian_path(
    #     arm,
    #     [pregrasp_pose.pose, grasp_pose.pose],
    #     description="grasp approach",
    # )

    arm.set_position_target([
        grasp_pose.pose.position.x, 
        grasp_pose.pose.position.y, 
        grasp_pose.pose.position.z
    ])
    # arm.set_position_target([
    #     0.2632,-0.0355,0.0627
    # ])

    if not arm.go(wait=True):
        raise PickExecutionError("Failed to reach grasp pose")
    arm.stop()
    arm.clear_pose_targets()

    # 4. Close the gripper on the object.
    hand.set_named_target(GRIP_CLOSE_STATE)
    if not hand.go(wait=True):
        raise PickExecutionError("Failed to close gripper")
    hand.stop()

    # 5. Attach to the planning scene, if we have something to attach.
    if object_name:
        arm.attach_object(object_name)
    else:
        rospy.logwarn(
            "No object_name given, skipping attach - the planning "
            "scene has no matching collision object to attach yet."
        )

    # 6. Straight-line retreat.
    # _run_cartesian_path(
    #     arm,
    #     [grasp_pose.pose, retreat_pose.pose],
    #     description="retreat",
    # )
    arm.set_position_target([
        0.138, 
        0.00, 
        0.167
    ])
    if not arm.go(wait=True):
        raise PickExecutionError("Failed to retreat")
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
    args = parser.parse_args(rospy.myargv(sys.argv[1:]))

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("executor_test", anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)

    arm = moveit_commander.MoveGroupCommander(ARM_GROUP)
    hand = moveit_commander.MoveGroupCommander(HAND_GROUP)
    print("Current End Effector Link:", arm.get_end_effector_link())
    arm.set_end_effector_link("end_effector_link")
    print("Current End Effector Link:", arm.get_end_effector_link())
    grasp = grasp_client.get_best_grasp(
        args.scene, args.object, server_url=args.server_url
    )

    if grasp is None:
        rospy.logerr("No grasp returned, nothing to execute.")
        raise SystemExit(1)

    try:
        execute_pick(
            arm,
            hand,
            tf_buffer,
            grasp,
            object_name=args.object_name,
            pregrasp_offset=args.pregrasp_offset,
            retreat_offset=args.retreat_offset,
        )
        rospy.loginfo("Pick sequence completed.")
    except PickExecutionError as exc:
        rospy.logerr("Pick sequence failed: %s", exc)
        raise SystemExit(1)

    moveit_commander.roscpp_shutdown()
