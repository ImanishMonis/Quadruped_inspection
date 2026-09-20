#!/usr/bin/env python3
"""
Multi-view point-cloud capture: left / right / top / close. Left and
right relocate the arm's own base (this project's quadruped-mounted-arm
base-relocation machinery -- base_teleport.py + isaac_sim_teleport_
listener.py, already used by executor.py's --teleport flow); top and
close are PURE ARM MOTION with no base relocation at all, per explicit
request, since neither needs quadruped-style repositioning.

Why base teleport instead of pure arm IK: an earlier version of this
script tried to reach different viewpoints by moving only the end
effector from one fixed base. That failed hard -- the tracked object was
already ~0.27m from the base (near the ~0.32m safe max reach, see
base_teleport.DEFAULT_SAFE_MAX_REACH), so any offset away from it for a
"different angle" pushed past max reach and MoveIt timed out on every
attempt. Relocating the BASE first (mirroring how the real quadruped
would walk to a new spot) and only then reaching the arm out to the
object from that new, close-by base position keeps every reach within
the arm's own well-tested envelope, exactly like grasp_planner.py's
existing plan_grasp_with_base_relocation does for a real grasp.

Per explicit request, the base's POSITION after teleporting is the arm's
own baseline position shifted sideways by a fixed lateral_offset -- NOT
re-derived from a standoff-to-object formula (an earlier version placed
the base at a computed standoff distance from the object instead). Only
the resulting YAW (how much the arm should rotate after being teleported
so it still faces the object) is calculated from real data: the object's
own segmented depth points (the currently-tracked mask's median 3D
position, via get_target_world_point -- see build_side_base_pose).

Sequence, per view ("left"/"right" teleport the base; "top"/"close" do not):

  1. Establish a known baseline base pose (0, 0, 0) -- both physically
     (base_teleport.publish_absolute_pose, consumed by
     isaac_sim_teleport_listener.py running inside Isaac Sim) and as the
     TF anchor (BaseTeleporter's world -> link1 broadcast). Needed
     because "world" only exists in this project's TF tree while a
     BaseTeleporter is broadcasting it (see tf_utils.py) -- without an
     explicit reset there is no guarantee the robot wasn't left
     somewhere else by a previous run.
  2. Get the tracked object's position once, in "world" frame (the fixed
     anchor -- NOT link1, which moves every time the base teleports).
  3. Shift the base sideways by lateral_offset from that baseline
     position, compute the yaw needed to face the (real, segmented)
     target from there, teleport there (real + TF, kept in sync exactly
     like plan_grasp_with_base_relocation does), then re-transform the
     frozen world-frame target into the NEW link1 frame and move the end
     effector to a point --view-standoff short of it HORIZONTALLY (see
     standoff_point) -- NOT the object's own coordinates, which drove
     the gripper right up against it (looked exactly like a grasp
     approach, even with no gripper/attach logic anywhere in this
     script). Height is floored at --min-view-height, not scaled down
     with the horizontal pull-back -- an earlier version scaled the
     whole 3D vector toward the base origin (z=0, ~floor height on this
     robot), which collided the end effector with the floor for a
     low-sitting object as a side effect of the horizontal pull-back.
  4. For "top": after step 3's reach (no NEW teleport -- back at the
     baseline base pose), raise joint2/joint3 by --top-raise-step while
     subtracting 2x that from joint4, which keeps joint2+joint3+joint4
     (the end effector's absolute pitch, since all three rotate about
     the same un-rotated local Y axis) constant, so the camera keeps
     facing the object while the arm extends upward.
     For "close": same baseline base pose, a smaller --close-standoff
     than the default view (nearer the object), still floor-floored.
  5. Ask the GUI to reseed tracking on the known target and capture
     (isaac_sim_native_gui.py's "reseed_and_capture" -- a big camera
     jump like this is far past what SAM2's 2-frame tracker can follow
     on its own, see sam2_service/AGENT.md).
  6. Return the arm to a forward-facing home pose (joint1=0, reusing the
     SRDF "Rest" state's joint2-4 -- NOT "stand_up", which pitches the
     camera up toward the ceiling) before the next view, per explicit
     request.

After all views: asks the GUI to fuse the session into one .pcd via
sam2_service's ground-truth-pose path (poses come straight from Isaac's
own USD camera transform, so no ICP is needed).

Also this arm is 4-DOF with position_only_ik (see tf_utils.py's module
docstring) -- there is still no independent control over which way the
camera points, only where the end effector is, even after relocating the
base. A candidate that doesn't actually see the object is rejected by
the GUI's own reseed/valid-depth checks, not silently trusted.

Prerequisite: isaac_sim_native_gui.py must already be running with the
target object clicked/selected (tracking active) before this script
starts. isaac_sim_teleport_listener.py must also already be running
inside Isaac Sim (pasted into its Script Editor separately -- it cannot
be started from here, see that file's own docstring).
"""

import argparse
import math
import sys

import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped transforms)
import zmq
from geometry_msgs.msg import PoseStamped

import moveit_commander

import base_teleport
import tf_utils


ARM_GROUP = "x_arm"

# joint1=0 faces the workspace on this robot (a target near the front
# measures close to 0 rad in link1 frame). The other three joints are
# read live from the SRDF's "stand_up" state (moderately raised,
# known-safe) rather than duplicated here.
HOME_JOINT1 = 0.0
# HOME_SOURCE_STATE = "stand_up"
HOME_SOURCE_STATE = "Rest"

DEFAULT_GUI_ADDR = "tcp://localhost:5556"
DEFAULT_LATERAL_OFFSET = 0.20   # m, fixed sideways shift from the arm's own baseline position
DEFAULT_VIEW_STANDOFF = 0.15    # m, how far short of the object's own position the end
                                # effector stops -- this is a VIEW, never a grasp approach
DEFAULT_MIN_VIEW_HEIGHT = 0.06  # m, floor clearance -- the end effector's own geometry
                                # (fingers, mount) extends below its commanded point, so
                                # this must stay above z=0 (floor/table height), not just >= 0
DEFAULT_TOP_RAISE_STEP = 0.20   # rad, added to joint2/joint3 (and doubled-subtracted from
                                # joint4) to raise the arm for the top view while keeping
                                # it facing the object
DEFAULT_CLOSE_STANDOFF = 0.08   # m, a smaller standoff than the default view for a closer look
# After the arm stops, before capturing. Covers BOTH physics settling and
# RTX Real-Time's temporal accumulation: while the arm-mounted camera is
# moving, the renderer keeps resetting its sample accumulation, so the feed
# looks noisy/grainy (most visibly on fine repeating textures like the
# FlatGrid floor) and only cleans up once motion stops. Confirmed 2026-09-18
# to be inherent renderer behaviour, not a bug in this pipeline: it happens
# identically when the arm is driven from RViz with none of these scripts
# running, and appears in the raw /camera/color/image_raw ROS topic too.
SETTLE_SEC = 3.0
TELEPORT_SETTLE_SEC = 0.5   # after a base teleport, before trusting link1's new TF


def _gui_request(ctx, addr, payload, timeout_ms=5000):
    """One REQ/REP round-trip to the Isaac Sim GUI's external command
    port. A fresh socket per call avoids the REQ send/recv state machine
    getting stuck if a previous call timed out."""
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(addr)
    sock.send_json(payload)
    if not sock.poll(timeout_ms):
        sock.close()
        raise RuntimeError(
            f"no reply from Isaac Sim GUI at {addr} within {timeout_ms} ms "
            "(is isaac_sim_native_gui.py running with an object selected?)"
        )
    resp = sock.recv_json()
    sock.close()
    return resp


def _pose_point(frame_id, xyz):
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.header.stamp = rospy.Time(0)
    p.pose.position.x, p.pose.position.y, p.pose.position.z = (float(v) for v in xyz)
    p.pose.orientation.w = 1.0
    return p


def get_target_world_point(ctx, gui_addr, tf_buffer, camera_frame, world_frame):
    """Ask the GUI for the tracked object's position in its own camera
    optical frame (this call also latches it as the GUI's own re-seed
    anchor for later reseed_and_capture calls), then transform through
    tf2 into world_frame -- fixed regardless of later base teleports,
    unlike link1."""
    resp = _gui_request(ctx, gui_addr, {"cmd": "get_target_point"})
    if not resp.get("ok"):
        raise RuntimeError(f"GUI has no valid tracked target: {resp.get('error')}")
    cx, cy, cz = resp["cam_point_opencv_optical"]
    cam_pose = _pose_point(camera_frame, (cx, cy, cz))
    world_pose = tf_utils.transform_pose(tf_buffer, cam_pose, world_frame)
    p = world_pose.pose.position
    return np.array([p.x, p.y, p.z])


def build_side_base_pose(target_world, base0, side, lateral_offset, safe_max_reach):
    """Base pose (x, y, yaw): position is the arm's own baseline position
    (base0), shifted sideways by a fixed lateral_offset -- NOT re-derived
    from a standoff-to-object formula (an earlier version placed the base
    at a computed standoff distance from the object; per explicit
    request, the position should instead be "with respect to the arm's
    correct position", i.e. a plain lateral shift from wherever it
    already is).

    Yaw is then computed purely from where the object actually is
    (target_world -- itself the median of the currently-tracked mask's
    own segmented depth points, unprojected and transformed to world
    frame; see get_target_world_point), so the camera still faces the
    real object from the new position, per explicit request that the
    needed rotation come from the object's own (partial) point cloud
    data rather than a geometric formula.

    Shifting sideways while holding the "depth" (base0 x) fixed can push
    the resulting distance-to-target past safe_max_reach (Pythagorean:
    sqrt(depth**2 + lateral_offset**2) > depth) -- found empirically for
    an object already sitting at ~0.32m depth (near the whole safe
    envelope), where even a modest lateral shift went out of reach and
    every capture failed. So lateral_offset is auto-shrunk (with a
    warning) to the largest value that keeps the base-to-target distance
    at or under safe_max_reach, computed exactly (not the depth-only
    approximation above) via the two points where a circle of radius
    safe_max_reach centered on the target crosses the line "shifted
    sideways from base0", to preserve the actual, slightly asymmetric
    yaw offset an off-center target already has toward one side over
    the other."""
    assert side in ("left", "right")
    sign = 1.0 if side == "left" else -1.0
    # Sign convention (which physical side is "left") is not verified
    # analytically here -- if left/right come out swapped in Isaac Sim,
    # flip this sign.
    dx = target_world[0] - base0[0]
    g = target_world[1] - base0[1]   # existing lateral gap between base0 and the target
    max_lat = math.sqrt(max(safe_max_reach ** 2 - dx ** 2, 0.0))
    offset_upper = (g + max_lat) if sign > 0 else (max_lat - g)
    offset = min(lateral_offset, max(0.0, offset_upper))
    if offset < lateral_offset - 1e-6:
        rospy.logwarn(
            "[%s] requested lateral_offset %.3fm would exceed safe_max_reach "
            "%.3fm at this object's depth (%.3fm) -- shrunk to %.3fm",
            side, lateral_offset, safe_max_reach, dx, offset,
        )

    x_base = base0[0]
    y_base = base0[1] + sign * offset
    yaw = math.atan2(target_world[1] - y_base, target_world[0] - x_base)
    dist = math.hypot(target_world[0] - x_base, target_world[1] - y_base)
    return float(x_base), float(y_base), float(yaw), float(dist)


def teleport_base(teleporter, x, y, yaw, base_topic):
    # Same pairing grasp_planner.plan_grasp_with_base_relocation uses:
    # the absolute-pose publish is what isaac_sim_teleport_listener.py
    # actually applies to the robot prim in Isaac Sim; the TF broadcast
    # is a separate, cosmetic mirror kept numerically in sync so later
    # tf2 lookups of link1 reflect it too.
    base_teleport.publish_absolute_pose(x, y, yaw)
    teleporter.set_pose(x, y, yaw)
    rospy.sleep(TELEPORT_SETTLE_SEC)


def standoff_point(point_link1, standoff, min_height):
    """Point `standoff` meters back from point_link1 HORIZONTALLY (X/Y
    only) toward the arm's own base (link1 origin) -- so the end effector
    (and the camera mounted near it) stops at a safe viewing distance
    instead of reaching all the way to the object's own coordinates.
    Height (Z) is handled separately, floored at min_height, and is NEVER
    scaled down along with the horizontal pull-back.

    Found empirically, in two stages:
    - commanding the end effector straight to the object's exact position
      drove the gripper right up against it -- looked exactly like a
      grasp approach even though nothing here closes the gripper or
      attaches anything, this script only ever wants a view.
    - pulling the point back along the full 3D line to the base ORIGIN
      (not just horizontally) collided the end effector with the floor:
      the base origin sits at z=0 (about floor height on this robot), so
      scaling the whole vector toward it also dragged a low object's
      height further toward 0 as a side effect of the horizontal pull-
      back -- two unrelated things coupled by one scale factor."""
    p = np.asarray(point_link1, float)
    horiz = p[:2]
    dist = float(np.linalg.norm(horiz))
    if dist < 1e-6:
        pulled = horiz
    else:
        direction = horiz / dist
        pulled_back = max(dist - standoff, 0.05)  # never collapse onto the base itself
        pulled = direction * pulled_back
    z = max(float(p[2]), min_height)
    return np.array([pulled[0], pulled[1], z])


def move_to_link1_point(arm, point_link1):
    arm.set_position_target(point_link1.tolist() if hasattr(point_link1, "tolist") else list(point_link1))
    ok = arm.go(wait=True)
    arm.stop()
    arm.clear_pose_targets()
    return ok


def _return_home(arm, robot):
    """Forward-facing, moderately-raised home pose -- see HOME_JOINT1's
    comment for why this is NOT the SRDF "Rest" state.

    Clamped against LIVE joint bounds (robot.get_joint(name).bounds()),
    not trusted as-is: found empirically that the SRDF's "stand_up" state
    (joint3=-1.5) predates the 0.05 rad safety margins added to the URDF
    in an earlier session (joint3's lower bound is now -1.45) and was
    never reconciled, so set_joint_value_target() rejected it outright."""
    joints = arm.get_named_target_values(HOME_SOURCE_STATE)
    joints["joint1"] = HOME_JOINT1
    for name, value in list(joints.items()):
        joint = robot.get_joint(name)
        lo, hi = joint.min_bound(), joint.max_bound()
        if value < lo or value > hi:
            rospy.logwarn(
                "home pose joint '%s'=%.4f outside live bounds [%.4f, %.4f] "
                "(stale SRDF state?) -- clamping", name, value, lo, hi,
            )
            joints[name] = min(max(value, lo), hi)
    arm.set_joint_value_target(joints)
    if not arm.go(wait=True):
        rospy.logwarn("Failed to return to home pose")
    arm.stop()
    arm.clear_pose_targets()


def run(args):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("multi_view_capture", anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)

    ctx = zmq.Context.instance()

    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander(ARM_GROUP)
    arm.set_end_effector_link("end_effector_link")
    # Same trap as executor.py: raw position_target coordinates are
    # interpreted in the group's pose reference frame, which defaults to
    # "world" (MoveIt's planning frame), not the frame these numbers are
    # computed in.
    arm.set_pose_reference_frame(tf_utils.DEFAULT_BASE_FRAME)
    arm.set_planning_time(args.planning_time)
    arm.set_num_planning_attempts(args.planning_attempts)

    teleporter = base_teleport.BaseTeleporter()
    teleporter.start()

    base0 = (0.0, 0.0, 0.0)
    rospy.loginfo("Resetting to baseline base pose (0, 0, 0)...")
    teleport_base(teleporter, *base0, base_topic=args.base_topic)
    rospy.sleep(0.5)  # let the reset actually land before measuring the target from it

    target_world = get_target_world_point(
        ctx, args.gui_addr, tf_buffer, args.camera_frame, tf_utils.DEFAULT_WORLD_FRAME
    )
    rospy.loginfo(
        "Target in %s frame: (%.3f, %.3f, %.3f)",
        tf_utils.DEFAULT_WORLD_FRAME, target_world[0], target_world[1], target_world[2],
    )

    captured = 0

    for side in ("left", "right"):
        x, y, yaw, dist = build_side_base_pose(
            target_world, base0[:2], side, args.lateral_offset, args.safe_max_reach,
        )
        rospy.loginfo(
            "[%s] teleporting base to (x=%.3f, y=%.3f, yaw=%.1f deg), %.3f m to target",
            side, x, y, math.degrees(yaw), dist,
        )
        teleport_base(teleporter, x, y, yaw, args.base_topic)

        target_link1 = tf_utils.transform_pose(
            tf_buffer, _pose_point(tf_utils.DEFAULT_WORLD_FRAME, target_world),
            tf_utils.DEFAULT_BASE_FRAME,
        )
        p = target_link1.pose.position
        point_link1 = np.array([p.x, p.y, p.z])
        view_point = standoff_point(point_link1, args.view_standoff, args.min_view_height)
        rospy.loginfo(
            "[%s] object at link1 (%.3f, %.3f, %.3f); reaching to view point (%.3f, %.3f, %.3f) "
            "(%.3fm standoff)", side, point_link1[0], point_link1[1], point_link1[2],
            view_point[0], view_point[1], view_point[2], args.view_standoff,
        )

        if not move_to_link1_point(arm, view_point):
            rospy.logwarn("[%s] IK/motion failed, skipping", side)
            _return_home(arm, robot)
            continue

        rospy.sleep(SETTLE_SEC)
        try:
            resp = _gui_request(ctx, args.gui_addr, {"cmd": "reseed_and_capture"})
        except RuntimeError as exc:
            rospy.logerr("[%s] %s", side, exc)
            _return_home(arm, robot)
            continue

        if resp.get("ok"):
            captured += 1
            rospy.loginfo("[%s] captured (pose_count=%s)", side, resp.get("pose_count"))
        else:
            rospy.logwarn("[%s] rejected: %s", side, resp.get("status_text") or resp.get("error"))

        _return_home(arm, robot)

    # Top-down and closer views: PURE ARM MOTION from the baseline base
    # pose, no teleport -- per explicit request, since neither of these
    # needs quadruped-style relocation.
    rospy.loginfo("[top/close] returning to baseline base pose...")
    teleport_base(teleporter, *base0, base_topic=args.base_topic)

    target_link1 = tf_utils.transform_pose(
        tf_buffer, _pose_point(tf_utils.DEFAULT_WORLD_FRAME, target_world),
        tf_utils.DEFAULT_BASE_FRAME,
    )
    p = target_link1.pose.position
    point_link1 = np.array([p.x, p.y, p.z])
    center_view = standoff_point(point_link1, args.view_standoff, args.min_view_height)
    rospy.loginfo(
        "[top] center reach to view point (%.3f, %.3f, %.3f) before raising", *center_view
    )

    if not move_to_link1_point(arm, center_view):
        rospy.logwarn("[top] center reach failed, skipping top view")
        _return_home(arm, robot)
    else:
        rospy.sleep(SETTLE_SEC)
        # joint2, joint3, joint4 all rotate about the same local Y axis
        # with zero rotation between their mounting origins (verified in
        # open_manipulator_x_arm.urdf.xacro), so the end effector's
        # absolute pitch is exactly joint2+joint3+joint4. Raising
        # joint2/joint3 and subtracting the same total from joint4 keeps
        # that sum constant, so the camera keeps facing the object
        # instead of tipping away while the arm extends upward.
        baseline_joints = arm.get_current_joint_values()  # [joint1, joint2, joint3, joint4]
        step = args.top_raise_step
        # Sign is a reasoned guess, not yet verified empirically: "Rest"
        # (folded, joint2=-1.5) -> "stand_up" (more extended, joint2=
        # -0.014) raises the arm by INCREASING joint2, so +step is used
        # here too. If this instead lowers the arm in Isaac Sim, flip
        # the sign of `step` (still -2*step on joint4 to compensate).
        top_joints = [
            baseline_joints[0],
            baseline_joints[1] + step,
            baseline_joints[2] + step,
            baseline_joints[3] - 2.0 * step,
        ]
        for i, name in enumerate(["joint1", "joint2", "joint3", "joint4"]):
            joint = robot.get_joint(name)
            lo, hi = joint.min_bound(), joint.max_bound()
            if top_joints[i] < lo or top_joints[i] > hi:
                rospy.logwarn(
                    "[top] joint '%s'=%.4f outside bounds [%.4f, %.4f] -- clamping",
                    name, top_joints[i], lo, hi,
                )
                top_joints[i] = min(max(top_joints[i], lo), hi)

        arm.set_joint_value_target(top_joints)
        ok = arm.go(wait=True)
        arm.stop()
        arm.clear_pose_targets()

        if not ok:
            rospy.logwarn("[top] raise move failed, skipping")
            _return_home(arm, robot)
        else:
            rospy.sleep(SETTLE_SEC)
            try:
                resp = _gui_request(ctx, args.gui_addr, {"cmd": "reseed_and_capture"})
                if resp.get("ok"):
                    captured += 1
                    rospy.loginfo("[top] captured (pose_count=%s)", resp.get("pose_count"))
                else:
                    rospy.logwarn("[top] rejected: %s", resp.get("status_text") or resp.get("error"))
            except RuntimeError as exc:
                rospy.logerr("[top] %s", exc)
            _return_home(arm, robot)

    # Closer view: same baseline base pose, smaller standoff (nearer the
    # object) -- also pure arm motion, no teleport.
    target_link1 = tf_utils.transform_pose(
        tf_buffer, _pose_point(tf_utils.DEFAULT_WORLD_FRAME, target_world),
        tf_utils.DEFAULT_BASE_FRAME,
    )
    p = target_link1.pose.position
    point_link1 = np.array([p.x, p.y, p.z])
    close_view = standoff_point(point_link1, args.close_standoff, args.min_view_height)
    rospy.loginfo(
        "[close] reaching to view point (%.3f, %.3f, %.3f) (%.3fm standoff)",
        close_view[0], close_view[1], close_view[2], args.close_standoff,
    )

    if not move_to_link1_point(arm, close_view):
        rospy.logwarn("[close] IK/motion failed, skipping")
        _return_home(arm, robot)
    else:
        rospy.sleep(SETTLE_SEC)
        try:
            resp = _gui_request(ctx, args.gui_addr, {"cmd": "reseed_and_capture"})
            if resp.get("ok"):
                captured += 1
                rospy.loginfo("[close] captured (pose_count=%s)", resp.get("pose_count"))
            else:
                rospy.logwarn("[close] rejected: %s", resp.get("status_text") or resp.get("error"))
        except RuntimeError as exc:
            rospy.logerr("[close] %s", exc)
        _return_home(arm, robot)

    rospy.loginfo("Captured %d/4 views.", captured)

    if captured >= 2:
        rospy.loginfo("Fusing captured views into a point cloud...")
        resp = _gui_request(ctx, args.gui_addr, {"cmd": "generate_pointcloud"}, timeout_ms=60000)
        rospy.loginfo("Fuse result: %s", resp)
    else:
        rospy.logwarn("Fewer than 2 views captured -- skipping fuse (need >=2).")

    teleporter.stop()
    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Capture left/right/top/close views of the object currently "
                    "tracked in isaac_sim_native_gui.py -- left/right shift the "
                    "arm's base sideways (quadruped-mounted-arm architecture) "
                    "and face it based on the object's own segmented depth "
                    "points, top/close are pure arm motion from the baseline "
                    "base pose -- then fuse into one point cloud."
    )
    parser.add_argument("--gui-addr", default=DEFAULT_GUI_ADDR,
                         help="ZeroMQ address of isaac_sim_native_gui.py's "
                              "external command port (COMMAND_BIND in that file)")
    parser.add_argument("--camera-frame", default=tf_utils.DEFAULT_CAMERA_FRAME)
    parser.add_argument("--base-topic", default=base_teleport.DEFAULT_GO1_TOPIC)
    parser.add_argument("--lateral-offset", type=float, default=DEFAULT_LATERAL_OFFSET,
                         help="fixed sideways shift (m) from the arm's own baseline "
                              "position for the left/right views")
    parser.add_argument("--safe-max-reach", type=float,
                         default=base_teleport.DEFAULT_SAFE_MAX_REACH,
                         help="used only to warn if a shifted base ends up farther "
                              "from the target than the arm can reach")
    parser.add_argument("--view-standoff", type=float, default=DEFAULT_VIEW_STANDOFF,
                         help="m short of the object's own position the end effector "
                              "stops at -- this is a view, never a grasp approach")
    parser.add_argument("--min-view-height", type=float, default=DEFAULT_MIN_VIEW_HEIGHT,
                         help="m, floor clearance floor for every reach target")
    parser.add_argument("--top-raise-step", type=float, default=DEFAULT_TOP_RAISE_STEP,
                         help="rad added to joint2/joint3 (doubled-subtracted from joint4) "
                              "to raise the arm for the top view while keeping it facing "
                              "the object")
    parser.add_argument("--close-standoff", type=float, default=DEFAULT_CLOSE_STANDOFF,
                         help="m short of the object's own position for the closer view "
                              "(smaller than --view-standoff)")
    parser.add_argument("--planning-time", type=float, default=10.0)
    parser.add_argument("--planning-attempts", type=int, default=10)
    args = parser.parse_args(rospy.myargv(sys.argv[1:]))

    try:
        run(args)
    except RuntimeError as exc:
        rospy.logerr(str(exc))
        moveit_commander.roscpp_shutdown()
        sys.exit(1)
