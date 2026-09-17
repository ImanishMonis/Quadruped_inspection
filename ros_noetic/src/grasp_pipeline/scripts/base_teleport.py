#!/usr/bin/env python3
"""
Base Placement, Displacement Calculation, and Arm Base Teleportation.

Coordinates mobile base positioning with arm manipulation for a 4-DOF robot
(OpenManipulator-X) mounted on a planar mobile base (Unitree Go1).

Instead of ignoring grasp orientation due to the arm's limited DOFs,
this module calculates where the mobile base should move so that the arm's
reachable sagittal pitch plane aligns exactly with the grasp approach vector.

Key features:
1. compute_base_placement: Computes optimal world base pose and relative
   (dx, dy, dtheta) body displacements from an AnyGrasp 6-DOF pose.
2. publish_teleport_params: Publishes (dx, dy, dtheta) onto ROS 1 topics
   (default: /go1/move_relative and /mobile_base/teleport_params).
3. BaseTeleporter: Dynamic TF broadcaster that teleports the arm's base
   (world -> link1) in simulation, allowing MoveIt to plan and execute
   the grasp with full orientation.
"""

import math
import threading
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Pose2D, TransformStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import tf_utils

# Default standoff distance from the arm base (link1) to the grasp target.
# OpenManipulator-X total reach is ~0.38m; 0.22m gives comfortable reach
# well away from singularities and joint pitch limits.
DEFAULT_STANDOFF_DISTANCE = 0.22

# Meters. See compute_base_placement's docstring - these bound the true
# 3D distance to an elevated grasp, not just its horizontal component.
# safe_max_reach is deliberately below the ~0.38m documented max reach:
# a grasp at ~0.37m total distance timed out in practice (near-singular
# configurations), so this leaves real margin rather than hugging the
# documented limit.
DEFAULT_SAFE_MAX_REACH = 0.32
DEFAULT_MIN_STANDOFF = 0.12

# Cache of latched Publishers, keyed by topic name.
#
# Added 2026-09-16 (BUG-17 investigation): both publish functions below
# used to create a brand-new rospy.Publisher on every call, sleep 0.1s,
# then publish once. That 0.1s is a fixed guess at how long the
# publisher-subscriber TCP handshake takes to complete - if a subscriber
# (e.g. isaac_sim_teleport_listener.py, running inside Isaac Sim's own
# process) isn't fully connected within that window, the message is
# silently dropped, since nothing queues messages for a not-yet-connected
# subscriber. This is exactly what happened testing the Isaac Sim listener
# live: the ROS-side math was confirmed correct, but the robot never
# physically moved, with no error anywhere - a classic "publish
# immediately after creating the publisher" race, not a bug in the
# listener itself. Fixed by (a) reusing one Publisher per topic for the
# life of the process instead of recreating it every call, so the
# connection only needs to establish once, and (b) latch=True, so any
# subscriber that connects even after a message was sent still receives
# the last one automatically - removing the race entirely rather than
# just guessing a longer sleep.
_publisher_cache = {}


def _get_latched_publisher(topic, msg_type=Pose2D, queue_size=10):
    if topic not in _publisher_cache:
        _publisher_cache[topic] = rospy.Publisher(
            topic, msg_type, queue_size=queue_size, latch=True
        )
        rospy.sleep(0.1)  # give the first publish on a topic time to advertise
    return _publisher_cache[topic]

# Primary command topic for Unitree Go1 relative locomotion
DEFAULT_GO1_TOPIC = "/go1/move_relative"
# Debug / inspection topic
DEFAULT_DEBUG_TOPIC = "/mobile_base/teleport_params"
# Absolute target_base_world (x, y, yaw) for a sim-side listener to consume
# directly - see 2026-09-16 note below on why this exists separately from
# the relative-displacement topics above.
DEFAULT_ISAAC_TELEPORT_TOPIC = "/isaac/base_teleport_absolute"


def normalize_angle(angle_rad):
    """Wrap angle to [-pi, pi]."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def compute_base_placement(
    grasp_pose_world,
    standoff_distance=DEFAULT_STANDOFF_DISTANCE,
    current_base_pos=(0.0, 0.0),
    current_base_yaw=0.0,
    safe_max_reach=DEFAULT_SAFE_MAX_REACH,
    min_standoff=DEFAULT_MIN_STANDOFF,
):
    """
    Computes optimal base placement from a 6-DOF grasp pose in the world frame.

    Parameters
    ----------
    grasp_pose_world : geometry_msgs.msg.PoseStamped or geometry_msgs.msg.Pose
        The target grasp pose expressed in the global world frame.
    standoff_distance : float
        Max horizontal distance (meters) between arm base (link1) and grasp
        center - used as-is for grasps near link1's own height, but shrunk
        automatically for elevated grasps (see safe_max_reach below).
    current_base_pos : tuple of (float, float)
        Current (x, y) coordinates of the mobile base in the world frame.
    current_base_yaw : float
        Current heading angle (radians) of the mobile base in the world frame.
    safe_max_reach : float
        Meters. Added 2026-09-17 (found live: a grasp at Z=0.29m with the
        fixed 0.22m standoff gave a *total* 3D distance of ~0.37m from
        link1 - right at the arm's ~0.38m documented max reach - and
        planning timed out. The fixed standoff only ever bounded the
        horizontal (X/Y) distance, never the true 3D distance to an
        elevated grasp. The effective standoff is now
        sqrt(max(safe_max_reach**2 - z_relative**2, min_standoff**2)), so
        it shrinks as height increases, keeping the true 3D distance
        within this reach budget instead of silently growing past it.
        Since base placement only ever changes (x, y, yaw) - never Z (the
        SRDF's virtual_joint is planar, see AGENT_SESSION.md BUG-17) -
        grasp_pose_world's own Z equals the grasp's height relative to
        link1's origin directly, with no separate reference needed.
    min_standoff : float
        Meters. Floor on the shrunk standoff distance, so the base never
        gets uncomfortably close to (or on top of) the object even for
        very high grasps.

    Returns
    -------
    dict with:
        "target_base_world": (x_base, y_base, yaw_base)
        "relative_displacement": (dx_body, dy_body, dyaw_body)
        "approach_vector": np.ndarray [ax, ay, az]
        "is_top_down": bool
        "standoff_distance": float (the actual, possibly-shrunk value used)
    """
    if hasattr(grasp_pose_world, "pose"):
        p = grasp_pose_world.pose.position
        pos = np.array([p.x, p.y, p.z])
        q = grasp_pose_world.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
    else:
        p = grasp_pose_world.position
        pos = np.array([p.x, p.y, p.z])
        q = grasp_pose_world.orientation
        quat = [q.x, q.y, q.z, q.w]

    approach_vec = tf_utils.extract_approach_vector(quat)
    ax, ay, az = approach_vec
    horiz_norm = math.hypot(ax, ay)

    # Check if the grasp is top-down (vertical approach)
    is_top_down = (horiz_norm < 0.1) and (az < -0.5)

    if not is_top_down:
        # Standard side / angled grasp:
        # Approach vector points TOWARD the object from the gripper.
        # The base must sit behind the object along the approach line.
        approach_yaw = math.atan2(ay, ax)
        target_base_yaw = approach_yaw
    else:
        # Top-down grasp: approach is straight down (-Z).
        # Orientation is determined by the gripper finger closing axis (+Y).
        # OpenManipulator-X's fingers close perpendicular to the arm's forward axis.
        # Therefore, the base should be placed perpendicular to the closing axis.
        closing_vec = tf_utils.extract_closing_vector(quat)
        cx, cy, _ = closing_vec
        closing_yaw = math.atan2(cy, cx)

        # Two candidate headings (+/- 90 deg from closing vector)
        cand1 = normalize_angle(closing_yaw + math.pi / 2.0)
        cand2 = normalize_angle(closing_yaw - math.pi / 2.0)

        # Pick candidate closest to current base heading
        diff1 = abs(normalize_angle(cand1 - current_base_yaw))
        diff2 = abs(normalize_angle(cand2 - current_base_yaw))
        target_base_yaw = cand1 if diff1 <= diff2 else cand2

    # Shrink the horizontal standoff for elevated grasps so the true 3D
    # distance from link1 stays within safe_max_reach - see
    # compute_base_placement's docstring. pos[2] is the grasp's height
    # relative to link1's own origin directly: base placement only ever
    # changes (x, y, yaw), never Z.
    z_relative = pos[2]
    effective_standoff = min(
        standoff_distance,
        math.sqrt(max(safe_max_reach ** 2 - z_relative ** 2, min_standoff ** 2)),
    )

    # Target base world position: placed behind grasp along approach heading
    x_base = pos[0] - effective_standoff * math.cos(target_base_yaw)
    y_base = pos[1] - effective_standoff * math.sin(target_base_yaw)

    # Relative displacement in world coordinates
    dx_world = x_base - current_base_pos[0]
    dy_world = y_base - current_base_pos[1]
    dyaw_world = normalize_angle(target_base_yaw - current_base_yaw)

    # Transform displacement into current robot body frame:
    # Forward (+X): along current heading
    # Lateral (+Y): leftward (positive in ROS REP-103)
    cos_curr = math.cos(current_base_yaw)
    sin_curr = math.sin(current_base_yaw)
    dx_body = cos_curr * dx_world + sin_curr * dy_world
    dy_body = -sin_curr * dx_world + cos_curr * dy_world
    dyaw_body = dyaw_world

    rospy.loginfo(
        "[BasePlacement] Grasp world pos: (%.3f, %.3f, %.3f), approach: [%.2f, %.2f, %.2f]",
        pos[0], pos[1], pos[2], ax, ay, az
    )
    if effective_standoff < standoff_distance - 1e-6:
        rospy.loginfo(
            "[BasePlacement] Standoff shrunk %.3fm -> %.3fm for elevated grasp "
            "(z=%.3fm, safe_max_reach=%.3fm) to keep total 3D distance in reach.",
            standoff_distance, effective_standoff, z_relative, safe_max_reach
        )
    rospy.loginfo(
        "[BasePlacement] Target base world: (x=%.3f, y=%.3f, yaw=%.1f deg)",
        x_base, y_base, math.degrees(target_base_yaw)
    )
    rospy.loginfo(
        "[BasePlacement] Relative body command: dx=%.3f m, dy=%.3f m, dyaw=%.1f deg",
        dx_body, dy_body, math.degrees(dyaw_body)
    )

    return {
        "target_base_world": (float(x_base), float(y_base), float(target_base_yaw)),
        "relative_displacement": (float(dx_body), float(dy_body), float(dyaw_body)),
        "approach_vector": approach_vec,
        "is_top_down": is_top_down,
        "standoff_distance": float(effective_standoff),
    }


def publish_teleport_params(
    dx,
    dy,
    dtheta,
    topic=DEFAULT_GO1_TOPIC,
    debug_topic=DEFAULT_DEBUG_TOPIC,
    queue_size=10,
):
    """
    Publishes relative displacement parameters (dx, dy, dtheta) as Pose2D.
    """
    msg = Pose2D()
    msg.x = float(dx)
    msg.y = float(dy)
    msg.theta = float(dtheta)

    pub_main = _get_latched_publisher(topic, queue_size=queue_size)
    pub_main.publish(msg)

    if debug_topic and debug_topic != topic:
        pub_debug = _get_latched_publisher(debug_topic, queue_size=queue_size)
        pub_debug.publish(msg)

    rospy.loginfo(
        "[BaseTeleport] Published Pose2D to '%s': dx=%.3f m, dy=%.3f m, dtheta=%.1f deg",
        topic, dx, dy, math.degrees(dtheta)
    )
    return msg


def publish_absolute_pose(
    x,
    y,
    yaw,
    topic=DEFAULT_ISAAC_TELEPORT_TOPIC,
    queue_size=10,
):
    """
    Publishes the absolute target_base_world pose (x, y, yaw) as Pose2D.

    Added 2026-09-16 (BUG-17 investigation): BaseTeleporter's world -> link1
    TF broadcast does NOT make MoveIt plan against a relocated base - see
    AGENT_SESSION.md. grasp_to_base_pose() computes the grasp pose relative
    to link1 via the arm's own physical kinematic chain (camera -> link5 ->
    ... -> link1), which never routes through "world" at all, so no TF
    broadcast on world -> link1 can affect it. A TF-only "teleport" is
    cosmetic; for a placement to actually change what's reachable, the
    robot's root prim must be moved for real in Isaac Sim. This topic
    carries the absolute pose an Isaac Sim-side listener should apply
    directly to the robot prim's transform - deliberately absolute, not the
    relative (dx, dy, dtheta) above, so it doesn't compound error by
    integrating deltas against whatever Isaac Sim's own idea of "current
    pose" is.
    """
    msg = Pose2D()
    msg.x = float(x)
    msg.y = float(y)
    msg.theta = float(yaw)

    pub = _get_latched_publisher(topic, queue_size=queue_size)
    pub.publish(msg)

    rospy.loginfo(
        "[BaseTeleport] Published absolute target pose to '%s': x=%.3f m, y=%.3f m, yaw=%.1f deg",
        topic, x, y, math.degrees(yaw)
    )
    return msg


class BaseTeleporter:
    """
    Maintains a continuous dynamic TF broadcast for the arm's base (world -> link1)
    so that MoveIt's planning scene immediately reflects the teleported base pose.
    """

    def __init__(self, parent_frame="world", child_frame="link1", rate_hz=20.0):
        self.parent_frame = parent_frame
        self.child_frame = child_frame
        self.rate_hz = rate_hz
        self.broadcaster = tf2_ros.TransformBroadcaster()

        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self._yaw = 0.0

        self._running = False
        self._thread = None
        self._lock = threading.Lock()

    def set_pose(self, x, y, yaw, z=0.0):
        """Update the base pose to broadcast."""
        with self._lock:
            self._x = float(x)
            self._y = float(y)
            self._z = float(z)
            self._yaw = float(yaw)
        self.broadcast_once()

    def get_pose(self):
        with self._lock:
            return self._x, self._y, self._yaw, self._z

    def broadcast_once(self):
        with self._lock:
            x, y, z, yaw = self._x, self._y, self._z, self._yaw

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        quat = quaternion_from_euler(0.0, 0.0, yaw)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.broadcaster.sendTransform(t)

    def _loop(self):
        rate = rospy.Rate(self.rate_hz)
        while self._running and not rospy.is_shutdown():
            self.broadcast_once()
            rate.sleep()

    def start(self):
        """Start continuous broadcasting in a background thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        rospy.loginfo(
            "[BaseTeleporter] Started dynamic TF broadcaster: %s -> %s at %.0f Hz",
            self.parent_frame, self.child_frame, self.rate_hz
        )

    def stop(self):
        """Stop background broadcasting."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

