"""
Isaac Sim-side listener for base_teleport.py's absolute teleport topic.

RUN THIS INSIDE ISAAC SIM, NOT via docker exec / ros_noetic:
  - Paste into Isaac Sim's Script Editor (Window -> Script Editor) and run
    it once while your scene is open and playing.
  - This is a different execution context than the grasp_pipeline scripts
    in this same directory, which run inside the ros_noetic container. This
    file only makes sense running inside Isaac Sim's own Python (the Kit
    process), which is why it isn't wired into any roslaunch file.

Why this exists (2026-09-16, BUG-17 investigation - see AGENT_SESSION.md):
base_teleport.py's BaseTeleporter broadcasts a world -> link1 TF, but that
broadcast does NOT affect what MoveIt actually plans against -
grasp_to_base_pose() computes the grasp pose relative to link1 via the
arm's own physical kinematic chain (camera -> link5 -> ... -> link1), which
never routes through "world" at all. A TF-only teleport is cosmetic. For a
base placement to make an out-of-reach grasp reachable, the robot's root
has to actually move in the simulation - that's what this script does, by
listening for the absolute (x, y, yaw) grasp_planner.py already publishes
via base_teleport.publish_absolute_pose() and applying it to the robot.

BEFORE RUNNING - fill in / verify these two things, they cannot be guessed
from outside Isaac Sim:

  ARTICULATION_ROOT_PATH: the stage path of the robot's articulation root
  prim (in this stage: the top-level "open_manipulator_x" prim - the one
  the Isaac URDF importer tagged with PhysicsArticulationRootAPI, NOT
  "root_joint" and NOT the "world" Xform under it). Find it in the Stage
  tree: select it, right-click -> Copy Prim Path.

  Z_HEIGHT: the robot's current Z position in the stage (this script only
  moves it in X/Y and yaw, matching a 2D mobile base - it does not touch Z).

HISTORY - two earlier approaches tried and ruled out by live testing, not
assumption (see AGENT_SESSION.md BUG-17 for the full account each time):

  v1: edited the "world" Xform prim's Translate/Rotate directly
  (UsdGeom.XformCommonAPI). Not tried live - ruled out ahead of time once
  the stage tree showed "world" is rigidly anchored by a PhysicsFixedJoint
  ("root_joint", Body0 unset = global frame, Body1 = the "world" Xform):
  PhysX re-asserts a joint's constraint every physics step, so a raw Xform
  edit during Play would very likely get overridden immediately.

  v2: edited "root_joint"'s own local frame offset (localPos0/localRot0)
  instead, reasoning that since Body0 is unset, that offset IS the anchor
  point PhysX enforces. Tried live: the edit executed with no error and
  printed the expected confirmation, but the robot did NOT move. Root
  cause: localPos0/localRot0 are authored USD data, but PhysX builds its
  live constraint state from the stage once (effectively treating a
  fixed-to-world body as a static anchor) - it does not hot-reload raw
  joint attribute edits made to an already-running simulation. Editing the
  authoring data doesn't retroactively move the already-built physics
  object.

  v3 (this version): uses Isaac Sim's high-level
  omni.isaac.core.articulations.Articulation.set_world_pose() instead of
  editing any USD attribute directly. This is the API Isaac Sim provides
  specifically for repositioning an articulated robot's root while
  simulating - it correctly propagates into the live PhysX articulation
  state (including satisfying whatever fixed-to-world joint anchors the
  root), unlike raw USD edits which only take effect at stage
  parse/(re)initialization time. Not yet verified live - try this next.
"""

import threading

import numpy as np
import rospy
from geometry_msgs.msg import Pose2D

from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.rotations import euler_angles_to_quat

# --- fill these in for your stage before running ---
ARTICULATION_ROOT_PATH = "/open_manipulator_x"  # VERIFY - see docstring above
Z_HEIGHT = 0.0  # VERIFY - the robot's current Z in the stage
TOPIC = "/isaac/base_teleport_absolute"
# ----------------------------------------------------

_lock = threading.Lock()
_pending = None  # (x, y, yaw_rad) or None
_articulation = None


def _get_articulation():
    global _articulation
    if _articulation is None:
        _articulation = Articulation(prim_path=ARTICULATION_ROOT_PATH)
        _articulation.initialize()
    return _articulation


def _on_teleport_msg(msg):
    global _pending
    with _lock:
        _pending = (msg.x, msg.y, msg.theta)


def _apply_pending():
    """Runs on Kit's main thread via the app update loop - do not touch
    the simulation/articulation directly from the rospy callback's
    thread."""
    global _pending
    with _lock:
        pose = _pending
        _pending = None
    if pose is None:
        return

    x, y, yaw_rad = pose

    try:
        art = _get_articulation()
    except Exception as exc:
        print(f"[isaac_sim_teleport_listener] ERROR: could not initialize "
              f"Articulation at '{ARTICULATION_ROOT_PATH}' - fix "
              f"ARTICULATION_ROOT_PATH at the top of this script and "
              f"re-run. ({exc})")
        return

    # Isaac Core expects orientation as a (w, x, y, z) quaternion, not
    # ROS's (x, y, z, w) - euler_angles_to_quat handles that convention
    # for us from a plain (roll, pitch, yaw) triple.
    quat_wxyz = euler_angles_to_quat(np.array([0.0, 0.0, yaw_rad]))

    art.set_world_pose(
        position=np.array([x, y, Z_HEIGHT]),
        orientation=quat_wxyz,
    )

    print(f"[isaac_sim_teleport_listener] Set articulation "
          f"'{ARTICULATION_ROOT_PATH}' world pose to x={x:.3f} y={y:.3f} "
          f"z={Z_HEIGHT:.3f} yaw={yaw_rad:.3f}rad")


def start():
    if not rospy.core.is_initialized():
        rospy.init_node("isaac_sim_teleport_listener", anonymous=True, disable_signals=True)

    global _sub, _update_sub
    _sub = rospy.Subscriber(TOPIC, Pose2D, _on_teleport_msg, queue_size=1)

    import omni.kit.app
    _update_sub = (
        omni.kit.app.get_app().get_update_event_stream()
        .create_subscription_to_pop(lambda e: _apply_pending())
    )
    print(f"[isaac_sim_teleport_listener] Listening on '{TOPIC}', "
          f"applying to articulation '{ARTICULATION_ROOT_PATH}'. Run "
          f"stop() to tear down before re-running this script, or you'll "
          f"get duplicate subscribers.")


def stop():
    global _sub, _update_sub, _articulation
    try:
        _sub.unregister()
    except Exception:
        pass
    _update_sub = None
    _articulation = None
    print("[isaac_sim_teleport_listener] Stopped.")


start()
