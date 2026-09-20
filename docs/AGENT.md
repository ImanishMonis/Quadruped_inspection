# AGENT.md

# OpenManipulator-X + MoveIt + Isaac Sim + AnyGrasp Project

> **Related docs:** [`PROJECT_STATUS.md`](PROJECT_STATUS.md) is the short, supervisor-facing
> summary (state, structure, open issues, next steps). [`AGENT_SESSION.md`](AGENT_SESSION.md) is
> the detailed engineering log and bug register. [`END_TO_END_TESTING.md`](END_TO_END_TESTING.md)
> is the runbook. **This** file describes the *design* and the decisions behind it.

## Project Overview

This project implements a perception-guided grasping pipeline for the **OpenManipulator-X** robotic arm.

The final system consists of:

```
Teammate
    │
Scene.pcd
Object.pcd
    │
    ▼
MoveIt Container (ROS Noetic)
    │
    │  HTTP Request
    ▼
AnyGrasp Container
    │
Best Grasp
    ▼
MoveIt
    │
TF camera→base
    ▼
Motion Planning
    ▼
Robot
```

The system is intentionally split into **two completely independent containers**.

---

# Containers

## 1. MoveIt Container

Contains

- ROS Noetic
- MoveIt
- OpenManipulator packages
- Robot control
- Planning
- TF

Responsible for

- Receiving teammate point clouds
- Calling AnyGrasp service
- Transforming grasp from camera frame to base frame
- Executing grasp

---

## 2. AnyGrasp Container

Contains

- AnyGrasp SDK
- GSNet
- Flask REST server

NO ROS installed.

Reason:

The AnyGrasp container uses a node-locked license.

Rebuilding or recreating the container may invalidate the license.

Therefore we decided NOT to modify the container any more than necessary.

It simply acts as an inference server.

---

# Current Folder Structure

```
anygrasp_sdk/

├── grasp_detection/
├── grasp_tracking/

├── anygrasp_pipeline/
│
├── loader.py
├── mask.py
├── detector.py
├── transforms.py
├── visualize.py
├── config.py
├── main.py
├── server.py
│
└── example_data/
        scene.pcd
        object.pcd
```

MoveIt workspace

```
ws_moveit/

src/

X_moveit_config/

open_manipulator_description/

grasp_pipeline/

scripts/
    grasp_client.py
    tf_utils.py
    grasp_planner.py
    executor.py
    live_capture.py
    base_teleport.py      (new — mobile-base placement for grasp reach, see AGENT_SESSION.md §4)

spot_policy_control/      (new — Isaac Sim locomotion controller, see its own AGENT.md; not yet
                            wired to the grasp pipeline. NOTE: drives Boston Dynamics Spot, not
                            the Unitree Go1 named in the lab brief — unresolved, see
                            AGENT_SESSION.md §1/§7)
```

> ⚠️ `scene_receiver.py` above was planned but never built — see AGENT_SESSION.md §4 for current
> status. `planner.py` was renamed `grasp_planner.py` during implementation.
>
> ⚠️ **As of 2026-09-16, this tree lives in `~/ws_moveit_clean` on `pringles` as its own git repo,
> separate from and ahead of `quadruped-atHome`'s `nav/` tree, and `grasp_pipeline`/
> `spot_policy_control` there are uncommitted anywhere.** See `AGENT_SESSION.md` §2 and Roadmap
> Step 6 before assuming this repo's `nav/` reflects current, tested code.

---

# Current Status

Completed

✓ Point cloud loader

✓ Object mask generation

✓ AnyGrasp wrapper

✓ Visualization

✓ Flask REST API

✓ End-to-end AnyGrasp inference

✓ grasp_client.py, TF conversion (`tf_utils.py`), MoveIt planning (`grasp_planner.py`), robot
execution (`executor.py`) — all built and run against Isaac Sim's live simulated camera, not just
canned data. See `AGENT_SESSION.md` for exactly what "run" means here (full chain executes; a
camera-mount calibration gap was found and partially fixed; no pick has fully succeeded yet).

✓ (2026-09-16, uncommitted) `base_teleport.py` — computes mobile-base placement so the arm can
reach a grasp approach vector outside its own fixed-base workspace; wired into `grasp_planner.py`
and `executor.py`. A separate, not-yet-integrated `spot_policy_control` package can drive a
simulated quadruped's locomotion in Isaac Sim, but the two aren't connected to each other yet.

Remaining

- A real pick actually succeeding against a placed object (currently blocked on the camera
  calibration gap above and on `moveit_isaac_controller_manager`'s fire-and-forget execution)
- `scene_receiver.py` (publishing captured geometry into the MoveIt planning scene for collision
  awareness / `attach_object()`)
- Connecting `base_teleport.py`'s output to an actual locomotion controller (`spot_policy_control`
  or otherwise) — currently the two exist independently
- Resolving whether the target quadruped is Unitree Go1 (per the lab brief) or Boston Dynamics
  Spot (what `spot_policy_control` actually drives)
- Getting the newest work (`base_teleport.py`, `spot_policy_control`) into version control — as of
  this writing both exist only, uncommitted, in a separate `ws_moveit_clean` git repo on `pringles`

`AGENT_SESSION.md` is the up-to-date source for exact status, bugs, and next steps — this section
is a snapshot, not a live tracker.

---

# Important Decisions

## 1.

The teammate provides TWO point clouds

```
scene.pcd

object.pcd
```

Both are already expressed in the SAME camera coordinate frame.

---

## 2.

Object mask is NOT a 2D segmentation mask.

It is a boolean array

```
shape = (N,)
dtype = bool
```

matching

```
scene.points
```

Example

```
scene.points

P1
P2
P3
P4
P5

mask

False
True
True
False
False
```

This mask is passed directly into

```
region_steering
```

---

## 3.

Collision detection is performed using the ENTIRE scene.

Therefore

```
scene.points
```

are always passed into AnyGrasp.

The object cloud is ONLY used for generating the region mask.

---

## 4.

Inference uses

```
scene.points

+

object_mask
```

NOT

```
object.points
```

The wrapper currently supports

```
predict_all(points)

predict(points,
        region_mask,
        collision_detection=True)
```

---

## 5.

The wrapper returns GraspGroup.

The caller chooses

```
best

top5

top20

etc
```

instead of detector.py always returning only the best grasp.

---

# Current REST API

Health

GET

```
/
```

returns

```
{
    "status":"running",
    "service":"AnyGrasp"
}
```

Prediction

POST

```
/predict
```

Multipart form

```
scene.pcd

object.pcd
```

Returns

```
{
    success,

    score,

    translation,

    rotation,

    width,

    depth
}
```

---

# Important Python Files

loader.py

Loads

```
PCD

↓

Open3D PointCloud

↓

numpy arrays
```

mask.py

Computes object mask using nearest neighbour search.

detector.py

Wrapper around

```
create_detector()

get_grasp()
```

visualize.py

Uses Open3D

Displays

- point cloud

- grippers

server.py

Flask inference server.

main.py

Standalone testing script.

---

# Docker

Container

```
anygrasp_display
```

Start

```
docker start anygrasp_display

docker exec -it anygrasp_display bash
```

Workspace

```
/workspace/anygrasp_sdk
```

Run standalone

```
cd /workspace/anygrasp_sdk/anygrasp_pipeline

python3 main.py
```

Run server

```
python3 server.py
```

Test service

```
curl http://127.0.0.1:5000/
```

Prediction

```
curl -X POST \
-F scene=@/workspace/anygrasp_sdk/anygrasp_pipeline/example_data/scene.pcd \
-F object=@/workspace/anygrasp_sdk/anygrasp_pipeline/example_data/object.pcd \
http://127.0.0.1:5000/predict
```

---

# MoveIt Container

Start

```
docker start ros_noetic

docker exec -it ros_noetic bash
```

ROS

```
source /opt/ros/noetic/setup.bash

source ~/ws_moveit/devel/setup.bash
```

Run

```
roscore
```

---

# Isaac Sim

Version

```
Isaac Sim 4.5
```

Important

Driver 595 causes crashes.

Driver was downgraded to

```
580
```

which fixed Isaac Sim.

Launch

```
cd ~/isaacsim

./isaac-sim.sh
```

---

# Camera

Intel RealSense D455

Camera frame

```
camera_link
```

Point clouds from teammate are already expressed in

```
camera_link
```

---

# TF

Final execution requires

```
camera_link

↓

base_link
```

This transformation will be obtained INSIDE the MoveIt container using ROS TF.

The AnyGrasp container intentionally has NO ROS.

---

# Why No ROS in AnyGrasp?

Reasons

- Keeps inference independent.

- Avoids rebuilding the licensed container.

- Easier deployment.

Communication occurs using

HTTP REST

MoveIt becomes the orchestrator.

---

# Planned Remaining Pipeline

```
scene_receiver.py

↓

HTTP POST

↓

AnyGrasp Server

↓

Best Grasp

↓

TF

camera

↓

base

↓

MoveIt

↓

Robot
```

---

# Current Working Test

Synthetic PCD files were generated.

Current output

```
50000 scene points

10000 object points

874 grasps

best score ≈0.068
```

Visualization is working.

REST API is working.

The next major milestone is implementing the MoveIt-side HTTP client and grasp execution.

> ⚠️ The paragraph above is an early snapshot and is now out of date — the MoveIt-side client and
> grasp execution are built and running. See [`PROJECT_STATUS.md`](PROJECT_STATUS.md) for the
> current state.

---

# Hard-won constraints (read before changing frames, IK or limits)

Each of these cost significant debugging time, and several were *silent* — printed values looked
correct while the robot physically moved somewhere else. Full detail per item is in
[`AGENT_SESSION.md`](AGENT_SESSION.md)'s bug register.

1. **`set_position_target()` interprets raw XYZ in the group's pose reference frame**, which
   defaults to the planning frame (`world`) — *not* whatever frame you computed the target in.
   `executor.py` must call `set_pose_reference_frame(tf_utils.DEFAULT_BASE_FRAME)`. Without it,
   link1-frame targets were executed as world-frame coordinates and the arm landed off by exactly
   the base-teleport transform (79.6mm measured). RViz looked correct throughout, because a
   `PoseStamped` carries its frame while raw coordinates do not. (BUG-19)

2. **This arm is 4-DOF and uses `position_only_ik`.** It cannot achieve AnyGrasp's arbitrary 6-DOF
   orientations. Consequences: `compute_cartesian_path()` fails 0% every time (it needs orientation
   reachable at every interpolated step), and grasp *orientation* is effectively advisory — only
   position is commanded. Don't reintroduce Cartesian approaches or pose targets expecting
   orientation control.

3. **AnyGrasp's convention is: rotation matrix column 0 = approach axis, column 1 = closing axis**
   (`grasp_detection/USAGE.md`, Note 2), matching this robot's own local +X / +Y. Therefore
   `ROTATION_OFFSET_QUATERNION` must be **identity**. A stray 90° yaw there silently made every
   "approach vector" read the *closing* axis instead, sending base placement to the wrong side of
   the object. (BUG-15)

4. **The SRDF virtual joint is `planar`, not `fixed`.** A `fixed` virtual joint is baked into
   MoveIt's model at load time and never re-read from TF, so no amount of TF broadcasting will move
   the arm's base as far as the planner is concerned. Because it's planar (x, y, yaw), a vector's
   Z-component is identical in `world` and `link1` — prefer `link1` as a gravity reference, since
   `world` only exists while a `BaseTeleporter` is broadcasting. (BUG-17)

5. **Plan targets must keep a margin from mechanical joint limits.** A target sitting exactly on a
   limit plans fine but the physics engine will not drive to it — the arm silently doesn't move.
   Joints 1–4 carry a 0.05 rad margin in the URDF for this reason.

6. **Base standoff must account for grasp height.** Bounding only the horizontal distance lets the
   true 3-D distance to an elevated grasp exceed the arm's reach. `compute_base_placement()` shrinks
   standoff against a reach budget (`safe_max_reach`).

7. **The AnyGrasp container has no ROS and cannot know which way is up.** Any "parallel to the
   ground" filtering belongs on the ROS side. If the AnyGrasp side needs to reproduce the same
   selection (e.g. `main.py --level-only` for visualisation), it reads the up-vector from the
   `<scene>.meta.json` sidecar that `live_capture.py` writes from live TF. (BUG-18)

8. **A sparse point cloud is usually a framing problem, not a code bug.** Check `--dump-stats`'s raw
   point count and the depth image's finite-pixel mean first: if the mean sits at the near-clip
   value, the gripper or arm is filling the camera's view and the arm needs repositioning.

9. **Never derive `camera_optical_joint`'s rotation from a UI's Euler-angle display alone —
   verify against a live ground-truth object position first.** Two independent paper derivations
   of this rotation (one when it was first set, one this session composing Isaac's RSD455 sensor
   chain) were both wrong when tested live; one made the error *larger*. The only method that
   actually worked: print the raw, untransformed grasp translation in `camera_optical_frame`
   directly in `tf_utils.py`, compare it against a known object position, and solve/verify from
   that. If the camera's actual USD mount ever changes again (e.g. swapping the RSD455 asset for
   something else), re-verify this the same way — don't assume the current `rpy` still applies.
   (BUG-13, BUG-20)

10. **This robot has a fixed base — there is no mobile base (Go1 quadruped) in the current scene.**
    `isaac_sim_native_gui.py` (a teammate's file, in the separate `humanoid_lab` repo) assumes a
    mobile base walking around a table; its `read_table_geometry()`/`RIG_PATH`-teleport view
    planner does not apply here and will crash or silently do nothing useful. Multi-view capture
    for this robot instead relocates the *arm's own base* using this repo's existing
    `base_teleport.py` + `isaac_sim_teleport_listener.py` machinery (see `multi_view_capture.py`),
    exactly like `executor.py`'s `--teleport` path already does for grasp execution.

11. **A camera-pose function that divides out a "world" reference prim must never use a prim that
    itself gets moved.** `isaac_sim_native_gui.py`'s `WORLD_FRAME_PRIM` was set to
    `/open_manipulator_x` to work around this stage having no `/World` wrapper Xform — fine until
    `multi_view_capture.py` started physically teleporting that exact prim for multi-view capture,
    at which point every recorded camera pose silently had the base relocation divided back out
    again. Use a prim that's guaranteed to stay fixed (this stage: `/FlatGrid`, a sibling of
    `open_manipulator_x` at the stage root).