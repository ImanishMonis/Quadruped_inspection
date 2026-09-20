# Project Status — Perception-Guided Grasping (WP3)

**Project:** Human-in-the-Loop Object Inspection and Grasping with a Quadruped Robot
**Supervisor:** Niklas Mueller-Goldingen
**Work package:** WP3 — Grasp pose estimation, MoveIt planning, execution
**Platform:** ROBOTIS OpenManipulator-X (4-DOF) + Intel RealSense D455, validated in Isaac Sim 4.5
**Last updated:** 2026-09-20

---

## 1. Summary

The WP3 pipeline is **built end-to-end and runs in simulation**: a live depth capture from the
simulated RealSense is converted to a point cloud, sent to AnyGrasp for 6-DOF grasp detection,
transformed into the robot's frame, and executed through MoveIt onto the arm in Isaac Sim.

The full sequence (capture → detect → transform → plan → execute → close gripper → retreat) now
completes without error. What is **not yet demonstrated** is a verified successful pick of the
object — the remaining blockers are described in §5.

---

## 2. System architecture

Two deliberately independent Docker containers communicate over HTTP. The AnyGrasp container holds
a node-locked license, so it is never rebuilt and contains no ROS; REST is the boundary between
them.

```
Isaac Sim 4.5  (host, GPU)
  simulated RealSense D455  ──depth + camera_info──┐
  simulated OpenManipulator-X  ◄──joint commands───┤
                                                   │
┌──────────────────────────────────────────────────┴────────┐
│ ros_noetic container (ROS 1 Noetic + MoveIt)              │
│   live_capture.py   depth image → scene.pcd / object.pcd  │
│   grasp_client.py   HTTP POST → AnyGrasp                  │
│   tf_utils.py       camera frame → robot base frame       │
│   grasp_planner.py  grasp + pre-grasp poses, base placing │
│   base_teleport.py  mobile-base placement computation     │
│   executor.py       full pick sequence via MoveIt         │
└───────────────────────────┬───────────────────────────────┘
                            │ HTTP (REST)
┌───────────────────────────┴───────────────────────────────┐
│ anygrasp_display container (AnyGrasp SDK, no ROS)         │
│   server.py    Flask /predict endpoint                    │
│   detector.py  AnyGrasp/GSNet wrapper                     │
│   main.py      standalone run + Open3D visualization      │
└───────────────────────────────────────────────────────────┘
```

---

## 3. Repository structure

```
ros_noetic/src/
  grasp_pipeline/scripts/          ← WP3 core (our work)
    live_capture.py                capture point clouds from the simulated camera
    grasp_client.py                REST client for the AnyGrasp service
    tf_utils.py                    frame conversions, pose maths, TF broadcasting
    grasp_planner.py               grasp/pre-grasp pose computation, grasp selection
    base_teleport.py               mobile-base placement for out-of-reach grasps
    executor.py                    full pick sequence through MoveIt
    multi_view_capture.py          multi-view point-cloud capture (left/right/top/close)
    isaac_sim_teleport_listener.py runs inside Isaac Sim, relocates the robot
  X_moveit_config/                 MoveIt configuration (URDF/SRDF, planners, launch)
  moveit_isaac_controller_manager/ MoveIt → Isaac Sim controller plugin
  open_manipulator_control/        trajectory bridge to Isaac Sim
  spot_policy_control/             quadruped locomotion controller (Isaac Sim)

anygrasp_display/anygrasp_sdk/anygrasp_pipeline/
  server.py, detector.py, loader.py, mask.py, main.py, visualize.py

docs/
  PROJECT_STATUS.md                this file
  AGENT.md                         design and architectural decisions
  AGENT_SESSION.md                 detailed engineering log and bug register
  END_TO_END_TESTING.md            step-by-step runbook
```

---

## 4. Current state by component

| Component | State |
|---|---|
| Point-cloud capture from simulated camera | Working |
| AnyGrasp inference service (REST) | Working |
| Grasp transformation into robot frame | Working, verified numerically (camera-mount rotation fixed this session, see §6) |
| Grasp selection (best-score, or "parallel to ground" filter) | Working |
| Mobile-base placement computation | Working, height-aware |
| MoveIt planning and execution | Working |
| Full pick sequence runs to completion | Yes |
| **Verified successful pick of the object** | **Not yet** |
| Object reconstruction from multiple viewpoints (WP2) | Not in this work package |
| Collision-aware planning scene (`scene_receiver.py`) | Not implemented |

---

## 5. Open issues

1. **AnyGrasp sometimes selects background geometry** rather than the target object. The object
   mask is currently a depth-range filter (10 cm–1 m) standing in for WP1's user-selection GUI; when
   the camera's view includes floor or walls in that range, candidate grasps can land on them. A
   reach/height sanity check that rejects such candidates before planning would make this fail
   clearly instead of as a planning timeout.

2. **Execution feedback is open-loop.** The MoveIt↔Isaac Sim controller reports success immediately
   rather than confirming arrival, so a motion command can report success without the arm having
   reached the target. Joint states must be checked independently to confirm a move. Replacing this
   with a proper `FollowJointTrajectory` action server is the correct fix.

3. ~~**Camera mount calibration.**~~ **Resolved this session (BUG-20).** The camera's *position*
   mount offset was already correct; the *rotation* (`camera_optical_joint`) was missing a 90° roll,
   which silently turned a real object's vertical offset from the optical axis into a spurious
   lateral offset in the robot's frame. Fixed and verified live against a known object position,
   both centred and off-centre.

4. **Physical base relocation in simulation is verified for grasp execution, but a related
   multi-view fusion frame bug was found and fixed.** `executor.py --teleport`'s base relocation
   works. Separately, `isaac_sim_native_gui.py` (the multi-view capture GUI, teammate's repo) used
   the wrong "fixed" reference prim for reporting camera poses — one that itself gets teleported
   during multi-view capture — which silently cancelled out the base relocation in every recorded
   pose. Fixed by switching to a prim that's actually fixed (`/FlatGrid`); **not yet re-verified**
   with the diagnostic tool (`sam2_service/refine_session_poses.py --dry-run`) after the fix — do
   that first next session.

---

## 6. Issues encountered and how they were resolved

The largest share of effort went into diagnosing silent coordinate-frame and configuration faults —
cases where every printed value looked correct while the robot physically moved somewhere else.

| Issue | Resolution |
|---|---|
| Robot moved to a position offset from the computed grasp | Targets were computed in the robot base frame but passed to MoveIt as raw coordinates, which it interprets in the **planning frame** (`world`). Set the pose reference frame explicitly. Positional error dropped from **79.6 mm to 1.5 mm**. |
| Base placement sent the robot to the wrong side of the object | A rotation offset constant was a 90° rotation instead of identity, so the "approach axis" being read was actually the gripper's *closing* axis. Reset to identity; verified the base now sits along the true approach line. |
| Base "teleport" had no effect on planning | Four separate causes: duplicate TF broadcasters, a fixed joint baked into the URDF, a `fixed` virtual joint in the SRDF (which MoveIt never re-reads from TF), and a stale pose timestamp causing historical rather than current transform lookups. All four fixed. |
| Some poses failed to execute entirely in simulation | Planner produced targets sitting exactly on a joint's mechanical limit, which the physics engine would not drive to. Added a safety margin to the planning limits. |
| Elevated grasps were unreachable despite correct base placement | Base standoff only bounded horizontal distance, so total 3-D distance to a raised object exceeded the arm's reach. Made the standoff height-aware. |
| Straight-line (Cartesian) approach always failed | The arm is 4-DOF and uses position-only IK, so it cannot satisfy the orientation constraints a Cartesian path requires. Replaced with a direct position target. |
| Point clouds were extremely sparse | Not a software fault — the arm pose put the gripper in front of the camera, so most of the frame was near-clip noise. Identified by inspecting the raw depth image directly. |
| AnyGrasp service returned HTTP 500 on every request | A grasp-filtering function indexed the SDK's grasp container with a list, which it does not support. Disabled it in favour of filtering on the ROS side, where the world orientation is actually known. |
| A centred object still produced a nonzero lateral (Y) grasp offset | `camera_optical_joint`'s rotation was missing a 90° roll, so the camera's real vertical offset from an object was injected as a lateral offset in the robot's frame. Two paper derivations of the correct rotation were tried and both were wrong when tested live (one made the error larger); fixed by printing the raw, untransformed camera-frame translation and solving directly against a known object position. Verified for both a centred and an off-centre object. |

A full engineering log with reproduction steps for each is maintained in `AGENT_SESSION.md`.

---

## 7. Next steps

1. Verify a complete, successful pick of the object in simulation, confirming arrival from joint
   states rather than command return values.
2. Re-run `sam2_service/refine_session_poses.py --dry-run` on a fresh multi-view session to confirm
   the `WORLD_FRAME_PRIM` fusion-frame fix actually resolved the fusion misalignment.
3. Add a reachability check that rejects unreachable grasp candidates before planning.
4. Replace the open-loop controller with a `FollowJointTrajectory` action server.
5. Publish captured geometry into the MoveIt planning scene for collision-aware planning.
6. Begin WP4 evaluation metrics (grasp success rate, task duration).
