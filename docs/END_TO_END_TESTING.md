# End-to-End Testing Guide

Copy-paste runbook for testing the WP3 grasp pipeline start to finish:
AnyGrasp inference -> ROS client -> TF conversion -> grasp planning ->
MoveIt execution -> Isaac Sim.

Companion to [`AGENT.md`](AGENT.md) (design) and [`AGENT_SESSION.md`](AGENT_SESSION.md)
(state/history/bugs). This file is just "what to actually type," kept
in sync with whatever currently works - if a step here stops matching
reality, fix this file, don't work around it silently.

**Assumes:** you're on the machine with the containers (`anygrasp_display`,
`ros_noetic`) and Isaac Sim already set up - see `AGENT_SESSION.md` §2 for
which machine that is.

---

## 0. Containers up

```bash
docker start anygrasp_display ros_noetic   # no-op if already running
docker ps   # confirm both show "Up"
```

## 1. Isaac Sim

Launch it, open the robot stage, **press Play**, then confirm it's actually
stepping (not just rendering) before doing anything else:

```bash
docker exec ros_noetic bash -c "source /opt/ros/noetic/setup.bash && rostopic hz /clock"
```

Expect a steady ~60Hz. If you get "no new messages," the sim isn't
playing, or the ActionGraph is broken - see `AGENT_SESSION.md` §5 (BUG-5)
and the session log for the ActionGraph corruption/reset-loop story
before going further.

## 2. AnyGrasp server

```bash
docker exec -d anygrasp_display bash -c \
  "cd /workspace/anygrasp_sdk/anygrasp_pipeline && python3 server.py > /tmp/anygrasp_server.log 2>&1"
```

Wait ~5s for the license check + model load, then confirm:

```bash
docker exec anygrasp_display curl -s http://127.0.0.1:5000/
# expect: {"status":"running","service":"AnyGrasp"}
```

If it's already running from a previous session, skip the `docker exec -d`
and just run the curl check.

**Server URL note:** `grasp_client.py`'s default (`http://172.17.0.2:5000`)
is the AnyGrasp container's docker-bridge IP, which is only stable until
the container is recreated. If it's changed, check with:

```bash
docker inspect anygrasp_display --format '{{.NetworkSettings.Networks.bridge.IPAddress}}'
```

and pass `--server-url http://<that-ip>:5000` to any script below, or set
it once via `rosparam set /grasp_client/server_url http://<ip>:5000` if
you've wired that in.

## 3. MoveIt

```bash
docker exec -it ros_noetic bash
source /opt/ros/noetic/setup.bash
source ~/ws_moveit/devel/setup.bash
roslaunch X_moveit_config demo.launch
```

If RViz opens as a tiny/blank window and never finishes rendering, that's a GPU
passthrough problem, not a MoveIt problem - see "RViz hangs at a thumbnail-sized
window" near the end of this file before debugging anything else.

Leave this running in its own terminal/tab. In a second shell, confirm
`trajectory_bridge` came up on its own (it's launched automatically from
`fake_moveit_controller_manager.launch.xml` now - no separate `rosrun`
needed):

```bash
docker exec ros_noetic bash -c "source /opt/ros/noetic/setup.bash && rosnode list"
# expect to see: /move_group /robot_state_publisher /trajectory_bridge
#                /OmniIsaacRosBridge /rviz_... /virtual_joint_broadcaster_0
```

If `trajectory_bridge` is missing, MoveIt's plans will "succeed" but the
robot in Isaac Sim will never move - see `AGENT_SESSION.md`'s session log
entry on this exact failure mode.

## 4. Example point clouds into the container (one-time)

```bash
docker exec ros_noetic mkdir -p /root/ws_moveit/src/grasp_pipeline/example_data
docker cp /home/user/monisi1/git/quadruped-atHome/detection/anygrasp_pipeline/example_data/scene.pcd \
          ros_noetic:/root/ws_moveit/src/grasp_pipeline/example_data/scene.pcd
docker cp /home/user/monisi1/git/quadruped-atHome/detection/anygrasp_pipeline/example_data/object.pcd \
          ros_noetic:/root/ws_moveit/src/grasp_pipeline/example_data/object.pcd
```

Skip if `docker exec ros_noetic ls /root/ws_moveit/src/grasp_pipeline/example_data/`
already shows both files.

## 5. Run the pipeline, stage by stage

Open a shell in `ros_noetic` and source everything:

```bash
docker exec -it ros_noetic bash
source /opt/ros/noetic/setup.bash
source ~/ws_moveit/devel/setup.bash
cd /root/ws_moveit/src/grasp_pipeline/scripts
```

Run each of these **from inside that shell**, in order, reading the
output before moving to the next one:

```bash
# 1) Call AnyGrasp directly, print top-5 grasps as JSON
python3 grasp_client.py \
  --scene /root/ws_moveit/src/grasp_pipeline/example_data/scene.pcd \
  --object /root/ws_moveit/src/grasp_pipeline/example_data/object.pcd \
  --top-k 5
```

```bash
# 2) Convert the best grasp -> link1 frame, print it, and broadcast it
#    live as a TF frame named "grasp_pose" for as long as this runs.
#    Open RViz (Add -> TF, or Add -> Axes with frame "grasp_pose") and
#    LOOK where it lands relative to the real arm before trusting it -
#    this is the frame-convention check AGENT_SESSION.md flags as
#    mandatory, not optional. Ctrl+C to stop.
python3 tf_utils.py \
  --scene /root/ws_moveit/src/grasp_pipeline/example_data/scene.pcd \
  --object /root/ws_moveit/src/grasp_pipeline/example_data/object.pcd
```

```bash
# 3) Print the planned pre-grasp + grasp waypoints (no motion yet)
python3 grasp_planner.py \
  --scene /root/ws_moveit/src/grasp_pipeline/example_data/scene.pcd \
  --object /root/ws_moveit/src/grasp_pipeline/example_data/object.pcd
```

```bash
# 4) Run the actual pick through MoveIt - watch the Isaac Sim viewport.
#    Expected: gripper opens -> arm moves to pre-grasp -> straight-line
#    approach -> gripper closes -> straight-line retreat.
python3 executor.py \
  --scene /root/ws_moveit/src/grasp_pipeline/example_data/scene.pcd \
  --object /root/ws_moveit/src/grasp_pipeline/example_data/object.pcd
```

## 6. Useful checks while any of the above runs

From a second terminal (no need to `source` anything if you prefix each
command like this):

```bash
docker exec ros_noetic bash -c "source /opt/ros/noetic/setup.bash && rostopic hz /clock"              # sim actually stepping
docker exec ros_noetic bash -c "source /opt/ros/noetic/setup.bash && rostopic echo -n1 /joint_states"  # current arm pose
docker exec ros_noetic bash -c "source /opt/ros/noetic/setup.bash && rosnode list"                     # trajectory_bridge, move_group alive?
docker exec anygrasp_display tail -30 /tmp/anygrasp_server.log                                         # AnyGrasp server errors
```

## 7. If step 5.4 (`executor.py`) times out

This happened during initial testing: `ABORTED: TIMED_OUT` trying to reach
the pre-grasp pose, even at 10s x 10 planning attempts.

**Known cause (not a pipeline bug):** `example_data/scene.pcd` and
`object.pcd` are a canned real-world capture from an unknown camera pose.
`camera_link` is wrist-mounted (`open_manipulator_x.urdf.xacro`'s
`camera_joint`, parent `link5`), so it only means something relative to
the exact arm pose it was captured at. Converting that canned cloud
through whatever pose the simulated arm happens to be in *right now* can
put the grasp target outside the arm's reachable workspace - that's a
test-data mismatch, not a code defect.

**Quick diagnostic** - plan to the same position with a trivial
orientation, to tell a reach limit apart from a frame-convention bug:

```bash
docker exec ros_noetic bash -c "source /opt/ros/noetic/setup.bash && source ~/ws_moveit/devel/setup.bash && python3 -c \"
import sys, rospy, moveit_commander
from geometry_msgs.msg import PoseStamped
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('diag', anonymous=True)
arm = moveit_commander.MoveGroupCommander('x_arm')
arm.set_planning_time(10); arm.set_num_planning_attempts(10)
p = PoseStamped(); p.header.frame_id = 'link1'
p.pose.position.x, p.pose.position.y, p.pose.position.z = 0.309, 0.113, 0.328
p.pose.orientation.w = 1.0
arm.set_pose_target(p)
print('PLAN_OK:', bool(arm.plan()[0]))
moveit_commander.roscpp_shutdown()
\""
```

If that also fails -> reach limit, not orientation. **The real fix is a
live point-cloud capture from Isaac Sim's own simulated RealSense**
(confirmed live: `/camera/depth/image_raw` @ 30Hz, `/camera/color/camera_info`
with real intrinsics, `frame_id: camera_optical_frame`) instead of the
canned `example_data` - then the capture pose and the TF lookup pose are
the same consistent simulated camera, and the mismatch goes away. Not
built yet as of this writing; check `AGENT_SESSION.md`'s roadmap/session
log for current status before assuming it still needs doing.

If the diagnostic plan *succeeds*, that instead points at the rotation
offset in `tf_utils.py` (`ROTATION_OFFSET_QUATERNION`) - re-read that
module's docstring and verify empirically in RViz (step 5.2 above) which
axis is actually wrong. **As of session 4, the live and git copies of this
constant disagree with each other and with the module's own docstring -
see `AGENT_SESSION.md` BUG-15 before trusting either one.**

## 8. RViz sits on "Initializing" forever (config has `MotionPlanning` display)

Different symptom from §9's window-hang below - the RViz window itself
opens and renders fine, but shows an "Initializing" progress dialog that
never completes, specifically when loading a config that includes the
`moveit_rviz_plugin/MotionPlanning` display (`X_moveit_config/launch/moveit.rviz`
has exactly one such display). Plain `rviz` with no config, or a config
without that display, opens instantly by comparison - that's the tell.

**Confirmed root cause (session 4, verified with a live gdb backtrace on
the actually-hung process, not guessed):** `demo.launch:2` hardcodes
`<param name="use_sim_time" value="true" />`, because this whole config is
meant to run with Isaac Sim feeding `/clock` over the ROS bridge. If
`/clock` has no publisher - i.e. **Isaac Sim isn't running, or isn't
playing** - ROS time never becomes valid. `MotionPlanning`'s "Object
Recognition" panel constructs an actionlib client on startup, and that
client's `initClient()` calls `ros::Time::waitForValid()`, which blocks
**forever** with no timeout and no error message if sim time never arrives.
Confirmed via `gdb -p <rviz_pid> -batch -ex 'thread apply all bt'` (needs
`--cap-add=SYS_PTRACE` on the container, off by default) showing the main
thread stuck in exactly:

```
ros::Time::waitForValid()
 -> actionlib::ActionClient<ObjectRecognitionAction>::initClient()
 -> MotionPlanningFrame::MotionPlanningFrame(...)
 -> MotionPlanningDisplay::onInitialize()
```

and separately confirmed via `rosparam get /use_sim_time` (`true`) and
`rostopic hz /clock` (`no new messages`) while the hang was live.

**Fix:**
- **If testing against Isaac Sim** (the intended workflow): start Isaac Sim
  and press Play - so `/clock` is already ticking - *before*
  `roslaunch demo.launch`.
- **If testing RViz/MoveIt planning standalone, without Isaac Sim running:**
  there is currently no launch arg for this - `use_sim_time` is hardcoded
  `true` in `demo.launch:2`. Either set it to `false` for that test run
  (`rosparam set /use_sim_time false` won't help *after* roscore is already
  up with a param set by the launch file taking priority at node startup -
  edit the line directly, or comment it out, for a standalone RViz-only
  session and revert before testing against Isaac Sim again), or publish a
  fake clock (`rosrun rostopic pub /clock rosgraph_msgs/Clock -r 10
  "clock: {secs: 0, nsecs: 0}"` in a spare terminal) to unblock it without
  editing the launch file. Neither is applied by default as of this
  writing - the user was asked and preferred to just remember to start
  Isaac Sim first rather than add a `sim:=`/`use_sim_time:=` arg to
  `demo.launch`.

**Unrelated but easy to confuse with this:** a previous `demo.launch` run
left running (same static node names for `move_group`/`rviz`/
`trajectory_bridge`) can also produce a stuck-looking RViz while the new
launch is mid-handoff with the old one. Rule this out first since it's a
one-line check:

```bash
docker exec ros_noetic bash -c "ps aux | grep -E 'rviz|move_group|roscore|roslaunch|trajectory_bridge|rosmaster' | grep -v grep"
```

If anything shows up from an earlier session (check the start time in the
`ps` output), kill it explicitly by PID rather than trying to `pkill -f`
with a pattern - patterns like `-f ros` risk matching and killing your own
shell if you're running the kill from inside a `bash -c "..."` string that
itself contains "ros". Once the list comes back empty and you've confirmed
`/clock` is actually ticking (or `use_sim_time` is `false`), `roslaunch
X_moveit_config demo.launch` fresh - give `move_group` 15-20s to load the
OMPL/CHOMP/Pilz planning pipelines before assuming anything is stuck.

## 9. RViz hangs at a thumbnail-sized window on `roslaunch ... demo.launch`

Found session 4 (2026-09-16) on `pringles`. Symptom: `roslaunch X_moveit_config
demo.launch` runs, `move_group` and the other nodes come up, but the RViz
window itself opens tiny and blank/frozen and never finishes rendering -
"hangs at the thumbnail-like window."

**Root cause: the `ros_noetic` container has no GPU passthrough at all.**
Checked directly:

```bash
docker inspect ros_noetic --format '{{.HostConfig.Runtime}}'   # -> runc, not nvidia
docker exec ros_noetic ls /dev/dri                             # -> No such file or directory
docker exec ros_noetic ls /dev/nvidia0                         # -> No such file or directory
```

Yet the host has everything needed to do this properly:

```bash
nvidia-smi --query-gpu=name,driver_version --format=csv
# NVIDIA GeForce RTX 4080 SUPER, 580.173.02
dpkg -l | grep nvidia-container-toolkit
# nvidia-container-toolkit 1.19.1-1 (installed)
```

So the capability is there, it just was never requested when this container
was created. Without `/dev/dri` or an Nvidia device inside the container,
RViz's Ogre render window can't get a real OpenGL context - the X11 side is
fine (the container's `/tmp/.X11-unix` bind mount and a correctly-exported
`DISPLAY` are enough to open a window), but there is nothing to actually draw
into it, which is consistent with a window that opens and then never
completes its first frame.

**Fix - recreate `ros_noetic` with GPU passthrough.** Verified working session 4
(2026-09-16). Requires stopping and recreating the container (`docker update`
cannot add device access to a running/stopped container), so **confirm before
running this** - it wipes anything installed at runtime inside the container
that isn't under one of the bind mounts below (build products under
`~/ws_moveit_clean` and model files under `~/model` are safe, they're
bind-mounted from the host):

```bash
docker stop ros_noetic
docker rm ros_noetic
docker run -dit \
  --name ros_noetic \
  --gpus all \
  --network host \
  -e DISPLAY=:1 \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/user/monisi1/ws_moveit_clean:/root/ws_moveit \
  -v /home/user/monisi1/model:/root/model \
  osrf/ros:noetic-desktop-full
docker exec ros_noetic bash -c "source /opt/ros/noetic/setup.bash && source ~/ws_moveit/devel/setup.bash 2>/dev/null; echo ok"
```

(`--gpus all` is the only change from the container's previous config; `-e
DISPLAY=:1` is set to match whatever the host's actual X display currently
is - check with `echo $DISPLAY` on the host first, it won't always be `:1`.)

Re-run `roslaunch X_moveit_config demo.launch` after recreating and confirm
RViz actually paints its 3D view, not just that the window appears.

If GPU passthrough still doesn't fix it, a quick way to at least unblock
testing (accepting a much slower, unaccelerated RViz) is to force Mesa's
software rasterizer instead of trying for a real GL context:

```bash
docker exec -it ros_noetic bash -c "export LIBGL_ALWAYS_SOFTWARE=1 && source /opt/ros/noetic/setup.bash && source ~/ws_moveit/devel/setup.bash && roslaunch X_moveit_config demo.launch"
```

This is a fallback, not the fix - prefer the `--gpus all` recreate above,
since `pringles` has a GPU that's otherwise unused for this container.

**Gotcha found the moment this was actually tried: recreating from a bare
`osrf/ros:noetic-desktop-full` image has no MoveIt in it.** `demo.launch`
will bring up `move_group` in its printed NODES list but it silently fails
to actually start, and RViz logs:

```
PluginlibFactory: The plugin for class 'moveit_rviz_plugin/MotionPlanning'
failed to load. ... does not exist.
```

Confirm with `rosnode list` - if `/move_group` is missing from that list
(even though `roslaunch`'s startup banner mentioned it), this is why. It
means the *original* container had `ros-noetic-moveit` installed manually
at some point, never captured in a Dockerfile, so a from-scratch recreate
loses it silently. Fix:

```bash
docker exec ros_noetic bash -c "apt-get update -qq && apt-get install -y ros-noetic-moveit"
```

Then kill and relaunch (`pkill -9 -f roscore; pkill -9 -f roslaunch` inside
the container, being careful not to also match and kill your own shell -
give it a specific-enough pattern - then re-run `roslaunch` fresh). Confirm
`/move_group` now shows up in `rosnode list` and the log has no plugin or GL
errors. **This apt install is not persisted** - it's back in the container's
writable layer only and will vanish on the next recreate, exactly like the
GPU flag did. See `AGENT_SESSION.md` Roadmap Step 7 for writing an actual
Dockerfile so this stops recurring.

---

## 10. Symptom → cause quick reference

Faults hit repeatedly during development, with the shortest route to the cause. Full detail per
item is in `AGENT_SESSION.md`'s bug register; the design constraints behind several of these are
summarised in `AGENT.md`'s "Hard-won constraints" section.

| Symptom | Most likely cause | Check |
|---|---|---|
| Gripper lands offset from the `grasp_pose` marker, which itself looks right in RViz | Targets computed in `link1` but executed as `world` coordinates | `arm.get_pose_reference_frame()` — must be `link1`, not `world` (BUG-19) |
| Base teleports to the wrong side of the object | `ROTATION_OFFSET_QUATERNION` non-identity → "approach axis" is really the closing axis | `grep ROTATION_OFFSET_QUATERNION tf_utils.py` — must be `(0,0,0,1)` (BUG-15) |
| Base placement has no effect on planning | SRDF virtual joint is `fixed` (baked into MoveIt at load, never re-read from TF) | `grep virtual_joint config/open_manipulator_x.srdf` — must be `planar` (BUG-17) |
| Arm doesn't move at all for certain poses; `go()` still returns `True` | Target sits exactly on a mechanical joint limit; physics won't drive to it | Compare planned joint values against the URDF `<limit>` values |
| `ABORTED: TIMED_OUT` on a target at correct standoff distance | Grasp is elevated — total 3-D distance exceeds reach even though horizontal distance is fine | `sqrt(x²+y²+z²)` vs. the ~0.38 m reach |
| Cartesian path returns 0% every time | 4-DOF arm + `position_only_ik` cannot satisfy orientation at each step | Expected — use a position target instead |
| Captured cloud extremely sparse (hundreds of points) | Framing: gripper/arm filling the camera view, everything else near-clip | `live_capture.py --dump-stats`; if the depth mean ≈ near-clip, reposition the arm |
| Grasp lands far away / above the robot | AnyGrasp picked background geometry, not the object | Check the raw translation's depth and the cloud's Y-spread — a near-constant Y means a flat surface, not an object |
| `/predict` returns HTTP 500 | Degenerate input (e.g. a 1-point cloud left over from an earlier test) | `head -12` the `.pcd` and check `POINTS` |
| `go()` returns `True` but the arm didn't arrive | Open-loop controller reports success immediately (BUG-5) | Always verify against `/joint_states` |
| An object dead-centre in front of the arm still produces a nonzero `Target Y` | `camera_optical_joint`'s rotation is missing a 90° roll — the camera's real *vertical* offset from an object is being injected as a *lateral* offset | Print the raw translation in `camera_optical_frame` (before any TF transform) — a real object dead-ahead should show `x≈0` there; if `Target Y` in `link1` is still nonzero despite that, it's this rotation, not detection (BUG-20) |
| Multi-view fused point cloud shows a doubled/misaligned edge | The camera-pose function's "fixed" reference prim is itself being teleported during capture, silently cancelling out base relocation | `sam2_service/refine_session_poses.py <session> --dry-run` — object-centroid spread should be ~0-1cm; if it's several cm and ICP barely helps, check `WORLD_FRAME_PRIM` isn't the prim being moved |

## Quick reference: file locations

| What | Path |
|---|---|
| AnyGrasp source (git, version-controlled) | `detection/anygrasp_pipeline/` |
| AnyGrasp source (live, mounted into `anygrasp_display`) | `~/anygrasp_docker/anygrasp_sdk/anygrasp_pipeline/` -> `/workspace/anygrasp_sdk/anygrasp_pipeline` |
| WP3 ROS scripts (git, version-controlled) | `nav/grasp_pipeline/scripts/` |
| WP3 ROS scripts (live, mounted into `ros_noetic`) | `~/ws_moveit_clean/src/grasp_pipeline/scripts/` -> `/root/ws_moveit/src/grasp_pipeline/scripts` |
| Robot URDF/xacro (live, shared by ROS *and* Isaac Sim) | `~/model/open_manipulator/open_manipulator_description/` -> `/root/model/...` inside `ros_noetic` |
| Mobile-base placement + Spot locomotion (live only, session 4) | `~/ws_moveit_clean/src/grasp_pipeline/scripts/base_teleport.py` and `~/ws_moveit_clean/src/spot_policy_control/` - **not in the git repo at all**, not even as a stale copy |

**The git repo and the live container paths are two separate copies on
disk, not one bind-mounted path** (except the URDF/xacro under `~/model`,
which genuinely is shared). Any fix to a Python file needs to land in
*both* the git repo and the live copy (`docker cp` from the git path into
the container works even though the live directories are root-owned) or
it silently only half-applies. This bit us twice already - see
`AGENT_SESSION.md`'s session log.

**Session 4 update:** it's now worse than "two copies" for some files - `~/ws_moveit_clean` turned
out to be its own separate git repository (not a checkout of *this* repo), and it's currently
*ahead* of this one for `grasp_pipeline` (see `tf_utils.py`'s diverged constants, `AGENT_SESSION.md`
BUG-15/16) and has two entire packages (`base_teleport.py`, `spot_policy_control/`) that exist
nowhere in this git repo, and aren't committed in `ws_moveit_clean`'s own repo either. See
`AGENT_SESSION.md` §2 and Roadmap Step 6 before assuming a fresh clone of this repo has current code.
