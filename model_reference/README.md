# model_reference

Version-controlled copies of robot description files that are **otherwise untracked**.

## Why these are here

The live workspace references the robot description through
`ros_noetic/src/open_manipulator_description`, which is a **symlink to a machine-local path**
(`/root/model/...` inside the container, `~/model/open_manipulator/...` on the host). That path is
not a git repository, so edits to it were living on a single machine with no backup and no history.

Several of those edits are load-bearing — without them the arm either cannot be planned for
correctly or will not physically move:

| File | Change | Why it matters |
|---|---|---|
| `open_manipulator_x.urdf.xacro` | `world` link and `world_fixed` joint removed | A `type="fixed"` joint here was published by `robot_state_publisher` as a permanent static `world -> link1` transform, which fought the base-placement broadcast. `link1` is now parentless, matching the SRDF's `planar` virtual joint. |
| `open_manipulator_x.urdf.xacro` | `camera_optical_joint` given `rpy="0 1.5707963 0"` | Aligns the camera's optical forward axis with the gripper's approach axis; without it the captured cloud transforms into nonsense. |
| `open_manipulator_x_arm.urdf.xacro` | 0.05 rad safety margin on joints 1–4 | A target sitting exactly on a mechanical limit plans fine but the physics engine will not drive to it — the arm silently doesn't move. |

## Keeping these in sync

These are **copies, not the files actually loaded at runtime**. After editing the live files under
`~/model/open_manipulator/open_manipulator_description/urdf/open_manipulator_x/`, copy them here so
the change is tracked. Conversely, a fresh checkout of this repo does **not** put these files where
ROS will find them — they still have to be placed at the machine-local path the symlink points to.

Longer term the right fix is to vendor `open_manipulator_description` into this repo properly (or
add it as a submodule) and drop the symlink, so there is only one copy. See `docs/AGENT_SESSION.md`
(BUG-12) for background.
