# AGENT.md - AI Developer Guide & Architectural Invariants

This file serves as the definitive technical briefing for AI coding assistants and developers maintaining or extending the **Spot Policy Control** package and the downstream mobile manipulation pipeline.

---

## 1. Project Context & Environment Split

The system operates as a hybrid architecture across the **Host OS** and a **Docker Container**:

```
+-------------------------------------------------------------------------+
| HOST (Linux: pringles)                                                  |
| - NVIDIA GPU Drivers, Vulkan, Display (:0)                              |
| - Isaac Sim 4.5.0 directory: /home/user/monisi1/isaacsim/               |
| - Standalone Python launcher: /home/user/monisi1/isaacsim/python.sh     |
| - Workspace directory: /home/user/monisi1/ws_moveit_clean/              |
| - Model assets: /home/user/monisi1/model/                               |
+-------------------------------------------------------------------------+
                                    |
            Bind Mounts:            | Shared Host Network:
            ws_moveit_clean -> /root/ws_moveit
            model           -> /root/model
                                    | localhost:11311 (ROS Master)
                                    v
+-------------------------------------------------------------------------+
| DOCKER CONTAINER: ros_noetic                                            |
| - ROS 1 Noetic Desktop Full                                             |
| - Catkin workspace: /root/ws_moveit/                                    |
| - MoveIt motion planning packages (X_moveit_config, panda_moveit_config)|
| - Grasp pipeline (AnyGrasp inference client, tf_utils, executor)        |
+-------------------------------------------------------------------------+
```

---

## 2. Critical Architectural Invariants

### Invariant 1: Leg DOF Isolation (`_leg_dof_indices`)
- **NEVER** assume the robot articulation has exactly 12 DOFs (`len(self.robot.dof_names) == 12`).
- The long-term objective of this workspace is **mobile manipulation** by mounting the OpenManipulator-X arm (4-6 DOFs) onto `/World/Spot/body`.
- Once mounted, `self.robot.dof_names` will contain $12 + N$ joints.
- **Rule**: All observation gathering (`obs[12:24]`, `obs[24:36]`) and policy action dispatching (`ArticulationAction`) **MUST** filter by `self._leg_dof_indices`. Arm joint indices must remain completely untouched by the RL locomotion policy so MoveIt can actuate them independently.

### Invariant 2: SimulationApp Initialization Race Condition
- **NEVER** pass `"width"` and `"height"` inside `SimulationApp({...})`.
  ```python
  # BAD - Triggers Kit viewport menubar threading race condition (SIGSEGV):
  simulation_app = SimulationApp({"headless": False, "width": 1280, "height": 720})

  # GOOD - Matches NVIDIA standard standalone examples:
  simulation_app = SimulationApp({"headless": args.headless})
  ```
- Specifying custom dimensions causes Kit to fire window resize events while `omni.kit.viewport.menubar.core` is registering setting change subscriptions in a background thread, dereferencing null carb setting pointers (`setting_model.py:120`).

### Invariant 3: Zero-Velocity Stance Locking
- In Isaac Sim flat-terrain RL policies (`spot_policy.pt`), setting command $[0,0,0]$ through the raw neural network does not produce a frozen stand pose; residual limit-cycles cause the robot to drift or trot in place.
- **Rule**: `SpotArmSafePolicyController.forward()` implements an explicit deadband (`np.linalg.norm(cmd) < 0.02`). When within the deadband, residual policy actions decay exponentially (`* 0.9`) and `leg_targets` locks directly to `self._leg_default_pos`.

### Invariant 4: Command Watchdog Timeout
- Default watchdog timeout is **0.3 seconds** (300 ms).
- If the stream of `/cmd_vel` messages stops (e.g. key released or publisher interrupted), the controller zeroes the command within 300 ms, triggering the stance lock.

---

## 3. Package Structure

```
ws_moveit_clean/src/spot_policy_control/
├── CMakeLists.txt                  # Catkin build definition & script installation
├── package.xml                     # ROS 1 package dependencies
├── README.md                       # High-level overview & quick runbook
├── AGENT.md                        # This developer & agent guide
├── INSTRUCTION.md                  # Detailed architecture, math, flowcharts, & reproduction
├── launch/
│   └── spot_isaac.launch           # Launch file for optional teleop node
├── src/spot_policy_control/
│   ├── __init__.py
│   └── spot_policy_controller.py   # SpotArmSafePolicyController (subclass of SpotFlatTerrainPolicy)
└── scripts/
    ├── spot_ros_controller.py      # Main Isaac Sim runner with ROS 1/2 bridge
    ├── spot_teleop.py              # Keyboard teleoperation node (w/a/s/d/q/e)
    └── spot_move_relative.py       # Closed-loop relative displacement CLI client
```

---

## 4. Downstream Mobile Manipulation Roadmap

When implementing arm mounting in future turns:

1. **Spot Chassis Prim**:
   The base chassis of Spot is located at `/World/Spot/body`.
2. **Arm Assets**:
   The OpenManipulator-X model is located at `/home/user/monisi1/model/open_manipulator/` on the host (`/root/model/open_manipulator/` in Docker).
3. **Mounting Strategy**:
   - Reference the arm USD under `/World/Spot/body` with a fixed translation (e.g., $z = +0.15\text{ m}$ on top of the chassis).
   - Alternatively, use a PhysX `FixedJoint` prim connecting `Spot/body` to `open_manipulator/link1`.
4. **Controller Separation**:
   - Locomotion: `SpotArmSafePolicyController` drives the 12 leg joints via `/cmd_vel` or `/spot/move_relative`.
   - Manipulation: MoveIt's trajectory controller (or `trajectory_bridge.py`) drives the arm joints via `/joint_trajectory` without interfering with Spot's legs.
