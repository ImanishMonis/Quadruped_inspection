# Boston Dynamics Spot Policy & Motion Controller for Isaac Sim

A complete ROS interface and RL locomotion policy controller for the **Boston Dynamics Spot** quadruped in **NVIDIA Isaac Sim (v4.5.0)**, prepared for future robotic arm mounting (e.g. OpenManipulator-X) and MoveIt mobile manipulation.

---

## 1. Key Capabilities

1. **Built-in RL Locomotion Policy**: Evaluates NVIDIA's flat-terrain TorchScript policy (`spot_policy.pt`) at 50 Hz within a 500 Hz PhysX simulation loop.
2. **Velocity Control (`/cmd_vel`)**: Maps incoming `geometry_msgs/Twist` ($v_x, v_y, \omega_z$) to locomotion commands.
3. **Closed-Loop Relative Displacement Control (`/spot/move_relative`)**: Command Spot to move exact relative distances (e.g., *move forward 0.5 m*, *move left 0.3 m*, *turn 90 deg*). Automatically tracks progress via base odometry, decelerates smoothly, and stops at the goal.
4. **Zero-Velocity Stance Locking**: Automatically locks legs into the rock-solid default standing pose whenever velocity is zero (`|cmd| < 0.02 m/s`), completely eliminating residual neural network trotting or drift.
5. **Watchdog Auto-Stop (300 ms)**: Automatically brings Spot to a halt and holds standing stance if the velocity command stream stops for more than 0.3 seconds.
6. **Arm-Mounting Preparedness**: Uses explicit 12-DOF leg indexing (`_leg_dof_indices`). Mounting an arm (OpenManipulator-X) onto Spot's chassis (`/World/Spot/body`) will not crash observation/action tensors, leaving arm joints free for MoveIt.
7. **State Feedback**: Publishes `/joint_states` (`sensor_msgs/JointState`), `/odom` (`nav_msgs/Odometry`), and TF transforms (`odom` -> `base_link`).

---

## 2. Host vs. Docker Execution Model

| Component | Execution Environment | Reason |
|---|---|---|
| **Isaac Sim 4.5 & Simulation Scripts** | **HOST machine** (`pringles`) | Requires direct access to NVIDIA GPU drivers, Vulkan, and Isaac Sim installation (`~/isaacsim`). |
| **ROS Core & MoveIt Nodes** | **DOCKER container** (`ros_noetic`) | Contains ROS 1 Noetic environment, catkin workspace, and MoveIt configurations. |
| **Networking Bridge** | Shared (`localhost:11311`) | Docker uses host networking; Isaac Sim on the host communicates directly with `roscore` in Docker. |

---

## 3. Quick Runbook

### Step 1: Start ROS Master in Docker
```bash
# [HOST terminal]
docker start ros_noetic

# Verify roscore is active:
docker exec -it ros_noetic bash -c "source /opt/ros/noetic/setup.bash && rostopic list"
```

### Step 2: Launch Spot in Isaac Sim (on Host)
```bash
# [HOST terminal]
/home/user/monisi1/isaacsim/python.sh /home/user/monisi1/ws_moveit_clean/src/spot_policy_control/scripts/spot_ros_controller.py
```
*(Add `--headless` if running over an SSH terminal without an active X11/Vulkan display)*.

---

## 4. Commanding Spot

### Option A: Move by Exact Distance (Relative Displacement)
Use the included CLI tool to move Spot by precise offsets:
```bash
# [HOST or DOCKER terminal]
# Move front by 0.5 m:
python3 /home/user/monisi1/ws_moveit_clean/src/spot_policy_control/scripts/spot_move_relative.py --forward 0.5

# Move left by 0.3 m:
python3 /home/user/monisi1/ws_moveit_clean/src/spot_policy_control/scripts/spot_move_relative.py --left 0.3

# Move front 0.5 m AND left 0.3 m simultaneously:
python3 /home/user/monisi1/ws_moveit_clean/src/spot_policy_control/scripts/spot_move_relative.py --forward 0.5 --left 0.3

# Turn 90 degrees counter-clockwise:
python3 /home/user/monisi1/ws_moveit_clean/src/spot_policy_control/scripts/spot_move_relative.py --turn 90
```

Or publish directly to `/spot/move_relative` (`geometry_msgs/Pose2D`):
```bash
# [HOST or DOCKER terminal]
rostopic pub -1 /spot/move_relative geometry_msgs/Pose2D "{x: 0.5, y: 0.0, theta: 0.0}"
rostopic pub -1 /spot/move_relative geometry_msgs/Pose2D "{x: 0.0, y: 0.3, theta: 0.0}"
```

### Option B: Interactive Keyboard Teleoperation
```bash
# [HOST or DOCKER terminal]
python3 /home/user/monisi1/ws_moveit_clean/src/spot_policy_control/scripts/spot_teleop.py
```
- `w` / `x` : Trot forward / backward
- `a` / `d` : Strafe left / right
- `q` / `e` : Turn counter-clockwise / clockwise
- `s` / `SPACE` : Stop immediately

### Option C: Direct Continuous Velocity Topic
```bash
# [HOST or DOCKER terminal]
# Trot forward at 0.5 m/s (press Ctrl+C to stop; Spot stops in 0.3s):
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 5. Detailed Documentation
- See **`INSTRUCTION.md`** for full architecture diagrams, math/logic formulations, and complete reproducibility runbooks.
- See **`AGENT.md`** for codebase conventions, arm-mounting roadmap, and developer guidelines.
