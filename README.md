# Quadruped Inspection & Grasping Pipeline

This repository hosts the end-to-end 3D grasp execution pipeline for the Open Manipulator arm setup, spanning AnyGrasp detection, ROS TF transforms, MoveIt trajectory planning, and Isaac Sim execution.

---

## 1. System Architecture & Flow

```
+------------------+     JSON / Pose     +------------------+
| AnyGrasp Server  | <-----------------> |  grasp_client.py |
| (anygrasp_display|                     +--------+---------+
|    container)    |                              |
+------------------+                              v
                                         +------------------+
                                         |   tf_utils.py    |
                                         +--------+---------+
                                                  |
                                                  v
                                         +------------------+
                                         | grasp_planner.py |
                                         +--------+---------+
                                                  |
                                                  v
+------------------+   /clock & Joints   +------------------+
|    Isaac Sim     | <-----------------> |   executor.py    |
|   (Simulation)   |   trajectory_bridge | (ros_noetic cont)|
+------------------+                     +------------------+
```

---

## 2. Repository & Container Mapping

| Component / Function | Git / Host Location | Live Container Path |
|---|---|---|
| **AnyGrasp Pipeline** | `detection/anygrasp_pipeline/` | `anygrasp_display:/workspace/anygrasp_sdk/anygrasp_pipeline` |
| **ROS Grasp Scripts** | `nav/grasp_pipeline/scripts/` | `ros_noetic:/root/ws_moveit/src/grasp_pipeline/scripts` |
| **Robot Description URDF** | `~/model/open_manipulator/` | `ros_noetic:/root/model/open_manipulator/` |

---

## 3. Pipeline Execution Runbook

### Prerequisites & Server Launch

1. **Start Docker Containers:**
```bash
docker start anygrasp_display ros_noetic
docker ps
```

2. **Launch Isaac Sim & Verify Clock:**
Open your robot stage in Isaac Sim, press **Play**, then verify active stepping:
```bash
docker exec ros_noetic bash -c "source /opt/ros/noetic/setup.bash && rostopic hz /clock"
```

3. **Start AnyGrasp Inference Server:**
```bash
docker exec -d anygrasp_display bash -c \
  "cd /workspace/anygrasp_sdk/anygrasp_pipeline && python3 server.py > /tmp/anygrasp_server.log 2>&1"
```
Verify status after ~5 seconds:
```bash
docker exec anygrasp_display curl -s [http://127.0.0.1:5000/](http://127.0.0.1:5000/)
```

4. **Launch MoveIt Motion Planning:**
In a separate terminal tab:
```bash
docker exec -it ros_noetic bash
source /opt/ros/noetic/setup.bash
source ~/ws_moveit/devel/setup.bash
roslaunch X_moveit_config demo.launch
```

5. **Verify Trajectory Bridge Node:**
In another terminal tab:
```bash
docker exec ros_noetic bash -c "source /opt/ros/noetic/setup.bash && rosnode list"
```

---

### Step-by-Step Pipeline Execution

Enter the ROS container environment and run each stage sequentially:

```bash
docker exec -it ros_noetic bash
source /opt/ros/noetic/setup.bash
source ~/ws_moveit/devel/setup.bash
cd /root/ws_moveit/src/grasp_pipeline/scripts
```

```bash
# Stage 1: Query AnyGrasp server for top-5 candidates
python3 grasp_client.py \
  --scene /root/ws_moveit/src/grasp_pipeline/example_data/scene.pcd \
  --object /root/ws_moveit/src/grasp_pipeline/example_data/object.pcd \
  --top-k 5
```

```bash
# Stage 2: Convert best grasp pose to link1 frame and broadcast TF frame
python3 tf_utils.py \
  --scene /root/ws_moveit/src/grasp_pipeline/example_data/scene.pcd \
  --object /root/ws_moveit/src/grasp_pipeline/example_data/object.pcd
```

```bash
# Stage 3: Compute pre-grasp and grasp motion waypoints
python3 grasp_planner.py \
  --scene /root/ws_moveit/src/grasp_pipeline/example_data/scene.pcd \
  --object /root/ws_moveit/src/grasp_pipeline/example_data/object.pcd
```

```bash
# Stage 4: Execute full pick sequence (Open -> Pre-grasp -> Approach -> Close -> Retreat)
python3 executor.py \
  --scene /root/ws_moveit/src/grasp_pipeline/example_data/scene.pcd \
  --object /root/ws_moveit/src/grasp_pipeline/example_data/object.pcd
```

---

## 4. Diagnostics & Monitoring

Run these health checks in an independent terminal window during testing:

* **Simulation Clock:**
```bash
docker exec ros_noetic bash -c "source /opt/ros/noetic/setup.bash && rostopic hz /clock"
```

* **Current Joint States:**
```bash
docker exec ros_noetic bash -c "source /opt/ros/noetic/setup.bash && rostopic echo -n1 /joint_states"
```

* **AnyGrasp Logs:**
```bash
docker exec anygrasp_display tail -30 /tmp/anygrasp_server.log
```
