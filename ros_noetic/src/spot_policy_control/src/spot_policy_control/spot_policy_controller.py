"""
Spot Policy Controller Wrapper for Isaac Sim 4.5.
Inherits from SpotFlatTerrainPolicy and provides:
- Safe joint indexing (isolating the 12 leg DOFs for arm mounting)
- Clamped velocity command processing
- State getters for ROS JointState and Odometry publishing
"""

import fnmatch
from typing import Optional, Tuple, List
import carb
import numpy as np
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.policy.examples.robots.spot import SpotFlatTerrainPolicy


class SpotArmSafePolicyController(SpotFlatTerrainPolicy):
    """
    Enhanced Spot Policy Controller that isolates the 12 quadruped leg joints.
    This guarantees compatibility with mounted robotic arms (such as OpenManipulator-X)
    by ensuring the observation vector and joint actions only touch leg DOFs.
    """

    def __init__(
        self,
        prim_path: str = "/World/Spot",
        root_path: Optional[str] = None,
        name: str = "Spot",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        max_lin_vel_x: float = 1.5,
        max_lin_vel_y: float = 0.8,
        max_ang_vel_z: float = 1.2,
    ) -> None:
        super().__init__(
            prim_path=prim_path,
            root_path=root_path,
            name=name,
            usd_path=usd_path,
            position=position if position is not None else np.array([0.0, 0.0, 0.8]),
            orientation=orientation,
        )

        self.max_lin_vel_x = max_lin_vel_x
        self.max_lin_vel_y = max_lin_vel_y
        self.max_ang_vel_z = max_ang_vel_z

        self._leg_dof_indices: np.ndarray = np.arange(12, dtype=int)
        self._leg_dof_names: List[str] = []
        self._arm_dof_indices: List[int] = []
        self._arm_dof_names: List[str] = []
        self._leg_default_pos: np.ndarray = np.zeros(12)
        self.action: np.ndarray = np.zeros(12)
        self._previous_action: np.ndarray = np.zeros(12)
        self._is_initialized = False

    def initialize(self, *args, **kwargs) -> None:
        """
        Initializes the articulation and resolves leg DOF indices versus
        any mounted arm DOFs.
        """
        super().initialize(*args, **kwargs)

        # Parse actuator joint expressions from policy environment parameters
        actuator_data = self.policy_env_params.get("scene", {}).get("robot", {}).get("actuators", {})
        leg_patterns = []
        for actuator in actuator_data.values():
            exprs = actuator.get("joint_names_expr", [])
            leg_patterns.extend(exprs)

        # Fallback patterns for Boston Dynamics Spot leg joints
        if not leg_patterns:
            leg_patterns = [
                "*fl_hx*", "*fl_hy*", "*fl_kn*",
                "*fr_hx*", "*fr_hy*", "*fr_kn*",
                "*hl_hx*", "*hl_hy*", "*hl_kn*",
                "*hr_hx*", "*hr_hy*", "*hr_kn*",
                "*hip*", "*knee*", "*thigh*", "*calf*"
            ]

        all_dof_names = self.robot.dof_names
        leg_indices = []
        leg_names = []
        arm_indices = []
        arm_names = []

        for idx, dof_name in enumerate(all_dof_names):
            matched = False
            for pattern in leg_patterns:
                glob_pat = pattern.replace(".", "*") + "*"
                if fnmatch.fnmatch(dof_name.lower(), glob_pat.lower()):
                    matched = True
                    break
            if matched:
                leg_indices.append(idx)
                leg_names.append(dof_name)
            else:
                arm_indices.append(idx)
                arm_names.append(dof_name)

        if len(leg_indices) == 12:
            self._leg_dof_indices = np.array(leg_indices, dtype=int)
            self._leg_dof_names = leg_names
            self._arm_dof_indices = arm_indices
            self._arm_dof_names = arm_names
            self._leg_default_pos = np.array(self.default_pos)[self._leg_dof_indices]
            carb.log_info(f"[SpotArmSafe] Identified 12 leg joints: {self._leg_dof_names}")
            if arm_names:
                carb.log_info(f"[SpotArmSafe] Identified {len(arm_names)} arm/auxiliary joints: {arm_names}")
        else:
            carb.log_warn(
                f"[SpotArmSafe] Found {len(leg_indices)} matching leg joints out of {len(all_dof_names)} DOFs. "
                f"Falling back to first 12 DOFs."
            )
            self._leg_dof_indices = np.arange(min(12, len(all_dof_names)), dtype=int)
            self._leg_dof_names = [all_dof_names[i] for i in self._leg_dof_indices]
            self._leg_default_pos = np.array(self.default_pos)[:12]

        self._is_initialized = True

    def clamp_command(self, cmd: np.ndarray) -> np.ndarray:
        """
        Clamps (vx, vy, wz) command to safe policy limits.
        """
        clamped = np.copy(cmd)
        clamped[0] = np.clip(clamped[0], -self.max_lin_vel_x, self.max_lin_vel_x)
        clamped[1] = np.clip(clamped[1], -self.max_lin_vel_y, self.max_lin_vel_y)
        clamped[2] = np.clip(clamped[2], -self.max_ang_vel_z, self.max_ang_vel_z)
        return clamped

    def _compute_observation(self, command: np.ndarray) -> np.ndarray:
        """
        Compute the 48-dim observation vector using strictly the 12 leg joints.
        """
        lin_vel_I = self.robot.get_linear_velocity()
        ang_vel_I = self.robot.get_angular_velocity()
        pos_IB, q_IB = self.robot.get_world_pose()

        R_IB = quat_to_rot_matrix(q_IB)
        R_BI = R_IB.transpose()
        lin_vel_b = np.matmul(R_BI, lin_vel_I)
        ang_vel_b = np.matmul(R_BI, ang_vel_I)
        gravity_b = np.matmul(R_BI, np.array([0.0, 0.0, -1.0]))

        obs = np.zeros(48)
        # 1. Base lin vel in body frame (3)
        obs[:3] = lin_vel_b
        # 2. Base ang vel in body frame (3)
        obs[3:6] = ang_vel_b
        # 3. Projected gravity vector in body frame (3)
        obs[6:9] = gravity_b
        # 4. Commanded velocities (3)
        obs[9:12] = self.clamp_command(command)

        # 5. Leg joint positions relative to default (12)
        all_joint_pos = self.robot.get_joint_positions()
        all_joint_vel = self.robot.get_joint_velocities()

        if len(all_joint_pos) > 12:
            leg_pos = all_joint_pos[self._leg_dof_indices]
            leg_vel = all_joint_vel[self._leg_dof_indices]
        else:
            leg_pos = all_joint_pos
            leg_vel = all_joint_vel

        obs[12:24] = leg_pos - self._leg_default_pos
        # 6. Leg joint velocities (12)
        obs[24:36] = leg_vel
        # 7. Previous action (12)
        obs[36:48] = self._previous_action

        return obs

    def forward(self, dt: float, command: np.ndarray) -> None:
        """
        Advances policy by one physics step and commands only the leg joints.
        When command is zero, safely decelerates and holds the default standing stance.
        """
        clamped_cmd = self.clamp_command(command)
        cmd_norm = np.linalg.norm(clamped_cmd)

        if cmd_norm < 0.02:
            # Stand still: decay any residual policy action smoothly and hold default standing pose
            self._previous_action = self._previous_action * 0.9
            self.action = self.action * 0.9
            if np.max(np.abs(self.action)) < 0.01:
                self.action = np.zeros(12)
                self._previous_action = np.zeros(12)
            leg_targets = self._leg_default_pos + (self.action * self._action_scale)
        else:
            if self._policy_counter % self._decimation == 0:
                obs = self._compute_observation(clamped_cmd)
                self.action = self._compute_action(obs)
                self._previous_action = self.action.copy()

            leg_targets = self._leg_default_pos + (self.action * self._action_scale)

        if len(self.robot.dof_names) > 12:
            # Explicitly apply only to the 12 leg joints, leaving arm joints unaffected
            action = ArticulationAction(
                joint_positions=leg_targets,
                joint_indices=self._leg_dof_indices
            )
        else:
            action = ArticulationAction(joint_positions=leg_targets)

        self.robot.apply_action(action)
        self._policy_counter += 1

    def get_all_joint_states(self) -> Tuple[List[str], np.ndarray, np.ndarray, Optional[np.ndarray]]:
        """
        Returns all DOF names, positions, velocities, and applied efforts.
        """
        names = self.robot.dof_names
        positions = self.robot.get_joint_positions()
        velocities = self.robot.get_joint_velocities()
        efforts = self.robot.get_measured_joint_efforts()
        return names, positions, velocities, efforts

    def get_base_world_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Returns (position, orientation_quat) where position is [x, y, z]
        and orientation_quat is [w, x, y, z].
        """
        return self.robot.get_world_pose()

    def get_base_velocities(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Returns (linear_velocity, angular_velocity) in world frame.
        """
        return self.robot.get_linear_velocity(), self.robot.get_angular_velocity()
