#!/usr/bin/env python3
"""
Boston Dynamics Spot ROS Velocity Bridge & Policy Controller for Isaac Sim 4.5.

Subscribes to geometry_msgs/Twist (e.g. /cmd_vel), evaluates the built-in RL flat-terrain
locomotion policy, and applies joint commands to Spot in Isaac Sim.
Also publishes /joint_states, /odom, and TF transforms.
"""

import argparse
import os
import sys
import time
from typing import Tuple, Optional, List
import numpy as np

# Parse CLI arguments prior to SimulationApp startup
parser = argparse.ArgumentParser(description="Spot ROS Policy Controller in Isaac Sim")
parser.add_argument("--headless", action="store_true", default=False, help="Run simulation in headless mode")
parser.add_argument("--ros", type=int, default=1, choices=[1, 2], help="ROS version (1: Noetic, 2: Humble/Jazzy)")
parser.add_argument("--cmd_topic", type=str, default="/cmd_vel", help="Velocity command topic (geometry_msgs/Twist)")
parser.add_argument(
    "--move_relative_topic",
    type=str,
    default="/spot/move_relative",
    help="Relative displacement topic (geometry_msgs/Pose2D)",
)
parser.add_argument("--joint_states_topic", type=str, default="/joint_states", help="Joint state publish topic")
parser.add_argument("--odom_topic", type=str, default="/odom", help="Odometry publish topic")
parser.add_argument("--publish_tf", action="store_true", default=True, help="Broadcast odom -> base_link TF")
parser.add_argument(
    "--watchdog_timeout",
    type=float,
    default=0.3,
    help="Command timeout in seconds (auto-stops Spot when command stream stops, default: 0.3s)",
)
parser.add_argument("--physics_rate", type=float, default=500.0, help="Physics simulation rate (Hz)")
parser.add_argument("--rendering_rate", type=float, default=50.0, help="Rendering & publishing rate (Hz)")
parser.add_argument("--usd_path", type=str, default=None, help="Custom USD path for Spot (optional)")
parser.add_argument("--base_prim", type=str, default="/World/Spot", help="Prim path for Spot articulation")
args, unknown = parser.parse_known_args()

# Add package src to sys.path
script_dir = os.path.dirname(os.path.abspath(__file__))
pkg_src_dir = os.path.abspath(os.path.join(script_dir, "..", "src"))
if pkg_src_dir not in sys.path:
    sys.path.insert(0, pkg_src_dir)

# Initialize Omniverse SimulationApp (use standard config to avoid viewport menubar race condition)
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})

import carb
import omni
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from isaacsim.storage.native import get_assets_root_path
from spot_policy_control.spot_policy_controller import SpotArmSafePolicyController

# Enable the appropriate ROS Bridge extension
if args.ros == 1:
    carb.log_info("Enabling isaacsim.ros1.bridge...")
    enable_extension("isaacsim.ros1.bridge")
else:
    carb.log_info("Enabling isaacsim.ros2.bridge...")
    enable_extension("isaacsim.ros2.bridge")

simulation_app.update()

# Setup ROS interface
if args.ros == 1:
    import rosgraph
    import rospy
    from geometry_msgs.msg import Twist, TransformStamped, Pose2D
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header
    import tf2_ros

    if not rosgraph.is_master_online():
        carb.log_error(
            "ROS Master (roscore) is not online! "
            "Please start roscore or run 'docker start ros_noetic' before running this script."
        )
        simulation_app.close()
        sys.exit(1)

    rospy.init_node("spot_isaac_controller", anonymous=True, disable_signals=True)
    carb.log_info("ROS 1 node 'spot_isaac_controller' initialized.")
else:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, TransformStamped, Pose2D
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header
    from tf2_ros import TransformBroadcaster

    rclpy.init()
    ros2_node = Node("spot_isaac_controller")
    carb.log_info("ROS 2 node 'spot_isaac_controller' initialized.")


class RelativeDisplacementController:
    """
    Closed-loop displacement controller that drives Spot by a commanded
    relative offset (dx_body, dy_body, dtheta) using base pose feedback.
    """

    def __init__(self, spot: SpotArmSafePolicyController):
        self.spot = spot
        self.is_active = False
        self.target_world_pos = np.zeros(2)
        self.target_world_yaw = 0.0
        self.tolerance_pos = 0.03  # 3 cm tolerance
        self.tolerance_yaw = 0.05  # ~2.8 degrees tolerance
        self.max_lin_vel = 0.5  # Max linear speed (m/s)
        self.max_ang_vel = 0.6  # Max angular speed (rad/s)
        self.kp_pos = 1.5
        self.kp_yaw = 1.5

    def set_goal(self, dx_body: float, dy_body: float, dtheta: float):
        pos, quat = self.spot.get_base_world_pose()
        w, x, y, z = quat
        current_yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        cos_y = np.cos(current_yaw)
        sin_y = np.sin(current_yaw)

        # Convert local body offset to global world target
        dx_world = cos_y * dx_body - sin_y * dy_body
        dy_world = sin_y * dx_body + cos_y * dy_body

        self.target_world_pos = np.array([pos[0] + dx_world, pos[1] + dy_world])
        self.target_world_yaw = (current_yaw + dtheta + np.pi) % (2.0 * np.pi) - np.pi
        self.is_active = True
        carb.log_info(
            f"[RelativeMove] New goal: forward={dx_body:+.2f}m, left={dy_body:+.2f}m, turn={np.degrees(dtheta):+.1f} deg"
        )

    def step(self) -> Tuple[np.ndarray, bool]:
        """
        Calculates required velocity commands to reach the target pose.
        Returns: (cmd [vx, vy, wz], is_finished)
        """
        if not self.is_active:
            return np.zeros(3), True

        pos, quat = self.spot.get_base_world_pose()
        w, x, y, z = quat
        current_yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        # Position error in world coordinates
        err_world = self.target_world_pos - pos[:2]
        dist_err = float(np.linalg.norm(err_world))

        # Transform error into current body frame
        cos_y = np.cos(current_yaw)
        sin_y = np.sin(current_yaw)
        err_body_x = float(cos_y * err_world[0] + sin_y * err_world[1])
        err_body_y = float(-sin_y * err_world[0] + cos_y * err_world[1])

        # Heading error in [-pi, pi]
        err_yaw = float((self.target_world_yaw - current_yaw + np.pi) % (2.0 * np.pi) - np.pi)

        # Check termination criteria
        if dist_err <= self.tolerance_pos and abs(err_yaw) <= self.tolerance_yaw:
            self.is_active = False
            carb.log_info("[RelativeMove] Reached target displacement successfully! Holding stance.")
            return np.zeros(3), True

        # Calculate proportional velocity commands
        vx = np.clip(self.kp_pos * err_body_x, -self.max_lin_vel, self.max_lin_vel)
        vy = np.clip(self.kp_pos * err_body_y, -self.max_lin_vel, self.max_lin_vel)
        wz = np.clip(self.kp_yaw * err_yaw, -self.max_ang_vel, self.max_ang_vel)

        if dist_err <= self.tolerance_pos:
            vx = 0.0
            vy = 0.0
        if abs(err_yaw) <= self.tolerance_yaw:
            wz = 0.0

        return np.array([vx, vy, wz]), False

    def cancel(self):
        if self.is_active:
            carb.log_info("[RelativeMove] Relative motion canceled by new direct velocity command.")
            self.is_active = False


class SpotRosBridge:
    def __init__(self, world: World, spot_controller: SpotArmSafePolicyController):
        self.world = world
        self.spot = spot_controller
        self.rel_controller = RelativeDisplacementController(self.spot)
        self.latest_command = np.zeros(3)  # [vx, vy, wz]
        self.last_cmd_time = time.time()
        self.timeout = args.watchdog_timeout

        self.first_step = True
        self.reset_needed = False
        self._was_moving = False

        # ROS publishers and subscribers
        if args.ros == 1:
            self.sub = rospy.Subscriber(args.cmd_topic, Twist, self._cmd_callback_ros1, queue_size=1)
            self.rel_sub = rospy.Subscriber(
                args.move_relative_topic, Pose2D, self._rel_cmd_callback_ros1, queue_size=1
            )
            self.joint_pub = rospy.Publisher(args.joint_states_topic, JointState, queue_size=10)
            self.odom_pub = rospy.Publisher(args.odom_topic, Odometry, queue_size=10)
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        else:
            self.sub = ros2_node.create_subscription(Twist, args.cmd_topic, self._cmd_callback_ros2, 1)
            self.rel_sub = ros2_node.create_subscription(
                Pose2D, args.move_relative_topic, self._rel_cmd_callback_ros2, 1
            )
            self.joint_pub = ros2_node.create_publisher(JointState, args.joint_states_topic, 10)
            self.odom_pub = ros2_node.create_publisher(Odometry, args.odom_topic, 10)
            self.tf_broadcaster = TransformBroadcaster(ros2_node)

        carb.log_info(f"Subscribed to velocity command topic: {args.cmd_topic}")
        carb.log_info(f"Subscribed to relative displacement topic: {args.move_relative_topic}")
        carb.log_info(f"Publishing joint states to: {args.joint_states_topic}")
        carb.log_info(f"Publishing odometry to: {args.odom_topic}")

    def _cmd_callback_ros1(self, msg: Twist):
        # Cancel any active relative displacement command if direct velocity command arrives
        if self.rel_controller.is_active and (
            abs(msg.linear.x) > 0.01 or abs(msg.linear.y) > 0.01 or abs(msg.angular.z) > 0.01
        ):
            self.rel_controller.cancel()
        self.latest_command[0] = msg.linear.x
        self.latest_command[1] = msg.linear.y
        self.latest_command[2] = msg.angular.z
        self.last_cmd_time = time.time()
        carb.log_info(
            f"[SpotRosBridge] Velocity command received: vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, wz={msg.angular.z:.2f}"
        )

    def _cmd_callback_ros2(self, msg: Twist):
        if self.rel_controller.is_active and (
            abs(msg.linear.x) > 0.01 or abs(msg.linear.y) > 0.01 or abs(msg.angular.z) > 0.01
        ):
            self.rel_controller.cancel()
        self.latest_command[0] = msg.linear.x
        self.latest_command[1] = msg.linear.y
        self.latest_command[2] = msg.angular.z
        self.last_cmd_time = time.time()
        carb.log_info(
            f"[SpotRosBridge] Velocity command received: vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, wz={msg.angular.z:.2f}"
        )

    def _rel_cmd_callback_ros1(self, msg: Pose2D):
        self.rel_controller.set_goal(msg.x, msg.y, msg.theta)

    def _rel_cmd_callback_ros2(self, msg: Pose2D):
        self.rel_controller.set_goal(msg.x, msg.y, msg.theta)

    def on_physics_step(self, step_size: float) -> None:
        if self.first_step:
            carb.log_info("[SpotRosBridge] Initializing Spot articulation...")
            self.spot.initialize()
            self.first_step = False
            return

        if self.reset_needed:
            self.world.reset(True)
            self.reset_needed = False
            self.first_step = True
            return

        # 1. If relative displacement controller is active, it computes desired command
        if self.rel_controller.is_active:
            active_cmd, finished = self.rel_controller.step()
            self._was_moving = not finished
        else:
            # 2. Otherwise, check standard velocity command watchdog timeout
            if self.timeout > 0.0 and (time.time() - self.last_cmd_time > self.timeout):
                active_cmd = np.zeros(3)
                if self._was_moving:
                    carb.log_info("[SpotRosBridge] Command stream stopped. Watchdog active: Spot safely stopped and standing.")
                    self._was_moving = False
            else:
                active_cmd = self.latest_command
                if np.linalg.norm(active_cmd) > 0.02:
                    self._was_moving = True

        # Step RL locomotion policy (will hold stance when active_cmd is zero)
        self.spot.forward(step_size, active_cmd)

    def publish_ros_feedback(self) -> None:
        if not self.spot._is_initialized:
            return

        now_sec = time.time()
        # 1. Publish JointState
        names, positions, velocities, efforts = self.spot.get_all_joint_states()

        if args.ros == 1:
            js = JointState()
            js.header.stamp = rospy.Time.now()
            js.name = list(names)
            js.position = positions.tolist() if hasattr(positions, "tolist") else list(positions)
            js.velocity = velocities.tolist() if hasattr(velocities, "tolist") else list(velocities)
            if efforts is not None:
                js.effort = efforts.tolist() if hasattr(efforts, "tolist") else list(efforts)
            self.joint_pub.publish(js)
        else:
            js = JointState()
            js.header.stamp = ros2_node.get_clock().now().to_msg()
            js.name = list(names)
            js.position = [float(p) for p in positions]
            js.velocity = [float(v) for v in velocities]
            if efforts is not None:
                js.effort = [float(e) for e in efforts]
            self.joint_pub.publish(js)

        # 2. Publish Odometry and TF
        world_pos, world_rot = self.spot.get_base_world_pose()  # pos=[x, y, z], rot=[w, x, y, z]
        lin_vel, ang_vel = self.spot.get_base_velocities()

        if args.ros == 1:
            stamp = rospy.Time.now()
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = float(world_pos[0])
            odom.pose.pose.position.y = float(world_pos[1])
            odom.pose.pose.position.z = float(world_pos[2])
            odom.pose.pose.orientation.w = float(world_rot[0])
            odom.pose.pose.orientation.x = float(world_rot[1])
            odom.pose.pose.orientation.y = float(world_rot[2])
            odom.pose.pose.orientation.z = float(world_rot[3])

            odom.twist.twist.linear.x = float(lin_vel[0])
            odom.twist.twist.linear.y = float(lin_vel[1])
            odom.twist.twist.linear.z = float(lin_vel[2])
            odom.twist.twist.angular.x = float(ang_vel[0])
            odom.twist.twist.angular.y = float(ang_vel[1])
            odom.twist.twist.angular.z = float(ang_vel[2])

            self.odom_pub.publish(odom)

            if args.publish_tf:
                t = TransformStamped()
                t.header.stamp = stamp
                t.header.frame_id = "odom"
                t.child_frame_id = "base_link"
                t.transform.translation.x = float(world_pos[0])
                t.transform.translation.y = float(world_pos[1])
                t.transform.translation.z = float(world_pos[2])
                t.transform.rotation.w = float(world_rot[0])
                t.transform.rotation.x = float(world_rot[1])
                t.transform.rotation.y = float(world_rot[2])
                t.transform.rotation.z = float(world_rot[3])
                self.tf_broadcaster.sendTransform(t)
        else:
            stamp = ros2_node.get_clock().now().to_msg()
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = float(world_pos[0])
            odom.pose.pose.position.y = float(world_pos[1])
            odom.pose.pose.position.z = float(world_pos[2])
            odom.pose.pose.orientation.w = float(world_rot[0])
            odom.pose.pose.orientation.x = float(world_rot[1])
            odom.pose.pose.orientation.y = float(world_rot[2])
            odom.pose.pose.orientation.z = float(world_rot[3])

            odom.twist.twist.linear.x = float(lin_vel[0])
            odom.twist.twist.linear.y = float(lin_vel[1])
            odom.twist.twist.linear.z = float(lin_vel[2])
            odom.twist.twist.angular.x = float(ang_vel[0])
            odom.twist.twist.angular.y = float(ang_vel[1])
            odom.twist.twist.angular.z = float(ang_vel[2])

            self.odom_pub.publish(odom)

            if args.publish_tf:
                t = TransformStamped()
                t.header.stamp = stamp
                t.header.frame_id = "odom"
                t.child_frame_id = "base_link"
                t.transform.translation.x = float(world_pos[0])
                t.transform.translation.y = float(world_pos[1])
                t.transform.translation.z = float(world_pos[2])
                t.transform.rotation.w = float(world_rot[0])
                t.transform.rotation.x = float(world_rot[1])
                t.transform.rotation.y = float(world_rot[2])
                t.transform.rotation.z = float(world_rot[3])
                self.tf_broadcaster.sendTransform(t)


def main():
    carb.log_info("[Main] Initializing Isaac Sim simulation context...")
    physics_dt = 1.0 / args.physics_rate
    rendering_dt = 1.0 / args.rendering_rate

    world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=rendering_dt)

    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_warn("Could not find default Isaac Sim assets folder, checking local cache...")

    # Spawn default ground plane
    ground_prim = get_prim_at_path("/World/defaultGroundPlane")
    if not ground_prim.IsValid():
        world.scene.add_default_ground_plane()

    # Determine Spot USD path
    usd_path = args.usd_path
    if usd_path is None and assets_root_path is not None:
        usd_path = assets_root_path + "/Isaac/Robots/BostonDynamics/spot/spot.usd"

    carb.log_info(f"[Main] Spawning Spot from: {usd_path} at prim: {args.base_prim}")
    spot = SpotArmSafePolicyController(
        prim_path=args.base_prim,
        name="Spot",
        usd_path=usd_path,
        position=np.array([0.0, 0.0, 0.8]),
    )

    bridge = SpotRosBridge(world=world, spot_controller=spot)

    world.reset()
    world.add_physics_callback("physics_step", callback_fn=bridge.on_physics_step)
    world.play()

    carb.log_info("[Main] Spot simulation running. Awaiting velocity commands on " + args.cmd_topic)

    render_counter = 0
    render_ratio = max(1, int(args.physics_rate / args.rendering_rate))

    try:
        while simulation_app.is_running():
            world.step(render=True)

            if args.ros == 2:
                rclpy.spin_once(ros2_node, timeout_sec=0.0)

            if world.is_stopped():
                bridge.reset_needed = True

            if world.is_playing():
                render_counter += 1
                if render_counter % render_ratio == 0:
                    bridge.publish_ros_feedback()

    except KeyboardInterrupt:
        carb.log_info("[Main] KeyboardInterrupt received. Shutting down...")
    finally:
        world.stop()
        if args.ros == 2:
            ros2_node.destroy_node()
            rclpy.shutdown()
        simulation_app.close()
        carb.log_info("[Main] Isaac Sim cleanly closed.")


if __name__ == "__main__":
    main()
