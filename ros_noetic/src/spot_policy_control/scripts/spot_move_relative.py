#!/usr/bin/env python3
"""
CLI Tool to command Boston Dynamics Spot to move by a precise relative distance.
Examples:
  python3 spot_move_relative.py --forward 0.5
  python3 spot_move_relative.py --left 0.3
  python3 spot_move_relative.py --forward 0.5 --left 0.3
  python3 spot_move_relative.py --backward 0.2 --turn 90
"""

import argparse
import sys
import math

def main():
    parser = argparse.ArgumentParser(description="Send relative displacement command to Spot")
    parser.add_argument("--forward", "-f", type=float, default=0.0, help="Displacement forward (+m)")
    parser.add_argument("--backward", "-b", type=float, default=0.0, help="Displacement backward (+m)")
    parser.add_argument("--left", "-l", type=float, default=0.0, help="Displacement left (+m)")
    parser.add_argument("--right", "-r", type=float, default=0.0, help="Displacement right (+m)")
    parser.add_argument("--turn", "-t", type=float, default=0.0, help="Turn angle in DEGREES (+counter-clockwise, -clockwise)")
    parser.add_argument("--turn_rad", type=float, default=0.0, help="Turn angle in RADIANS")
    parser.add_argument("--topic", type=str, default="/spot/move_relative", help="Relative displacement topic")
    parser.add_argument("--ros", type=int, default=1, choices=[1, 2], help="ROS version (1 or 2)")
    args = parser.parse_args()

    dx = args.forward - args.backward
    dy = args.left - args.right
    dtheta = args.turn_rad + math.radians(args.turn)

    if abs(dx) < 1e-4 and abs(dy) < 1e-4 and abs(dtheta) < 1e-4:
        print("[WARN] No movement specified! Specify --forward, --left, or --turn.")
        sys.exit(0)

    print(f"[SpotRelativeMove] Sending command to {args.topic}:")
    print(f"  -> Forward displacement (dx): {dx:+.2f} m")
    print(f"  -> Lateral displacement (dy): {dy:+.2f} m (left is positive)")
    print(f"  -> Heading rotation (dtheta): {dtheta:+.2f} rad ({math.degrees(dtheta):+.1f} deg)")

    if args.ros == 1:
        import rospy
        from geometry_msgs.msg import Pose2D

        rospy.init_node("spot_move_relative_cli", anonymous=True)
        pub = rospy.Publisher(args.topic, Pose2D, queue_size=1)
        rospy.sleep(0.3)  # Wait for connection to establish

        msg = Pose2D(x=float(dx), y=float(dy), theta=float(dtheta))
        pub.publish(msg)
        rospy.sleep(0.2)
        print("[SpotRelativeMove] Command published successfully! Spot is executing movement.")
    else:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Pose2D

        rclpy.init()
        node = Node("spot_move_relative_cli")
        pub = node.create_publisher(Pose2D, args.topic, 1)
        node.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.3))

        msg = Pose2D(x=float(dx), y=float(dy), theta=float(dtheta))
        pub.publish(msg)
        node.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.2))
        node.destroy_node()
        rclpy.shutdown()
        print("[SpotRelativeMove] Command published successfully! Spot is executing movement.")

if __name__ == '__main__':
    main()
