#!/usr/bin/env python3
"""
Capture a live point cloud from Isaac Sim's simulated RealSense feed,
instead of relying on canned example_data (see AGENT_SESSION.md /
END_TO_END_TESTING.md section 7 for why that caused a reachability
mismatch: camera_link is wrist-mounted, so a canned cloud captured at
an unrelated pose doesn't mean anything once converted through the
CURRENT arm pose - with live capture, the capture pose and the TF
lookup pose are the same camera, so that mismatch disappears).

Deprojection follows the standard pinhole model, matching AnyGrasp's
own documented approach in grasp_detection/USAGE.md:

    depth_m = depth_image / depth_scale
    X = (u - cx) * depth_m / fx
    Y = (v - cy) * depth_m / fy
    Z = depth_m

Isaac Sim's simulated depth is 32FC1 (already in meters, depth_scale=1
- verified against the live topic before writing this).

Object mask: there is no real object-selection mechanism here (that's
WP1's job - a teammate's GUI - and doesn't exist in this repo). Previously
this used a configurable axis-aligned workspace bounding box (x/y/z) in
the camera's optical frame as an explicit stand-in. Replaced 2026-09-17
with a plain depth-range (Z only) filter, per the user's request - anything
between OBJECT_MIN_DEPTH and OBJECT_MAX_DEPTH counts as "object", full
stop, no X/Y windowing. Still just a placeholder for WP1's real selection,
now a simpler one.

Note on sparsity (found live, 2026-09-17): if the captured cloud looks far
too sparse regardless of this filter, check --dump-stats's raw point
count and the underlying depth image's finite-pixel mean BEFORE assuming
this script's logic is at fault. A live diagnostic that day found the
raw depth image was ~87% finite, but the finite mean was ~0.01m -
essentially identical to MIN_DEPTH's near-clip cutoff - meaning the vast
majority of "finite" pixels were near-clip/no-hit noise (almost certainly
the robot's own gripper/arm filling most of the frame), correctly
discarded by MIN_DEPTH, leaving very few genuine scene points. That's an
observe-pose/framing problem (matches session 3's near-identical incident,
see AGENT_SESSION.md), not a bug in the deprojection or filtering here -
narrowing the depth range further cannot manufacture points that were
never captured. Reposition the arm/camera for real clearance first.
"""

import numpy as np
import rospy
import sensor_msgs.msg


DEPTH_TOPIC = "/camera/depth/image_raw"
CAMERA_INFO_TOPIC = "/camera/color/camera_info"

MIN_DEPTH = 0.12   # meters; discard anything closer (near-clip/self-body noise)
MAX_DEPTH = 2.0    # meters; discard anything farther (background/no-return)

# Depth range (Z only, camera optical frame) counted as "object" -
# replaces the old axis-aligned box. Still a placeholder for WP1's real
# object selection - see module docstring.
OBJECT_MIN_DEPTH = 0.10  # meters
OBJECT_MAX_DEPTH = 1.0   # meters


def get_depth_and_intrinsics(depth_topic=DEPTH_TOPIC,
                              camera_info_topic=CAMERA_INFO_TOPIC,
                              timeout=5.0):
    """
    Grabs one depth frame + matching intrinsics.

    Returns
    -------
    depth : np.ndarray, shape (H, W), float32, meters
    intrinsics : dict with fx, fy, cx, cy
    frame_id : str
    """

    depth_msg = rospy.wait_for_message(
        depth_topic, sensor_msgs.msg.Image, timeout=timeout
    )
    info_msg = rospy.wait_for_message(
        camera_info_topic, sensor_msgs.msg.CameraInfo, timeout=timeout
    )

    if depth_msg.encoding != "32FC1":
        raise RuntimeError(
            f"Expected 32FC1 depth encoding, got {depth_msg.encoding!r} "
            "- deprojection assumes depth values are already in meters."
        )

    depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(
        depth_msg.height, depth_msg.width
    )

    fx, _, cx, _, fy, cy, _, _, _ = info_msg.K

    intrinsics = {"fx": fx, "fy": fy, "cx": cx, "cy": cy}

    return depth, intrinsics, depth_msg.header.frame_id


def deproject(depth, intrinsics, min_depth=MIN_DEPTH, max_depth=MAX_DEPTH):
    """
    Pinhole deprojection: depth image -> (N,3) point cloud in the
    camera's optical frame.
    """

    valid = np.isfinite(depth) & (depth > min_depth) & (depth < max_depth)

    v_idx, u_idx = np.nonzero(valid)
    z = depth[valid]

    x = (u_idx - intrinsics["cx"]) * z / intrinsics["fx"]
    y = (v_idx - intrinsics["cy"]) * z / intrinsics["fy"]

    return np.stack([x, y, z], axis=1).astype(np.float32)


def apply_depth_range(points, min_depth=OBJECT_MIN_DEPTH, max_depth=OBJECT_MAX_DEPTH):
    """
    Boolean mask: True for points whose Z (camera-forward depth) falls
    within [min_depth, max_depth]. Placeholder for real object selection -
    see module docstring. Replaces the old axis-aligned X/Y/Z box.
    """

    return (points[:, 2] >= min_depth) & (points[:, 2] <= max_depth)


def write_pcd(path, points):
    """
    Minimal ASCII .pcd writer (XYZ only) - no Open3D/PCL dependency
    needed on the ROS side, and Open3D (used by loader.py in the
    AnyGrasp container) reads this format back fine.
    """

    n = points.shape[0]

    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA ascii\n"
    )

    with open(path, "w") as f:
        f.write(header)
        for x, y, z in points:
            f.write(f"{x} {y} {z}\n")


def capture_scene_and_object(scene_path, object_path,
                              min_depth=OBJECT_MIN_DEPTH,
                              max_depth=OBJECT_MAX_DEPTH,
                              depth_topic=DEPTH_TOPIC,
                              camera_info_topic=CAMERA_INFO_TOPIC):
    """
    Captures one frame, writes scene_path (full point cloud) and
    object_path (points with min_depth <= Z <= max_depth) as .pcd files.
    Returns (scene_points, object_points, frame_id) for convenience.
    """

    depth, intrinsics, frame_id = get_depth_and_intrinsics(
        depth_topic, camera_info_topic
    )

    scene_points = deproject(depth, intrinsics)

    if scene_points.shape[0] == 0:
        raise RuntimeError(
            "No valid depth points captured - is Isaac Sim playing, and "
            "is anything within MIN_DEPTH/MAX_DEPTH of the camera?"
        )

    # Fixed 2026-09-17: this used to compute object_mask and then
    # immediately discard it (`object_points = scene_points` unconditionally,
    # with the masked line commented out) - object.pcd had been a plain
    # copy of scene.pcd this whole time, box or no box. Now actually applied.
    object_mask = apply_depth_range(scene_points, min_depth, max_depth)
    object_points = scene_points[object_mask]

    rospy.loginfo(
        "Captured %d scene points, %d in depth range [%.2f, %.2f]m (frame: %s)",
        scene_points.shape[0], object_points.shape[0], min_depth, max_depth, frame_id,
    )

    if object_points.shape[0] == 0:
        rospy.logwarn(
            "Depth-range filter selected 0 points - check min_depth/"
            "max_depth against where the object actually is in %s "
            "(--dump-stats can help).",
            frame_id,
        )

    write_pcd(scene_path, scene_points)
    write_pcd(object_path, object_points)

    return scene_points, object_points, frame_id


if __name__ == "__main__":

    import argparse

    parser = argparse.ArgumentParser(
        description="Capture scene.pcd/object.pcd from Isaac Sim's live "
                    "simulated camera instead of canned example_data."
    )
    parser.add_argument("--scene-out", default="/tmp/live_scene.pcd")
    parser.add_argument("--object-out", default="/tmp/live_object.pcd")
    parser.add_argument("--object-min-depth", type=float,
                         default=OBJECT_MIN_DEPTH)
    parser.add_argument("--object-max-depth", type=float,
                         default=OBJECT_MAX_DEPTH)
    parser.add_argument(
        "--dump-stats", action="store_true",
        help="Print the captured cloud's per-axis min/max/percentiles "
             "and exit without writing files - use this FIRST to check "
             "the raw point count/extent before assuming a sparse result "
             "is a bug rather than a framing/observe-pose problem, and to "
             "pick sane --object-min-depth/--object-max-depth for your "
             "actual scene.",
    )
    args = parser.parse_args()

    rospy.init_node("live_capture", anonymous=True)

    if args.dump_stats:
        depth, intrinsics, frame_id = get_depth_and_intrinsics()
        points = deproject(depth, intrinsics)
        print(f"frame_id: {frame_id}")
        print(f"point count: {points.shape[0]}")
        for i, axis in enumerate("xyz"):
            col = points[:, i]
            pct = np.percentile(col, [1, 5, 25, 50, 75, 95, 99])
            print(f"{axis}: min={col.min():.3f} max={col.max():.3f} "
                  f"percentiles(1/5/25/50/75/95/99)={pct}")
        raise SystemExit(0)

    capture_scene_and_object(
        args.scene_out, args.object_out,
        min_depth=args.object_min_depth, max_depth=args.object_max_depth,
    )

    rospy.loginfo("Wrote %s and %s", args.scene_out, args.object_out)
