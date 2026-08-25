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
WP1's job - a teammate's GUI - and doesn't exist in this repo). This
script uses a configurable axis-aligned workspace bounding box in the
camera's optical frame as an explicit stand-in, following the same
pattern AnyGrasp's own USAGE.md documents for workspace filtering.
Replace DEFAULT_OBJECT_BOX (or pass --box-x/--box-y/--box-z) once a
real selection mechanism exists, and check it against your actual scene
first - see the module's __main__ for a --dump-stats mode to inspect
the raw point cloud's extent before picking box bounds blindly.
"""

import numpy as np
import rospy
import sensor_msgs.msg


DEPTH_TOPIC = "/camera/depth/image_raw"
CAMERA_INFO_TOPIC = "/camera/color/camera_info"

MIN_DEPTH = 0.12   # meters; discard anything closer (near-clip/self-body noise)
MAX_DEPTH = 2.0    # meters; discard anything farther (background/no-return)

# Axis-aligned box in the camera's optical frame (x right, y down,
# z forward), meters. Placeholder for WP1's real object selection -
# see module docstring. These defaults are NOT verified against any
# particular scene - check with --dump-stats first.
DEFAULT_OBJECT_BOX = {
    "x": (-0.15, 0.15),
    "y": (-0.15, 0.15),
    "z": (0.1, 0.5),
}


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


def apply_workspace_box(points, box=None):
    """
    Boolean mask: True for points inside the axis-aligned box.
    Placeholder for real object selection - see module docstring.
    """

    if box is None:
        box = DEFAULT_OBJECT_BOX

    xmin, xmax = box["x"]
    ymin, ymax = box["y"]
    zmin, zmax = box["z"]

    return (
        (points[:, 0] >= xmin) & (points[:, 0] <= xmax)
        & (points[:, 1] >= ymin) & (points[:, 1] <= ymax)
        & (points[:, 2] >= zmin) & (points[:, 2] <= zmax)
    )


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


def capture_scene_and_object(scene_path, object_path, box=None,
                              depth_topic=DEPTH_TOPIC,
                              camera_info_topic=CAMERA_INFO_TOPIC):
    """
    Captures one frame, writes scene_path (full point cloud) and
    object_path (points inside `box`) as .pcd files. Returns
    (scene_points, object_points, frame_id) for convenience.
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

    object_mask = apply_workspace_box(scene_points, box)
    # object_points = scene_points[object_mask]
    object_points = scene_points
    
    rospy.loginfo(
        "Captured %d scene points, %d in the object box (frame: %s)",
        scene_points.shape[0], object_points.shape[0], frame_id,
    )

    if object_points.shape[0] == 0:
        rospy.logwarn(
            "Object box selected 0 points - check the box bounds against "
            "where the object actually is in %s (--dump-stats can help).",
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
    parser.add_argument("--box-x", type=float, nargs=2,
                         default=DEFAULT_OBJECT_BOX["x"])
    parser.add_argument("--box-y", type=float, nargs=2,
                         default=DEFAULT_OBJECT_BOX["y"])
    parser.add_argument("--box-z", type=float, nargs=2,
                         default=DEFAULT_OBJECT_BOX["z"])
    parser.add_argument(
        "--dump-stats", action="store_true",
        help="Print the captured cloud's per-axis min/max/percentiles "
             "and exit without writing files - use this FIRST to pick "
             "sane --box-x/--box-y/--box-z bounds for your actual scene.",
    )
    args = parser.parse_args()

    rospy.init_node("live_capture", anonymous=True)

    box = {
        "x": tuple(args.box_x),
        "y": tuple(args.box_y),
        "z": tuple(args.box_z),
    }

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
        args.scene_out, args.object_out, box=box
    )

    rospy.loginfo("Wrote %s and %s", args.scene_out, args.object_out)
