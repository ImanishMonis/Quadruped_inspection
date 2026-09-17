import argparse
import json
import math
import os

import numpy as np

from loader import load_scene, load_object
from mask import compute_object_mask, print_mask_statistics
from detector import AnyGraspDetector
from visualize import visualize_grasp


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--scene", required=True)
    parser.add_argument("--object", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--threshold", type=float, default=0.005)
    parser.add_argument("--visualize", action="store_true")
    parser.add_argument('--gripper_height', type=float, default=0.075, help='Gripper height')
    parser.add_argument(
        "--level-only", action="store_true",
        help="Visualize the same grasp executor.py --level-only would pick: "
             "the best-scoring one whose approach is within --max-tilt-deg of "
             "level. Needs the up direction (see --up-vector).",
    )
    parser.add_argument(
        "--max-tilt-deg", type=float, default=15.0,
        help="Must match what executor.py --max-tilt-deg was given, or the "
             "two sides will pick different grasps.",
    )
    parser.add_argument(
        "--up-vector", type=float, nargs=3, default=None,
        help="'Up' expressed in the camera's own frame. Defaults to reading "
             "<scene>.meta.json, which live_capture.py writes from live TF - "
             "so normally you don't pass this at all.",
    )
    return parser.parse_args()


def load_up_vector(scene_path, explicit=None):
    """
    Resolve the "up" direction in camera-frame coordinates: explicit CLI
    value if given, else the sidecar live_capture.py wrote next to the
    scene cloud. Returns None if neither is available.
    """
    if explicit is not None:
        return np.asarray(explicit, dtype=float)

    meta_path = scene_path + ".meta.json"
    if not os.path.exists(meta_path):
        return None

    with open(meta_path) as f:
        meta = json.load(f)
    up = meta.get("up_vector_in_camera")
    if up is None:
        return None

    print(f"Read up vector from {meta_path}: {up} "
          f"(reference frame: {meta.get('up_reference_frame')})")
    return np.asarray(up, dtype=float)


def select_level_grasp_index(grasps, up_vector, max_tilt_deg):
    """
    Index of the best-scoring grasp whose approach is within max_tilt_deg
    of level, or None.

    Deliberately mirrors grasp_planner.select_level_grasp() on the ROS
    side. That one transforms the approach axis into the world frame and
    checks |approach_world.z| <= sin(tilt); this is the same test rewritten
    as |dot(approach_camera, up_in_camera)| <= sin(tilt), which is
    algebraically identical and needs no ROS/TF here. The approach axis is
    column 0 of the rotation matrix (AnyGrasp's own convention, see
    grasp_detection/USAGE.md Note 2).

    Note the duplication is on purpose: the two containers are deliberately
    independent (AGENT.md), so this can't just import the ROS-side helper.
    If either side's tilt rule changes, change both.
    """
    up = up_vector / np.linalg.norm(up_vector)
    max_vertical = math.sin(math.radians(max_tilt_deg))

    for i in range(len(grasps)):
        approach = grasps[i].rotation_matrix[:, 0]
        if abs(float(np.dot(approach, up))) <= max_vertical:
            return i

    return None


def main():

    args = parse_args()

    # -----------------------------
    # Load point clouds
    # -----------------------------
    scene = load_scene(args.scene)
    obj = load_object(args.object)

    print("=" * 50)
    print("Scene")
    print("=" * 50)
    print(scene.num_points)

    print("=" * 50)
    print("Object")
    print("=" * 50)
    print(obj.num_points)

    # -----------------------------
    # Compute object mask
    # -----------------------------
    mask = compute_object_mask(
        scene.points,
        obj.points,
        threshold=args.threshold,
    )

    print_mask_statistics(mask)

    # -----------------------------
    # Initialize detector
    # -----------------------------
    detector = AnyGraspDetector(
        checkpoint_path=args.checkpoint
    )

    # -----------------------------
    # Run detector
    # -----------------------------
    # Collision detection needs the ENTIRE scene; the object cloud is
    # only used to build the region mask above (AGENT.md #3/#4). This
    # mirrors what server.py actually does in production.
    grasps = detector.predict(
        scene.points,
        collision_detection=True,
        region_mask=mask,
    )

    if grasps is None:
        print("No grasps found.")
        return

    print(f"Predicted grasps: {len(grasps)}")

    # Pick the same grasp the ROS side would act on, so what's shown here
    # matches what actually runs in Isaac Sim:
    #   - default:      grasps[0], exactly what grasp_client.get_best_grasp()
    #                   returns (both sides use this same predict() path)
    #   - --level-only: mirrors grasp_planner.select_level_grasp()
    selected_index = 0

    if args.level_only:
        up_vector = load_up_vector(args.scene, args.up_vector)
        if up_vector is None:
            print(
                "--level-only needs the up direction, but no --up-vector was "
                f"given and no metadata was found at {args.scene}.meta.json. "
                "Re-run live_capture.py (it writes that file from live TF), "
                "or pass --up-vector X Y Z explicitly."
            )
            return

        selected_index = select_level_grasp_index(
            grasps, up_vector, args.max_tilt_deg
        )
        if selected_index is None:
            print(
                f"No grasp within {args.max_tilt_deg} deg of level among "
                f"{len(grasps)} candidates - executor.py --level-only would "
                "reject this capture too. Try a larger --max-tilt-deg."
            )
            return

    selected = grasps[selected_index]
    print("=" * 50)
    print(f"Selected grasp (index {selected_index} of {len(grasps)})")
    print("=" * 50)
    print("score      :", selected.score)
    print("translation:", selected.translation)
    print("rotation   :\n", selected.rotation_matrix)

    if args.visualize:
        # Slice, don't index with a list: GraspGroup supports slicing and
        # single-int indexing, but a list of indices raises IndexError
        # (that's what broke the old filter_parallel_grasps, BUG-18).
        # visualize_grasp() renders the first gripper of whatever it gets.
        visualize_grasp(scene.points, grasps[selected_index:selected_index + 1])


if __name__ == "__main__":
    main()
