import argparse

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
    return parser.parse_args()


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

    best = grasps[0]

    print("=" * 50)
    print("Best grasp")
    print("=" * 50)
    print(best.translation)
    print(best.rotation_matrix)
    print(best.score)

    if args.visualize:
        visualize_grasp(scene.points, grasps)


if __name__ == "__main__":
    main()
