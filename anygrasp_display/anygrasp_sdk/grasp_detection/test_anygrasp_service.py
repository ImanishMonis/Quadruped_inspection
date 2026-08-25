import argparse
import numpy as np
import open3d as o3d
import os
from anygrasp_service import AnyGraspService


def load_xyzrgb(filename):
    """
    Load a segmented point cloud stored as

    x y z r g b

    Returns
    -------
    points : (N,3) float32
    colors : (N,3) float32
    """

    # data = np.loadtxt(filename, dtype=np.float32)

    # if data.shape[1] != 6:
    #     raise ValueError(
    #         "Point cloud must contain x y z r g b."
    #     )

    # points = data[:, :3]
    # colors = data[:, 3:6]

    # # convert uint8 colors if necessary
    # if colors.max() > 1.0:
    #     colors /= 255.0

    # return points, colors
    ext = os.path.splitext(filename)[1].lower()

    if ext == ".ply":
        pcd = o3d.io.read_point_cloud(filename)

        if pcd.is_empty():
            raise RuntimeError(f"Failed to load {filename}")

        points = np.asarray(pcd.points, dtype=np.float32)

        if pcd.has_colors():
            colors = np.asarray(pcd.colors, dtype=np.float32)
        else:
            colors = np.ones_like(points, dtype=np.float32)

        return points, colors

    # Otherwise assume xyzrgb text
    data = np.loadtxt(filename, dtype=np.float32)

    if data.shape[1] != 6:
        raise ValueError("Text point cloud must contain x y z r g b")

    points = data[:, :3]
    colors = data[:, 3:6]

    if colors.max() > 1:
        colors /= 255.0

    return points, colors


def visualize(points, colors, grasp):
    """
    Visualize the point cloud and the best grasp.
    """

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    cloud.colors = o3d.utility.Vector3dVector(colors)

    gripper = grasp.to_open3d_geometry()

    o3d.visualization.draw_geometries(
        [cloud, gripper],
        window_name="AnyGrasp Result"
    )


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--pointcloud",
        required=True,
        help="Path to xyzrgb point cloud"
    )

    parser.add_argument(
        "--checkpoint",
        required=True,
        help="checkpoint_detection.tar"
    )

    parser.add_argument(
        "--vis",
        action="store_true"
    )

    args = parser.parse_args()

    print("Loading point cloud...")
    points, colors = load_xyzrgb(args.pointcloud)

    print(f"{len(points)} points loaded.")

    detector = AnyGraspService(
        checkpoint_path=args.checkpoint
    )

    print("Running AnyGrasp...")

    grasp = detector.best_grasp(
        points,
        colors
    )

    if grasp is None:
        print("No grasp found.")
        return

    print("\nBest grasp")
    print("--------------------------")
    print("Score :", grasp.score)
    print("Width :", grasp.width)
    print("Translation :")
    print(grasp.translation)
    print("Rotation :")
    print(grasp.rotation_matrix)

    if args.vis:
        visualize(points, colors, grasp)


if __name__ == "__main__":
    main()