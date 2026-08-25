# import open3d as o3d
# import numpy as np

# mesh = o3d.geometry.TriangleMesh.create_cylinder(
#     radius=0.03,
#     height=0.15
# )

# pcd = mesh.sample_points_uniformly(30000)

# colors = np.zeros((len(pcd.points),3))
# colors[:,0] = 1.0       # red
# pcd.colors = o3d.utility.Vector3dVector(colors)

# o3d.io.write_point_cloud("bottle.ply", pcd)

import numpy as np
import open3d as o3d
from pathlib import Path


OUTPUT_DIR = Path("example_data")


def create_table():
    """
    Create a flat table plane.
    """

    x = np.linspace(-0.5, 0.5, 200)
    y = np.linspace(-0.5, 0.5, 200)

    xx, yy = np.meshgrid(x, y)

    zz = np.zeros_like(xx)

    points = np.stack(
        [
            xx.flatten(),
            yy.flatten(),
            zz.flatten()
        ],
        axis=1
    )

    return points


def create_object():
    """
    Create a simple cube object above the table.
    """

    mesh = o3d.geometry.TriangleMesh.create_box(
        width=0.15,
        height=0.15,
        depth=0.15
    )

    mesh.translate(
        [-0.075, -0.075, 0.05]
    )

    pcd = mesh.sample_points_uniformly(
        number_of_points=10000
    )

    return np.asarray(pcd.points)


def save_cloud(points, filename):

    cloud = o3d.geometry.PointCloud()

    cloud.points = o3d.utility.Vector3dVector(
        points
    )

    o3d.io.write_point_cloud(
        str(filename),
        cloud
    )

    print(
        f"Saved {filename}"
    )



def main():

    OUTPUT_DIR.mkdir(
        exist_ok=True
    )

    table = create_table()

    obj = create_object()


    # Scene = table + object
    scene = np.vstack(
        [
            table,
            obj
        ]
    )


    save_cloud(
        scene,
        OUTPUT_DIR / "scene.pcd"
    )

    save_cloud(
        obj,
        OUTPUT_DIR / "object.pcd"
    )


    print()
    print("Scene points:", len(scene))
    print("Object points:", len(obj))



if __name__ == "__main__":
    main()