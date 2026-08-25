import open3d as o3d
import numpy as np



def visualize_grasp(
        points,
        grasp_group
):

    cloud = o3d.geometry.PointCloud()

    cloud.points = (
        o3d.utility.Vector3dVector(points)
    )


    geometries = [
        cloud
    ]


    grippers = (
        grasp_group
        .to_open3d_geometry_list()
    )


    geometries.extend(
        grippers[:1]
    )


    o3d.visualization.draw_geometries(
        geometries
    )