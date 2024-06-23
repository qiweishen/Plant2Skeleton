import torch
import numpy as np
import open3d as o3d

from models.tools import remove_ground


# Numpy version
def slice_points(points: np.ndarray, slice_height: float=0.0025) -> np.ndarray:
    """
    Input:
        points: input points data, np.ndarray(N, C=3)
        slice_height: the height of each slice layer, float
    Return:
        sliced_points: output points data, np.ndarray(slice_num, N, C=3)
    """
    slice_num = int(1.0 / slice_height)
    slice_bounds = np.linspace(0, 1.0, slice_num + 1)

    slices = []
    for i in range(slice_num):
        lower_bound = slice_bounds[i]
        upper_bound = slice_bounds[i + 1]

        mask = (lower_bound <= points[:, 2]) & (points[:, 2] < upper_bound)
        sliced_points = points[mask]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(sliced_points)
        labels = np.array(pcd.cluster_dbscan(eps=0.01, min_points=10, print_progress=False))

        slices.append(sliced_points)

    result = np.vstack(slices)

    return result


def scale_point_cloud_height_to_unit(points: np.ndarray) -> np.ndarray:
    # Assuming the height is along the Y-axis (the second column in the points array)
    min_height = np.min(points[:, 1])
    max_height = np.max(points[:, 1])

    # Calculate the height range
    height_range = max_height - min_height

    # Avoid division by zero in case all points have the same height
    if height_range == 0:
        raise ValueError("All points have the same height; cannot scale to unit height.")

    # Calculate the scale factor for the height to be 1.0 unit
    scale_factor = 1.0 / height_range

    # Scale the entire point cloud to maintain the aspect ratio
    scaled_points = (points - min_height) * scale_factor

    return scaled_points


if __name__ == "__main__":
    point_set = np.loadtxt("../../preprocessing/example_data/downsampled_50000.xyz")
    # point_set = remove_ground(point_set)[:, :3]

    point_set = scale_point_cloud_height_to_unit(point_set)
    np.savetxt("./normalized_data_1.txt", point_set, fmt="%.8f")

    sliced_points = slice_points(point_set)
    np.savetxt("./sliced_data_1.txt", sliced_points, fmt="%.8f")

    print("Finished!")
