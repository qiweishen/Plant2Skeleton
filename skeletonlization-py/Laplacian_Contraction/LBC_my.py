import open3d as o3d
import numpy as np
import robust_laplacian
from scipy.spatial import Delaunay
from scipy.sparse import csr_matrix


if __name__ == "__main__":
    # Load point cloud
    pcd = o3d.io.read_point_cloud("../../preprocessing/example_data/downsampled_50000.xyz")
    point_set = np.asarray(pcd.points)

    # Compute the cotangent laplacian matrix and the mass matrix
    L, M = robust_laplacian.point_cloud_laplacian(point_set, mollify_factor=1e-5, n_neighbors=point_set.shape[0] * 0.012)

    # Compute the contraction and attraction constraint matrices
    WL = robust_laplacian.weight_matrix(L, M, 0.5)
    WH = 1.0

    print("Done!")
