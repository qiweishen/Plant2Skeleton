import numpy as np
import open3d as o3d
import time
from tqdm import tqdm

from preprocessing.Down_Sampling import uniform_voxel_sampling



def compute_weighted_principal_components(points: np.ndarray, h: float, pt: np.ndarray) -> (np.ndarray, np.ndarray):
    # Compute the weighted covariance matrix
    cov_matrix = np.zeros((points.shape[1], points.shape[1]))
    for i in range(points.shape[0]):
        if not np.array_equal(points[i, :], pt):
            theta = np.exp(-(np.linalg.norm(pt - points[i, :])) ** 2 / (h / 2) ** 2)
            diff = pt - points[i, :]
            cov_matrix += theta * np.outer(diff, diff)

    # Compute the eigenvalues and eigenvectors
    eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)

    # Sort the eigenvalues and eigenvectors in ascending order
    idx = np.argsort(eigenvalues)
    eigenvalues = eigenvalues[idx]
    eigenvectors = eigenvectors[:, idx]

    # Normalize eigenvectors to ensure they are unit vectors
    eigenvectors = eigenvectors / np.linalg.norm(eigenvectors, axis=0)

    return eigenvalues, eigenvectors


def compute_sigma(points: np.ndarray, h: float, pt: np.ndarray) -> float:
    eigenvalues, _ = compute_weighted_principal_components(points, h, pt)
    sigma = eigenvalues[2] / (eigenvalues[0] + eigenvalues[1] + eigenvalues[2])

    return sigma


def compute_smooth_sigma_within_neighbours(points: np.ndarray, kd_tree: o3d.geometry.KDTreeFlann, h: float, pt: np.ndarray, k_neighbours: int) -> float:
    [_, idx, _] = kd_tree.search_knn_vector_3d(pt, k_neighbours)
    sigma = 0
    for neighbour in points[idx]:
        eigenvalues, _ = compute_weighted_principal_components(points, h, neighbour)
        sigma += eigenvalues[2] / (eigenvalues[0] + eigenvalues[1] + eigenvalues[2])

    sigma /= k_neighbours

    return sigma


def project_to_dominant_pca_direction(points: np.ndarray) -> np.ndarray:
    # Compute the eigenvalues and eigenvectors
    cov_matrix = np.cov(points, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)

    # Sort the eigenvalues and eigenvectors
    idx = np.argsort(eigenvalues)[::-1]
    eigenvectors = eigenvectors[:, idx]

    # Normalize eigenvectors to ensure they are unit vectors
    eigenvectors = eigenvectors / np.linalg.norm(eigenvectors, axis=0)

    # Move points to the origin
    mean = np.mean(points, axis=0)
    centered_points = points - mean

    # Compute the projection matrix
    projection_matrix = eigenvectors[:, :1]

    # Project the points to the dominant PCA direction
    projected_points = np.dot(centered_points, projection_matrix)

    return projected_points


def compute_A(points: np.ndarray, pt: np.ndarray, h: float) -> float:
    numerator = 0
    denominator = 0
    for i in range(points.shape[0]):
        theta = np.exp(-(np.linalg.norm(pt - points[i, :])) ** 2 / (h / 2) ** 2)
        alpha = theta / (np.linalg.norm(pt - points[i, :]) + 1e-8)
        numerator += alpha * points[i, :]
        denominator += alpha

    A = numerator / denominator

    return A


def compute_R(points: np.ndarray, pt: np.ndarray, u: float, sigma: float, h: float) -> np.ndarray:
    numerator = 0
    denominator = 0
    for i in range(points.shape[0]):
        if not np.array_equal(points[i, :], pt):
            theta = np.exp(-(np.linalg.norm(pt - points[i, :])) ** 2 / (h / 2) ** 2)
            beta = theta / (np.linalg.norm(pt - points[i, :]) ** 2)
            numerator += (pt - points[i, :]) * beta
            denominator += beta

    R = u * sigma * (numerator / denominator)

    return R



def l1_medial_skeleton(points: np.ndarray, u: float=0.35, k_neighbours: int=5, iter_num: int=5) -> np.ndarray:
    """
    Input:
        points: input points data, np.ndarray(N, C=3)
        h: initial neighborhood radius, float
        u: control the globel level of penalty, float
        k_neighbours: the number of neighbours, int
    Return:
        skeleton: skeleton points data, np.ndarray(N, C=3)
    """
    diagonal_length = np.linalg.norm(np.max(points, axis=0) - np.min(points, axis=0))
    h = 2 * diagonal_length / (points.shape[0] ** (1 / 3))

    # Initial sampling， 0.05 is the sampling ratio
    idx = np.random.choice(points.shape[0], int(points.shape[0] * 0.05), replace=True)
    skeleton = points[idx, :]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(skeleton)
    kd_tree = o3d.geometry.KDTreeFlann(pcd)

    while iter_num > 0:
        count = 1

        new_skeleton = []
        for pt in tqdm(skeleton, position=0, desc=f"Iteration time: {count}.1 - Contract points"):
            # Contract points
            sigma = compute_sigma(points, h, pt)
            new_skeleton_point = compute_A(points, pt, h) + compute_R(skeleton, pt, u, sigma, h)
            new_skeleton.append(new_skeleton_point)
        skeleton = np.array(new_skeleton)

        candidate_branch_points = []
        for pt in tqdm(skeleton, position=0, desc=f"Iteration time: {count}.2 - Find Branch points"):
            sigma = compute_smooth_sigma_within_neighbours(skeleton, kd_tree, h, pt, k_neighbours)

            # Filter out the candidate branch points
            if sigma > 0.9:
                candidate_branch_points.append([sigma, pt])
        candidate_branch_points = np.array(candidate_branch_points)

        # Identify the branch points
        branch_points = []
        seed_point = candidate_branch_points[candidate_branch_points[:, 0].argmax()]
        ss = project_to_dominant_pca_direction(candidate_branch_points)



        count += 1
        iter_num -= 1

    return skeleton


if __name__ == "__main__":
    for num in [5000]:
        # Read point cloud from txt file
        point_set = np.loadtxt("../airplane_0352.txt", delimiter=",")
        point_set = point_set[:, :3]

        idx = np.random.choice(point_set.shape[0], num, replace=True)
        point_set = point_set[idx]

        start = time.time()
        skeleton_points = l1_medial_skeleton(points=point_set)
        end = time.time()

        np.savetxt(f"./L1M_outputs/skeleton_{num * 0.05}.txt", skeleton_points, delimiter=" ")