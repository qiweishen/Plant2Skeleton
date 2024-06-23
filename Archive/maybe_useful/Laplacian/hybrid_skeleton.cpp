// Take the initial laplacian skeleton (iteration time ≤ 3 ?) as guidance, and use slicing method to follow that
// guidance to get the final skeleton.

// Region growing method is used to get the guidance.
// 1. 使用区域生长法将点云分割成多个区域，每个区域的法向量基本一致，方便定位方向
// 2. 对每个区域，使用最小二乘法拟合直线，得到直线方程，然后沿着直线方向进行切片，得到骨架

#include <omp.h>

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h>

#include <open3d/Open3D.h>
#include <open3d/geometry/KDTreeFlann.h>
#include <open3d/io/PointCloudIO.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>


typedef open3d::geometry::PointCloud PointCloud;
typedef open3d::geometry::KDTreeFlann KDTreeFlann;


void loadXYZFile(std::string& file_path, std::shared_ptr<PointCloud>& cloudPtr) {
    std::ifstream file(file_path);
    std::string suffix = file_path.substr(file_path.find_last_of("."));
    if (!file.is_open()) {
        throw std::runtime_error("No File Found");
    } else if (suffix != ".txt" && suffix != ".xyz" && suffix != ".ply" && suffix != ".pcd") {
        throw std::runtime_error("Only Support .txt .xyz .ply .pcd Format");
    } else if (suffix == ".txt" || suffix == ".xyz"){
        std::vector<Eigen::Vector3d> point_cloud;
        double x, y, z;
        while (file >> x >> y >> z) {
            Eigen::Vector3d point(x, y, z);
            point_cloud.push_back(point);
        }
        file.close();
        cloudPtr->points_ = point_cloud;
    } else if (suffix == ".ply" || suffix == ".pcd") {
        open3d::io::ReadPointCloud(file_path, *cloudPtr);
    }
}




int main() {
    std::string input_file_path = "../skeleton_points_1.pcd";
    std::shared_ptr<PointCloud> cloudPtr(new PointCloud);
    loadXYZFile(input_file_path, cloudPtr);

    // Find the lowest point
    Eigen::Vector3d lowest_point = cloudPtr->points_[0];
    for (auto& point : cloudPtr->points_) {
        if (point(2) < lowest_point(2)) {
            lowest_point = point;
        }
    }

    // Find the nearest point to the lowest point
    KDTreeFlann kdtree;
    kdtree.SetGeometry(*cloudPtr);
    int k = 16;
    std::vector<int> indices;
    std::vector<double> distances;
    kdtree.SearchKNN(lowest_point, k, indices, distances);

}