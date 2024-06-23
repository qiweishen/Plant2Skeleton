#include "../../../src/GeometryCentral/Skeleton.h"
#include "../../../src/GeometryCentral/Tools.h"

#include <omp.h>
#include <memory>
#include <iostream>
#include <fstream>

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h>

#include <open3d/Open3D.h>
#include <open3d/geometry/KDTreeFlann.h>
#include <open3d/io/PointCloudIO.h>
#include <open3d/visualization/visualizer/Visualizer.h>

typedef open3d::geometry::PointCloud PointCloud;
typedef open3d::geometry::KDTreeFlann KDTreeFlann;
typedef open3d::visualization::Visualizer Visualizer;






double ComputeSigma(const int& point_index,
                    const std::shared_ptr<PointCloud>& cloudPtr,
                    const std::shared_ptr<KDTreeFlann>& kdtreePtr,
                    const int& k) {
    std::vector<int> indices;
    std::vector<double> distances;
    kdtreePtr->SearchKNN(cloudPtr->points_[point_index], k, indices, distances);
    std::shared_ptr<PointCloud> neighbor_cloudPtr(new PointCloud);
    for (const int& index : indices) {
        neighbor_cloudPtr->points_.push_back(cloudPtr->points_[index]);
    }
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    std::tie(mean, covariance) = neighbor_cloudPtr->ComputeMeanAndCovariance();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
    Eigen::Matrix3d eigen_vectors = eigen_solver.eigenvectors(); // The columns are inverted, col(2).value > col(1).value > col(0).value
    Eigen::Vector3d eigen_values = eigen_solver.eigenvalues();
    double lambda_0 = eigen_values(0);
    double lambda_1 = eigen_values(1);
    double lambda_2 = eigen_values(2);
    double sigma = lambda_2 / (lambda_0 + lambda_1 + lambda_2);
    return sigma;
}


double ComputeSmoothSigma(const int& point_index,
                          const std::shared_ptr<PointCloud>& cloudPtr,
                          const std::shared_ptr<KDTreeFlann>& kdtreePtr,
                          const int& k,
                          const std::vector<double>& sigmas) {
    std::vector<int> indices;
    std::vector<double> distances;
    kdtreePtr->SearchKNN(cloudPtr->points_[point_index], k, indices, distances);
    double smooth_sigma = 0;
    for (const int& index : indices) {
        smooth_sigma += sigmas[index];
    }
    smooth_sigma /= k;
    return smooth_sigma;
}


int main() {
    std::string file_path = "../data/C20-1_v4476.xyz";
    std::shared_ptr<PointCloud> cloudPtr(new PointCloud);
    LoadPointCloud(file_path, cloudPtr);

    Skeleton skeleton(cloudPtr, 8, 0.01);
    double diagonal_length = skeleton.GetDiagonal();
    std::shared_ptr<PointCloud> contracted_cloudPtr = skeleton.LaplacianSkeleton();

    std::shared_ptr<KDTreeFlann> kdtree(new KDTreeFlann(*contracted_cloudPtr));
    std::vector<double> sigmas(contracted_cloudPtr->points_.size(), 0);
    for (int i = 0; i < contracted_cloudPtr->points_.size(); i++) {
        sigmas[i] = ComputeSigma(i, contracted_cloudPtr, kdtree, 8);
    }

    std::vector<double> smooth_sigmas(contracted_cloudPtr->points_.size(), 0);
    for (int i = 0; i < contracted_cloudPtr->points_.size(); i++) {
        smooth_sigmas[i] = ComputeSmoothSigma(i, contracted_cloudPtr, kdtree, 16, sigmas);
    }

    std::ofstream file("../data/Output/Separate_Contraction/C20-1_v4476_step1.xyz");
    for (int i = 0; i < contracted_cloudPtr->points_.size(); i++) {
        file << contracted_cloudPtr->points_[i](0) << " " << contracted_cloudPtr->points_[i](1) << " " << contracted_cloudPtr->points_[i](2) << " " << smooth_sigmas[i] << std::endl;
    }
    file.close();


// Separate the initial skeleton and contract them independently.
    std::shared_ptr<PointCloud> potential_joint_cloudPtr(new PointCloud);
    std::shared_ptr<PointCloud> non_joint_cloudPtr(new PointCloud);
    for (int i = 0; i < cloudPtr->points_.size(); i++) {
        if (smooth_sigmas[i] < 0.9) {
            potential_joint_cloudPtr->points_.push_back(contracted_cloudPtr->points_[i]);
        } else {
            non_joint_cloudPtr->points_.push_back(contracted_cloudPtr->points_[i]);
        }
    }


    std::vector<int> clusters = potential_joint_cloudPtr->ClusterDBSCAN(0.02 * diagonal_length, 16);
    // Map to hold the separated clusters
    std::unordered_map<int, std::shared_ptr<PointCloud>> separated_clusters;
    std::shared_ptr<PointCloud> noise_cloudPtr(new PointCloud);
    // Iterate over the cluster indices to group point indices by cluster
    for (size_t i = 0; i < clusters.size(); ++i) {
        int cluster_id = clusters[i];
        if (cluster_id != -1) {
            if (separated_clusters.find(cluster_id) == separated_clusters.end()) {
                // Create a new point cloud for this cluster
                separated_clusters[cluster_id] = std::make_shared<PointCloud>();
            }
            // Add point to the corresponding cluster point cloud
            separated_clusters[cluster_id]->points_.push_back(potential_joint_cloudPtr->points_[i]);
        } else {
            noise_cloudPtr->points_.push_back(potential_joint_cloudPtr->points_[i]);
        }
    }



    std::shared_ptr<PointCloud> again_contracted_cloudPtr(new PointCloud);
    // Continue contraction
    for (const auto& cluster : separated_clusters) {
        std::shared_ptr<PointCloud> cloudPtr = cluster.second;
        std::shared_ptr<Skeleton> skeleton(new Skeleton(cloudPtr, 8, 0.005));
        std::shared_ptr<PointCloud> contracted_cloudPtr = skeleton->LaplacianSkeleton();
        *again_contracted_cloudPtr += *contracted_cloudPtr;
    }
    *again_contracted_cloudPtr += *noise_cloudPtr;
    *again_contracted_cloudPtr += *non_joint_cloudPtr;
//    again_contracted_cloudPtr->RemoveStatisticalOutliers(64, 2.0);



//    // SearchRadius to group the contracted points
//    std::vector<int> belongings(again_contracted_cloudPtr->points_.size(), -1);
//    std::unordered_map<int, std::vector<int>> radius_clusters;
//    double radius = 0.01 * diagonal_length;
//    KDTreeFlann kdtreee(*again_contracted_cloudPtr);
//    std::vector<int> indices;
//    std::vector<double> distances;
//    for (int i = 0; i < again_contracted_cloudPtr->points_.size(); i++) {
//        if (belongings[i] == -1) {
//            kdtreee.SearchRadius(again_contracted_cloudPtr->points_[i], radius, indices, distances);
//            for (int& j : indices) {
//                if (belongings[j] == -1) {
//                    belongings[j] = i;
//                    radius_clusters[i].push_back(j);
//                } else {
//                    continue;
//                }
//            }
//        } else {
//            continue;
//        }
//    }
//    std::shared_ptr<PointCloud> final_cloudPtr(new PointCloud);
//    for (const auto& cluster : radius_clusters) {
//        std::shared_ptr<PointCloud> cloudPtr(new PointCloud);
//        cloudPtr->points_.push_back(again_contracted_cloudPtr->points_[cluster.first]);
//        for (const int& index : cluster.second) {
//            cloudPtr->points_.push_back(again_contracted_cloudPtr->points_[index]);
//        }
//        cloudPtr->RemoveDuplicatedPoints();
//        Eigen::Vector3d mean = cloudPtr->GetCenter();
//        final_cloudPtr->points_.push_back(mean);
//    }


    WritePLYFile<PointCloud>("../data/Output/Separate_Contraction/C20-1_v4476_step2.ply", potential_joint_cloudPtr, Eigen::Vector3d(0.0, 1.0, 0.0));
    WritePLYFile<PointCloud>("../data/Output/Separate_Contraction/C20-1_v4476_step3.ply", again_contracted_cloudPtr, Eigen::Vector3d(0.0, 0.0, 1.0));
//    WritePLYFile<PointCloud>("../data/Output/Separate_Contraction/C20-1_v17297_step4.ply", final_cloudPtr, Eigen::Vector3d(1.0, 0.0, 0.0));








    Visualizer viewer;
    viewer.CreateVisualizerWindow("Open3D", 2400, 1200);
//    cloudPtr->colors_ = std::vector<Eigen::Vector3d>(cloudPtr->points_.size(), Eigen::Vector3d(0, 0, 0));
//    viewer.AddGeometry(cloudPtr);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    for (const auto& cluster : separated_clusters) {
        Eigen::Vector3d random_color(dis(gen), dis(gen), dis(gen));
        cluster.second->colors_ = std::vector<Eigen::Vector3d>(cluster.second->points_.size(), random_color);
        viewer.AddGeometry(cluster.second);
    }
    viewer.Run();


    return EXIT_SUCCESS;
}