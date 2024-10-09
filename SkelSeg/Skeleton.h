#ifndef SKELETON_H
#define SKELETON_H


#include "utility/logger/logger.h"
#include "deps/nlohmann/json.hpp"

#include <omp.h>
#include <memory>
#include <fstream>
#include <utility>
#include <vector>
#include <string>
#include <chrono>
#include <unordered_set>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h>

#include <geometrycentral/pointcloud/point_cloud.h>
#include <geometrycentral/pointcloud/point_position_geometry.h>
#include <geometrycentral/surface/edge_length_geometry.h>
#include <geometrycentral/surface/halfedge_factories.h>
#include <geometrycentral/surface/intrinsic_mollification.h>
#include <geometrycentral/surface/simple_idt.h>
#include <geometrycentral/surface/tufted_laplacian.h>
#include <geometrycentral/pointcloud/point_cloud_heat_solver.h>



struct VectorHash {
    std::size_t operator()(const std::vector<size_t> &v) const {
        std::size_t seed = v.size();
        for (auto &i: v) {
            seed ^= i + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

class Skeleton {
public:
    explicit Skeleton(std::shared_ptr<Eigen::MatrixXd> &cloudPtr, nlohmann::json &config) :
            cloudPtr_(cloudPtr),
            config_(config),
            L_Ptr_(std::make_shared<Eigen::SparseMatrix<double>>(cloudPtr->rows(), cloudPtr->rows())),
            WL_Ptr_(std::make_shared<Eigen::VectorXd>(cloudPtr->rows())),
            WH_Ptr_(std::make_shared<Eigen::VectorXd>(cloudPtr->rows())),
            sigmaPtr_(std::make_shared<std::vector<double>>(cloudPtr->rows())),
            smooth_sigmaPtr_(std::make_shared<std::vector<double>>(cloudPtr->rows())) {}

    std::shared_ptr<Eigen::MatrixXd> GetLaplacianSkeleton() {
        MyLogger.Log("--------------------------------------------------", 0, true, false);
        MyLogger.Log("Start Skeleton extraction", 0, true, true);
        std::shared_ptr<Eigen::MatrixXd> contracted_cloudPtr = ContractionIteration();
        return contracted_cloudPtr;
    }

private:
    std::shared_ptr<Eigen::MatrixXd> cloudPtr_;
    nlohmann::json config_;

    int pts_num_ = static_cast<int>(cloudPtr_->rows());
    std::vector<int> tip_indices_;
    double threshold_edge_length_ = -1.0;
    std::vector<double> faces_area_;
    std::shared_ptr<Eigen::SparseMatrix<double>> L_Ptr_;
    std::shared_ptr<Eigen::VectorXd> WL_Ptr_;
    std::shared_ptr<Eigen::VectorXd> WH_Ptr_;
    std::shared_ptr<std::vector<double>> sigmaPtr_;
    std::shared_ptr<std::vector<double>> smooth_sigmaPtr_;

    // Hyper-parameters
    // Do not recommend to change the following parameters
    const double initial_WL_ = 1.0;
    const double initial_WH_ = 1.0;
    const double sL_ = 3.0;
    const double tip_point_WH_ = 9.0;
    const double contracted_point_WL_ = 0.0;
    const double contracted_point_WH_ = 3.0;
    const double max_WL_ = 9.0;
    const int k_neighbors_distance_constraint_ = 4;
    const double WD_ = 9.0;
    const int k_neighbors_dual_area_ = 8;

    // Recommend to change the following parameters in the ../configue.json file
    const double diagonal_length_ = config_["Preprocess"]["Normalize_Diagonal_Length"].get<double>();
    int k_neighbors_ = config_["Constraint_Laplacian_Operator"]["Initial_k"].get<int>();
    const int delta_k_ = config_["Constraint_Laplacian_Operator"]["Delta_k"].get<int>();
    const int max_k_ = config_["Constraint_Laplacian_Operator"]["Max_k"].get<int>();
    const double fix_eigen_ratio_ = config_["Adaptive_Contraction"]["Smooth_Sigma_Threshold"].get<double>();
    const double sigma_radius_ =
            config_["Adaptive_Contraction"]["Sigma_Sphere_Radius_Ratio"].get<double>() * diagonal_length_;
    const int sigma_k_ = config_["Adaptive_Contraction"]["Sigma_KNN_Search"].get<int>();
    const double max_distance_ = config_["Adaptive_Contraction"]["Max_Distance_Ratio"].get<double>() * diagonal_length_;
    const int max_iteration_time_ = config_["Terminate_Condition"]["Max_Iteration"].get<int>();
    const double contraction_threshold_ = config_["Terminate_Condition"]["Convergence_Threshold"].get<double>();
    const std::filesystem::path output_path_ = config_["Output_Settings"]["Output_Folder_Path"].get<std::filesystem::path>();
    const bool binary_format_ = config_["Output_Settings"]["Output_File_DataFormat"].get<std::string>() == "Binary";


    void InitializeParameters(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr);

    void EstablishLaplacianMatrix(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr);

    void FindTipPoints(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr,
                       const std::shared_ptr<geometrycentral::pointcloud::PointCloud> &gc_cloudPtr,
                       const std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> &gc_geom);

    void FindEdgeThreshold(const std::shared_ptr<geometrycentral::pointcloud::PointCloud> &gc_cloudPtr,
                           const std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> &gc_geom);

    void CollapseEdges(std::shared_ptr<geometrycentral::pointcloud::PointCloud> &gc_cloudPtr,
                       std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> &gc_geom);

    std::tuple<Eigen::SparseMatrix<double>, Eigen::MatrixXd>
    DistanceConstraint(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr);

    Eigen::MatrixXd LaplacianContraction(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr);

    void GetMeanVertexDualArea(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr);

    void ComputeSigma(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, const std::string &neighborhood);

    void UpdateKNeighbors();

    std::shared_ptr<Eigen::MatrixXd> ContractionIteration();
};



#endif // SKELETON_H
