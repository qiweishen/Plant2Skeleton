#ifndef CPPTEST_SKELETON_H
#define CPPTEST_SKELETON_H


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

#include "geometrycentral/pointcloud/point_cloud.h"
#include "geometrycentral/pointcloud/point_position_geometry.h"
#include "geometrycentral/surface/edge_length_geometry.h"
#include "geometrycentral/surface/halfedge_factories.h"
#include "geometrycentral/surface/intrinsic_mollification.h"
#include "geometrycentral/surface/simple_idt.h"
#include "geometrycentral/surface/tufted_laplacian.h"


struct VectorHash {
    std::size_t operator()(const std::vector<size_t>& v) const {
        std::size_t seed = v.size();
        for (auto& i : v) {
            seed ^= i + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

class Skeleton {
public:
    explicit Skeleton(std::shared_ptr<Eigen::MatrixXd> &cloudPtr, int k_neighbors, double fix_eigen_ratio,
                      double diagonal_length, std::string output_path) :
            cloudPtr_(cloudPtr),
            k_neighbors_(k_neighbors),
            fix_eigen_ratio_(fix_eigen_ratio),
            diagonal_length_(diagonal_length),
            output_path_(std::move(output_path)),
            L_Ptr_(std::make_shared<Eigen::SparseMatrix<double>>(cloudPtr->rows(), cloudPtr->rows())),
            S_Ptr_(std::make_shared<Eigen::MatrixXd>(cloudPtr->rows(), 3)),
            WL_Ptr_(std::make_shared<Eigen::VectorXd>(cloudPtr->rows())),
            WH_Ptr_(std::make_shared<Eigen::VectorXd>(cloudPtr->rows())),
            WS_Ptr_(std::make_shared<Eigen::VectorXd>(cloudPtr->rows())),
            sigmaPtr_(std::make_shared<std::vector<double>>(cloudPtr->rows())),
            smooth_sigmaPtr_(std::make_shared<std::vector<double>>(cloudPtr->rows())) {}

    std::shared_ptr<Eigen::MatrixXd> GetLaplacianSkeleton() {
        std::shared_ptr<Eigen::MatrixXd> contracted_cloudPtr = ContractionIteration();
        return contracted_cloudPtr;
    }

private:
    std::shared_ptr<Eigen::MatrixXd> cloudPtr_;
    unsigned int k_neighbors_;
    double fix_eigen_ratio_;
    double diagonal_length_;
    std::string output_path_;

    // Parameters
    const int max_iteration_time_ = 15;
    double initial_WL_ = 1.0;
    double initial_WH_ = 1.0;
    double sL_ = 1.5;

    int pts_num_ = static_cast<int>(cloudPtr_->rows());
    std::vector<int> tip_indices_;
    double threshold_edge_length_ = -1.0;
    std::vector<double> faces_area_;
    std::shared_ptr<Eigen::SparseMatrix<double>> L_Ptr_;
    std::shared_ptr<Eigen::MatrixXd> S_Ptr_;
    std::shared_ptr<Eigen::VectorXd> WL_Ptr_;
    std::shared_ptr<Eigen::VectorXd> WH_Ptr_;
    std::shared_ptr<Eigen::VectorXd> WS_Ptr_;

    std::shared_ptr<std::vector<double>> sigmaPtr_;
    std::shared_ptr<std::vector<double>> smooth_sigmaPtr_;

    void InitializeParameters(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr);
    void EstablishLaplacianMatrix(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr);
    void FindEdgeThreshold(std::shared_ptr<geometrycentral::pointcloud::PointCloud> &gc_cloudPtr,
                           std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> &gc_geom);
    void CollapseEdges(std::shared_ptr<geometrycentral::pointcloud::PointCloud> &gc_cloudPtr,
                       std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> &gc_geom);
    std::tuple<Eigen::SparseMatrix<double>, Eigen::MatrixXd> LengthConstraint(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr);
    Eigen::MatrixXd LaplacianContraction(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr);
    void GetMeanVertexDualArea(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr);
    void ComputeSigma(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, const std::string &neighborhood);
    void UpdateKNeighbors();
    std::shared_ptr<Eigen::MatrixXd> ContractionIteration();
};


#endif //CPPTEST_SKELETON_H
