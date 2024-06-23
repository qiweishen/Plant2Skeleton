#ifndef CPPTEST_SKELETON_H
#define CPPTEST_SKELETON_H


#include <omp.h>
#include <memory>
#include <fstream>
#include <utility>
#include <vector>
#include <string>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h>

#include "../deps/geometry-central/include/geometrycentral/pointcloud/point_cloud.h"
#include "../deps/geometry-central/include/geometrycentral/pointcloud/point_position_geometry.h"


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
            WL_Ptr_(std::make_shared<Eigen::VectorXd>(cloudPtr->rows())),
            WH_Ptr_(std::make_shared<Eigen::VectorXd>(cloudPtr->rows())),
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
    double initial_WL_ = 3.0;
    double initial_WH_ = 1.0;
    double sL_ = 1.0;

    int pts_num_ = static_cast<int>(cloudPtr_->rows());
    std::vector<double> faces_area_;
    std::shared_ptr<Eigen::SparseMatrix<double>> L_Ptr_;
    std::shared_ptr<Eigen::VectorXd> WL_Ptr_;
    std::shared_ptr<Eigen::VectorXd> WH_Ptr_;

    std::shared_ptr<std::vector<double>> sigmaPtr_;
    std::shared_ptr<std::vector<double>> smooth_sigmaPtr_;

    void EstablishLaplacianMatrix(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr);
    void InitializeParameters(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr);
    Eigen::MatrixXd LaplacianContraction(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr);
    void ComputeSigma(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, const std::string &neighborhood);
    void UpdateKNeighbors();
    void GetMeanVertexDualArea(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr);
    std::shared_ptr<Eigen::MatrixXd> ContractionIteration();
};


#endif //CPPTEST_SKELETON_H
