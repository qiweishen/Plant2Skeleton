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

#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/KDTreeFlann.h>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>



typedef open3d::geometry::PointCloud PointCloud;
typedef open3d::geometry::KDTreeFlann KDTreeFlann;

typedef CGAL::Exact_predicates_exact_constructions_kernel CGAL_K;
typedef CGAL::Delaunay_triangulation_2<CGAL_K> CGAL_Delaunay_2;
typedef CGAL_K::Point_2 CGAL_Point_2;
typedef CGAL_Delaunay_2::Vertex_handle CGAL_Vertex_handle_2;
typedef CGAL_Delaunay_2::Face_circulator CGAL_Face_circulator_2;





struct CGAL_Point_Hash {
    std::size_t operator()(const CGAL_Point_2& p) const {
        auto hash1 = std::hash<double>{}(CGAL::to_double(p.x()));
        auto hash2 = std::hash<double>{}(CGAL::to_double(p.y()));
        return hash1 ^ (hash2 << 1);  // Combine the two hash values
    }
};


struct Ring {
    int id{};
    std::vector<int> neighbors;
};


class Point {
public:
    Point() = default;
    explicit Point(int point_id, std::vector<Ring>& ring) : point_id_(point_id), ring_(ring) {}

    int get_id() const { return point_id_; }
    std::vector<Ring> get_ring() const { return ring_; }

private:
    int point_id_{};
    std::vector<Ring> ring_;
};


class Skeleton {
public:
    explicit Skeleton(std::shared_ptr<PointCloud>& cloudPtr, int k_neighbors, double fix_eigen_ratio, double diagonal_length,
                      std::string output_path) :
        cloudPtr_(cloudPtr),
        min_neighbors_(k_neighbors + 1), // Open3D KDTreeFlann search includes the query point itself, thus add 1
        fix_eigen_ratio_(fix_eigen_ratio),
        diagonal_length_(diagonal_length),
        output_path_(output_path),
        verticesPtr_(std::make_shared<std::vector<Point>>(cloudPtr->points_.size())),
        k_neighborsPtr_(std::make_shared<Eigen::VectorXi>(cloudPtr->points_.size())),
        ptsPtr_(std::make_shared<Eigen::MatrixXd>(cloudPtr->points_.size(), 3)),
        eigen_vectorsPtr_(std::make_shared<std::vector<Eigen::MatrixXd>>(cloudPtr->points_.size())),
        L_Ptr_(std::make_shared<Eigen::SparseMatrix<double, Eigen::RowMajor>>(cloudPtr->points_.size(), cloudPtr->points_.size())),
        WL_Ptr_(std::make_shared<Eigen::VectorXd>(cloudPtr->points_.size())),
        S_neighborsPtr_(std::make_shared<std::vector<std::vector<int>>>(cloudPtr->points_.size())),
        S_Ptr_(std::make_shared<Eigen::VectorXd>(cloudPtr->points_.size())),
        WS_Ptr_(std::make_shared<Eigen::VectorXd>(cloudPtr->points_.size())),
        WH_Ptr_(std::make_shared<Eigen::VectorXd>(cloudPtr->points_.size())),
        sL_(3.0),
        sigma_(cloudPtr_->points_.size(), 0.0), // TODO: transform to Pointer
        smooth_sigma_(cloudPtr_->points_.size(), 0.0), // TODO: transform to Pointer
        iteration_time_(50) {}

    std::shared_ptr<PointCloud> GetLaplacianSkeleton() {
        Convert2Matrix();
        k_neighborsPtr_->fill(min_neighbors_);
        std::shared_ptr<Eigen::MatrixXd> contracted_ptsPtr = ContractionIteration();
        std::shared_ptr<PointCloud> contracted_cloudPtr(new PointCloud);
        for (int i = 0; i < contracted_ptsPtr->rows(); ++i) {
            Eigen::Vector3d point(contracted_ptsPtr->row(i));
            contracted_cloudPtr->points_.push_back(point);
        }
        return contracted_cloudPtr;
    }

private:
    std::shared_ptr<PointCloud> cloudPtr_;
    int min_neighbors_;

    double fix_eigen_ratio_;
    double diagonal_length_;
    std::string output_path_;

    double sL_;
    int iteration_time_;

    std::shared_ptr<std::vector<Point>> verticesPtr_;
    std::shared_ptr<Eigen::VectorXi> k_neighborsPtr_;
    std::shared_ptr<Eigen::MatrixXd> ptsPtr_;
    std::shared_ptr<std::vector<Eigen::MatrixXd>> eigen_vectorsPtr_;

    std::shared_ptr<Eigen::SparseMatrix<double, Eigen::RowMajor>> L_Ptr_;
    std::shared_ptr<Eigen::VectorXd> WL_Ptr_;
    std::shared_ptr<std::vector<std::vector<int>>> S_neighborsPtr_;
    std::shared_ptr<Eigen::VectorXd> S_Ptr_;
    std::shared_ptr<Eigen::VectorXd> WS_Ptr_;
    std::shared_ptr<Eigen::VectorXd> WH_Ptr_;

    std::vector<double> sigma_;
    std::vector<double> smooth_sigma_;

    void Convert2Matrix();
    void ComputeProjectPlane(int index, Eigen::Vector3d& center, std::shared_ptr<PointCloud>& plane_cloudPtr);
    static void Project2PCA(Eigen::MatrixXd& eigen_vectors, std::shared_ptr<PointCloud>& cloudPtr, std::shared_ptr<PointCloud>& projected_cloudPtr);
    static void DelaunayTriangulation(std::shared_ptr<PointCloud>& cloudPtr, CGAL_Delaunay_2& delaunay);
    void FindOneRingNeighbors(std::shared_ptr<PointCloud>& cloudPtr, bool first_flag);
    void UpdateKNeighbors();
    Eigen::VectorXd GetNeighborhoodExtent(const std::shared_ptr<Eigen::MatrixXd>& ptsPtr, const std::string& method);
    void EstablishLaplacianMatrix(const std::shared_ptr<Eigen::MatrixXd>& ptsPtr);
    Eigen::MatrixXd LaplacianContraction(const std::shared_ptr<Eigen::MatrixXd>& ptsPtr);
    void ComputeSigma(const std::shared_ptr<Eigen::MatrixXd>& ptsPtr, const std::string& neighborhood);
    void FindSpringNeighbors(const int neighbors);
    void EstablishSpringSystem(const std::shared_ptr<Eigen::MatrixXd>& ptsPtr, int neighbors, double spring_constant, double spring_length);
    void InitializeParameters(const std::shared_ptr<Eigen::MatrixXd>& ptsPtr);
    std::shared_ptr<Eigen::MatrixXd> ContractionIteration();
};



#endif //CPPTEST_SKELETON_H
