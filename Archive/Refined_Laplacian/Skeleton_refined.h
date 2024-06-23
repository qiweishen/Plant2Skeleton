#ifndef CPPTEST_SKELETON_H
#define CPPTEST_SKELETON_H



#include <omp.h>
#include <memory>
#include <fstream>
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
typedef CGAL::Delaunay_triangulation_2<CGAL_K> CGAL_Delaunay;
typedef CGAL_K::Point_2 CGAL_Point_2;
typedef CGAL_Delaunay::Vertex_handle CGAL_Vertex_handle;
typedef CGAL_Delaunay::Face_circulator CGAL_Face_circulator;





struct CGAL_Point_Hash {
    std::size_t operator()(const CGAL_Point_2& p) const {
        auto hash1 = std::hash<double>{}(CGAL::to_double(p.x()));
        auto hash2 = std::hash<double>{}(CGAL::to_double(p.y()));
        return hash1 ^ (hash2 << 1);  // Combine the two hash values
    }
};


struct Ring {
    int id{};
    std::vector<int> neighbors; // The maximum number of neighbors is 2
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
        k_neighbors_(k_neighbors),
        fix_eigen_ratio_(fix_eigen_ratio),
        diagonal_length_(diagonal_length),
        output_path_(output_path),
        verticesPtr_(std::make_shared<std::vector<Point>>()),
        spring_verticesPtr_(std::make_shared<std::vector<Point>>()),
        ptsPtr_(std::make_shared<Eigen::MatrixXd>(cloudPtr->points_.size(), 3)),
        L_Ptr_(std::make_shared<Eigen::SparseMatrix<double, Eigen::RowMajor>>(cloudPtr->points_.size(), cloudPtr->points_.size())),
        WL_Ptr_(std::make_shared<Eigen::VectorXd>(cloudPtr->points_.size())),
        WH_Ptr_(std::make_shared<Eigen::SparseMatrix<double>>(cloudPtr->points_.size(), cloudPtr->points_.size())),
        sL_(3.0),
        sigma_(cloudPtr_->points_.size(), 0.0),
        smooth_sigma_(cloudPtr_->points_.size(), 0.0),
        iteration_time_(50) {}

    std::shared_ptr<PointCloud> GetLaplacianSkeleton() {
        Convert2Matrix();
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
    int k_neighbors_;
    double fix_eigen_ratio_;
    double diagonal_length_;
    std::string output_path_;

    double sL_;
    int iteration_time_;

    std::shared_ptr<std::vector<Point>> verticesPtr_;
    std::shared_ptr<std::vector<Point>> spring_verticesPtr_;
    std::shared_ptr<Eigen::MatrixXd> ptsPtr_;

    std::shared_ptr<Eigen::SparseMatrix<double, Eigen::RowMajor>> L_Ptr_;
    std::shared_ptr<Eigen::VectorXd> WL_Ptr_;
    std::shared_ptr<Eigen::SparseMatrix<double>> WH_Ptr_;

    std::vector<double> sigma_;
    std::vector<double> smooth_sigma_;

    void Convert2Matrix();
    static void Project2PCA(std::shared_ptr<PointCloud>& cloudPtr, std::shared_ptr<PointCloud>& projected_cloudPtr);
    static void DelaunayTriangulation(std::shared_ptr<PointCloud>& cloudPtr, CGAL_Delaunay& delaunay);
    bool FindOneRingNeighbors(std::shared_ptr<PointCloud>& cloudPtr);
    Eigen::VectorXd GetNeighborhoodExtent(const std::shared_ptr<Eigen::MatrixXd>& ptsPtr, const std::string& method);
    void ComputeLaplacianMatrix(const std::shared_ptr<Eigen::MatrixXd>& ptsPtr, bool with_sigma);
    Eigen::MatrixXd LaplacianContraction(const std::shared_ptr<Eigen::MatrixXd>& ptsPtr);
    void ComputeSigma(const std::shared_ptr<Eigen::MatrixXd>& ptsPtr, const std::string& neighborhood);
    void InitializeParameters(const std::shared_ptr<Eigen::MatrixXd>& ptsPtr);
    std::shared_ptr<Eigen::MatrixXd> ContractionIteration();
};



#endif //CPPTEST_SKELETON_H
