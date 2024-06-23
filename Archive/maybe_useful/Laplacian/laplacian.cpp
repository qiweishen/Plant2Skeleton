#include <omp.h>
#include <memory>
#include <fstream>

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h>

#include <open3d/Open3D.h>
#include <open3d/geometry/KDTreeFlann.h>
#include <open3d/io/PointCloudIO.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Classification/Mesh_neighborhood.h>


typedef open3d::geometry::PointCloud PointCloud;
typedef open3d::geometry::KDTreeFlann KDTreeFlann;

typedef CGAL::Exact_predicates_inexact_constructions_kernel CGAL_K;
typedef CGAL::Delaunay_triangulation_2<CGAL_K> CGAL_Delaunay;
typedef CGAL_K::Point_2 CGAL_Point;
typedef CGAL_Delaunay::Vertex_handle CGAL_Vertex_handle;
typedef CGAL_Delaunay::Face_circulator CGAL_Face_circulator;





struct Ring {
    size_t id{};
    std::vector<size_t> neighbors; // The maximum number of neighbors is 2
};


class Point{
public:
    explicit Point(size_t point_id, std::vector<Ring> ring) : point_id_(point_id), ring_(ring) {}

    size_t get_id() const { return point_id_; }
    std::vector<Ring> get_ring() const { return ring_; }

private:
    size_t point_id_;
    std::vector<Ring> ring_;
};


void LoadPointCloud(const std::string& file_path, std::shared_ptr<PointCloud>& cloudPtr) {
    std::ifstream file(file_path);
    std::string suffix = file_path.substr(file_path.find_last_of("."));
    if (!file.is_open()) {
        throw std::runtime_error("No File Found");
    } else if (suffix != ".txt" && suffix != ".xyz" && suffix != ".ply" && suffix != ".pcd") {
        throw std::runtime_error("Only Support .txt .xyz .ply .pcd Format");
    } else if (suffix == ".txt" || suffix == ".xyz"){
        std::vector<Eigen::Vector3d> point_cloud;
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            double x, y, z;
            if (iss >> x >> y >> z) {
                Eigen::Vector3d point(x, y, z);
                point_cloud.push_back(point);
            }
        }
        file.close();
        cloudPtr->points_ = point_cloud;
    } else if (suffix == ".ply" || suffix == ".pcd") {
        open3d::io::ReadPointCloud(file_path, *cloudPtr);
    }
}


void Normalize(std::shared_ptr<PointCloud>& cloudPtr, Eigen::Vector3d& center, double& normalization_scaling) {
    // centre in zero
    center = (cloudPtr->GetMaxBound() + cloudPtr->GetMinBound()) / 2;
    for (Eigen::Vector3d& point : cloudPtr->points_) {
        point -= center;
    }

    // Calculate the scaling factor to make the diagonal 1.6 units
    Eigen::Vector3d diagonal = cloudPtr->GetMaxBound() - cloudPtr->GetMinBound();
    double maxDiagonal = diagonal.maxCoeff();
    normalization_scaling = 1.6 / maxDiagonal;

    // Apply the scaling
    for (Eigen::Vector3d& point : cloudPtr->points_) {
        point *= normalization_scaling;
    }
}


void Unnormalize(std::shared_ptr<Eigen::MatrixXd>& cloudPtr,
                 const Eigen::Vector3d& center,
                 const double normalizationScaling) {

    Eigen::MatrixXd unNormalizedCloud(cloudPtr->rows(), cloudPtr->cols());

    for (int i = 0; i < cloudPtr->rows(); ++i) {
        Eigen::Vector3d point = cloudPtr->row(i);
        point /= normalizationScaling;
        point += center;

        unNormalizedCloud.row(i) = point;
    }

    cloudPtr = std::make_shared<Eigen::MatrixXd>(unNormalizedCloud);
}


template<typename T>
void WritePLYFile(const std::string& file_path, const std::shared_ptr<T>& cloudPtr) {
}
template<>
void WritePLYFile<Eigen::Vector3d>(const std::string& file_path, const std::shared_ptr<Eigen::Vector3d>& cloudPtr) {
    PointCloud point_cloud;
    point_cloud.points_.push_back(*cloudPtr);
    point_cloud.PaintUniformColor(Eigen::Vector3d(1, 0, 0));
    open3d::io::WritePointCloud(file_path, point_cloud);
}
template<>
void WritePLYFile<Eigen::MatrixXd>(const std::string& file_path, const std::shared_ptr<Eigen::MatrixXd>& cloudPtr) {
    PointCloud point_cloud;
    for (int i = 0; i < cloudPtr->rows(); ++i) {
        Eigen::Vector3d point(cloudPtr->row(i));
        point_cloud.points_.push_back(point);
    }
    point_cloud.PaintUniformColor(Eigen::Vector3d(1, 0, 0));
    open3d::io::WritePointCloud(file_path, point_cloud);
}
template<>
void WritePLYFile<PointCloud>(const std::string& file_path, const std::shared_ptr<PointCloud>& cloudPtr) {
    open3d::io::WritePointCloud(file_path, *cloudPtr);
}


void Project2PCA(const std::shared_ptr<PointCloud>& cloudPtr,
                 std::shared_ptr<PointCloud>& projected_cloudPtr) {

    Eigen::Vector3d mean;
    Eigen::Matrix3d covariance;
    std::tie(mean, covariance) = cloudPtr->ComputeMeanAndCovariance();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
    Eigen::Matrix3d eigen_vectors = eigen_solver.eigenvectors(); // The columns are inverted, col(2).value > col(1).value > col(0).value

    size_t n = cloudPtr->points_.size();
    projected_cloudPtr->points_.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        Eigen::Vector3d centered = cloudPtr->points_.at(i);
        Eigen::Vector2d projected_pt(centered.dot(eigen_vectors.col(2)), centered.dot(eigen_vectors.col(1)));
        projected_cloudPtr->points_.emplace_back(projected_pt[0], projected_pt[1], 0);
    }
}


void DelaunayTriangulation(const std::shared_ptr<PointCloud>& cloudPtr, CGAL_Delaunay& delaunay) {
    for (const Eigen::Vector3d& point : cloudPtr->points_) {
        delaunay.insert(CGAL_Point(point[0], point[1]));
    }
}


Eigen::VectorXd GetNeighborhoodExtent(const std::shared_ptr<Eigen::MatrixXd>& ptsPtr,
                                      const std::shared_ptr<std::vector<Point>>& verticesPtr,
                                      const std::string& method) {

    Eigen::VectorXd result(ptsPtr->rows());

    if (method == "min") {
        for (const Point& point : *verticesPtr) {
            double min_distance = std::numeric_limits<double>::max();
            for (const Ring& ring : point.get_ring()) {
                double distance = (ptsPtr->row(point.get_id()) - ptsPtr->row(ring.id)).norm();
                min_distance = std::min(min_distance, distance);
            }
            result(point.get_id()) = min_distance;
        }
    } else if (method == "mean") {
        for (const Point& point : *verticesPtr) {
            for (const Ring& ring : point.get_ring()) {
                double distance = (ptsPtr->row(point.get_id()) - ptsPtr->row(ring.id)).norm();
                result(point.get_id()) += distance;
            }
            result(point.get_id()) /= point.get_ring().size();
        }
    } else if (method == "max") {
        for (const Point& point: *verticesPtr) {
            double max_distance = std::numeric_limits<double>::min();
            for (const Ring& ring: point.get_ring()) {
                double distance = (ptsPtr->row(point.get_id()) - ptsPtr->row(ring.id)).norm();
                max_distance = std::max(max_distance, distance);
            }
            result(point.get_id()) = max_distance;
        }
    } else {
        throw std::runtime_error("Invalid method!");
    }
    return result;
}


void FindOneRingNeighbors(const std::shared_ptr<PointCloud>& cloudPtr,
                          const int& k_neighbors,
                          std::shared_ptr<std::vector<Point>>& verticesPtr) {

    std::shared_ptr<KDTreeFlann> kdtree(new KDTreeFlann);
    kdtree->SetGeometry(*cloudPtr);

    // Compute Delaunay one Ring neighbors for Laplacian adjacency matrix construction (execute once only)
    verticesPtr->reserve(cloudPtr->points_.size());

//    #pragma omp parallel for shared(cloudPtr, kdtree, verticesPtr)
    for (int i = 0; i < cloudPtr->points_.size(); ++i) {
        Eigen::Vector3d point_xyz = cloudPtr->points_.at(i);
        std::vector<int> k_neighbors_idx;
        std::vector<double> _;
        kdtree->SearchKNN(point_xyz, k_neighbors + 1, k_neighbors_idx, _); // k_neighbors + 1, because the first point is the query point itself
        std::shared_ptr<PointCloud> k_neighbors_cloudPtr(new PointCloud);
        for (int& j : k_neighbors_idx) {
            k_neighbors_cloudPtr->points_.push_back(cloudPtr->points_.at(j));
        }

        std::shared_ptr<PointCloud> projected_cloudPtr(new PointCloud);
        Project2PCA(k_neighbors_cloudPtr, projected_cloudPtr);

        // Compute indices of one Ring points in the original point cloud
        std::unordered_map<CGAL_Point, size_t> one_ring_points_to_cloud_index_map;
        for (size_t j = 0; j < k_neighbors + 1; ++j) {
            CGAL_Point cgal_point(projected_cloudPtr->points_.at(j)[0], projected_cloudPtr->points_.at(j)[1]);
            one_ring_points_to_cloud_index_map[cgal_point] = k_neighbors_idx[j];
        }

        CGAL_Delaunay dt;
        DelaunayTriangulation(projected_cloudPtr, dt);

        std::vector<Ring> one_ring;
        CGAL_Point query_point(projected_cloudPtr->points_.at(0)[0], projected_cloudPtr->points_.at(0)[1]);
        CGAL_Vertex_handle v_handle = dt.nearest_vertex(query_point);

        assert(v_handle->point() == query_point);

        CGAL_Face_circulator fc = dt.incident_faces(v_handle), done(fc);
        if (fc != nullptr) {
            do {
                if (!dt.is_infinite(fc)) {
                    bool add_point_one = true;
                    bool add_point_two = true;

                    std::vector<size_t> ring_idx;
                    for (size_t j = 0; j < 3; ++j) {
                        CGAL_Point p = fc->vertex(j)->point();
                        size_t p_id = one_ring_points_to_cloud_index_map[p];
                        ring_idx.push_back(p_id);
                    }

                    // Move the query point to the first position
                    auto it = std::find(ring_idx.begin(), ring_idx.end(), i);
                    if (it != ring_idx.end()) {
                        std::rotate(ring_idx.begin(), it, it + 1);
                    }
                    assert(ring_idx[0] == k_neighbors_idx[0]);

                    for (Ring& j : one_ring) {
                        if (j.id == ring_idx[1]) {
                            add_point_one = false;
                            j.neighbors.push_back(ring_idx[2]);
                        }
                        if (j.id == ring_idx[2]) {
                            add_point_two = false;
                            j.neighbors.push_back(ring_idx[1]);
                        }
                    }

                    if (add_point_one) {
                        Ring r;
                        r.id = ring_idx[1];
                        r.neighbors.push_back(ring_idx[2]);
                        one_ring.push_back(r);
                    }
                    if (add_point_two) {
                        Ring r;
                        r.id = ring_idx[2];
                        r.neighbors.push_back(ring_idx[1]);
                        one_ring.push_back(r);
                    }
                }
            } while (++fc != done);
        }

        // Critical section to protect push_back operation
//        #pragma omp critical
        {
            Point point(i, one_ring);
            verticesPtr->push_back(point);
        }
    }

    assert (verticesPtr->size() == cloudPtr->points_.size());
}


void ComputeLaplacianMatrix(const std::shared_ptr<Eigen::MatrixXd>& ptsPtr,
                            const std::shared_ptr<std::vector<Point>>& verticesPtr,
                            std::shared_ptr<Eigen::SparseMatrix<double>>& L_Ptr) {

    L_Ptr->setZero();

    /* The laplacian operator is defined with the cotangent weights for each edge.
     * E.g., with the two triangles (abd) and (acd), for the edge (ad) we sum cotan(b) and cotan(c).
     *
     *    a---d
     *    | \ |
     *    b---c
     *
     * This part is adopted from https://github.com/rFalque/pointCloudSkeletonization/blob/master/include/skeleton_toolbox/pointSkeletonization.hpp#L381-L441
     */

    // Construct Laplacian matrix
//    #pragma omp parallel for shared(verticesPtr, ptsPtr, L_Ptr)
    for (const Point& point : *verticesPtr) {
        Eigen::Vector3d a = ptsPtr->row(point.get_id());
        std::vector<Ring> one_ring = point.get_ring();

        for (const Ring& ring : one_ring) {
            double cotan = 0;
            Eigen::Vector3d c = ptsPtr->row(ring.id);
            std::vector<size_t> connected_vertices = ring.neighbors;

            for (const size_t& index : connected_vertices) {
                Eigen::Vector3d b = ptsPtr->row(index);

                Eigen::Vector3d ba = a - b;
                Eigen::Vector3d bc = c - b;
                cotan += (bc.dot(ba)) / (bc.cross(ba).norm());
            }

//            #pragma omp critical
            L_Ptr->coeffRef(point.get_id(), ring.id) = cotan;
        }

        double temp = std::abs(L_Ptr->row(point.get_id()).sum());
        if (temp > 10000) {
            double scale_factor = 10000 / temp;
//            #pragma omp critical
            {
                for (int i = 0; i < L_Ptr->outerSize(); ++i) {
                    for (Eigen::SparseMatrix<double>::InnerIterator it(*L_Ptr, i); it; ++it) {
                        if (it.row() == point.get_id()) {
                            it.valueRef() *= scale_factor;
                        }
                    }
                }
            }
        }
    }

    Eigen::VectorXd diagonal = Eigen::VectorXd::Zero(ptsPtr->rows());
    for (int i = 0; i < ptsPtr->rows(); ++i) {
        diagonal(i) = L_Ptr->row(i).sum();
    }
    Eigen::SparseMatrix<double> diagonalMatrix(ptsPtr->rows(), ptsPtr->rows());
    for (int i = 0; i < ptsPtr->rows(); ++i) {
        diagonalMatrix.coeffRef(i, i) = diagonal(i); // Transpose the diagonal vector to a diagonal matrix
    }
    *L_Ptr = *L_Ptr - diagonalMatrix;
}


void InitializeParameters(const std::shared_ptr<Eigen::MatrixXd>& ptsPtr,
                          const std::shared_ptr<std::vector<Point>>& verticesPtr,
                          std::shared_ptr<double>& WL_Ptr,
                          std::shared_ptr<Eigen::SparseMatrix<double>>& WH_Ptr,
                          const double& WC,
                          std::shared_ptr<Eigen::SparseMatrix<double>>& L_Ptr) {

    // Compute initial WL_weight
    Eigen::VectorXd mean_extents = GetNeighborhoodExtent(ptsPtr, verticesPtr, "mean");
    WL_Ptr = std::make_shared<double>(1.0 / (5.0 * mean_extents.mean()));

    // Compute initial WH diagonal matrix
    for (int i = 0; i < ptsPtr->rows(); ++i) {
        WH_Ptr->coeffRef(i, i) = WC;
    }

    ComputeLaplacianMatrix(ptsPtr, verticesPtr, L_Ptr);
}


Eigen::MatrixXd LaplacianContraction(const std::shared_ptr<Eigen::MatrixXd>& ptsPtr,
                                     std::shared_ptr<double>& WL_Ptr,
                                     std::shared_ptr<Eigen::SparseMatrix<double>>& WH_Ptr,
                                     std::shared_ptr<Eigen::SparseMatrix<double>>& L_Ptr) {

    const int n = static_cast<int>(ptsPtr->rows());

    // Construct the matrix A (2n x n) { A = [L.*WL; sparse(1:P.npts, 1:P.npts, WH)]; }
    *L_Ptr = (*L_Ptr * *WL_Ptr).real();
    Eigen::SparseMatrix<double> A(2 * n, n);
    std::vector<Eigen::Triplet<double>> A_tripletList;
    // Fill the upper part of A
    for (int i = 0; i < L_Ptr->outerSize(); ++i) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(*L_Ptr, i); it; ++it) {
            A_tripletList.emplace_back(it.row(), it.col(), it.value());
        }
    }
    // Fill the lower half of A
    for (int i = 0; i < WH_Ptr->outerSize(); ++i) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(*WH_Ptr, i); it; ++it) {
            A_tripletList.emplace_back(n + it.row(), it.col(), it.value());
        }
    }
    A.setFromTriplets(A_tripletList.begin(), A_tripletList.end());

    // Construct vector b (2n x 3) { b = [zeros(P.npts,3); sparse(1:P.npts,1:P.npts, WH)*P.pts]; }
    Eigen::MatrixXd zeros = Eigen::MatrixXd::Zero(n, 3);
    Eigen::MatrixXd WHP = *WH_Ptr * *ptsPtr;
    Eigen::MatrixXd b(2 * n, 3);
    b << zeros, WHP;

    // Solve the linear system
    Eigen::SparseMatrix<double> ATA = A.transpose() * A;
    Eigen::MatrixXd ATb = A.transpose() * b;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(ATA);
    if (solver.info() != Eigen::Success) {
        std::cout << "Error: Eigen decomposition failed!" << std::endl;
    }
    Eigen::MatrixXd cpts = solver.solve(ATb);

    return cpts;
}



std::shared_ptr<Eigen::MatrixXd> ContractionIteration(std::shared_ptr<Eigen::MatrixXd>& ptsPtr,
                                                      const std::shared_ptr<std::vector<Point>>& verticesPtr,
                                                      std::shared_ptr<double>& WL_Ptr,
                                                      const double& WC,
                                                      std::shared_ptr<Eigen::SparseMatrix<double>>& WH_Ptr,
                                                      std::shared_ptr<Eigen::SparseMatrix<double>>& L_Ptr,
                                                      const double& sL,
                                                      const int& iteration_time) {

    const int n = static_cast<int>(ptsPtr->rows());
    std::vector<double> contraction_history;

    // Contract once
    std::cout << "Contraction iteration time: " << 1 << "\n";
    Eigen::MatrixXd cpts = LaplacianContraction(ptsPtr, WL_Ptr, WH_Ptr, L_Ptr);
    std::shared_ptr<Eigen::MatrixXd> cptsPtr(new Eigen::MatrixXd(cpts));

    // Update neighborhood extent
    const Eigen::VectorXd extents = GetNeighborhoodExtent(ptsPtr, verticesPtr, "min");
    Eigen::VectorXd new_extents = GetNeighborhoodExtent(cptsPtr, verticesPtr, "min");
    double contraction_rate = new_extents.sum() / extents.sum();
    std::cout << "Neighborhood extent has been updated! Current extent is: " << new_extents.sum() << "; " << "the contract ratio is: " << contraction_rate << std::endl;
    contraction_history.push_back(contraction_rate);

    for (int i = 0; i < iteration_time - 1; ++i) {
        std::cout << "Contraction iteration time: " << i + 2 << "\n";

        // Update WL, and check if it exceeds the maximum value
        *WL_Ptr *= sL;
        if (*WL_Ptr > 2048) {
            WL_Ptr = std::make_shared<double>(2048);
        }

        // Update WH, and check if it exceeds the maximum value
        for (int j = 0; j < n; ++j) {
            double temp = WC * extents(j) / new_extents(j);
            if (temp > 10000) {
                WH_Ptr->coeffRef(j, j) = 10000;
            } else {
                WH_Ptr->coeffRef(j, j) = temp;
            }
        }

        // Update Laplacian matrix
        ComputeLaplacianMatrix(cptsPtr, verticesPtr, L_Ptr);

        // Contract one more time
        Eigen::MatrixXd cpts_temp = LaplacianContraction(cptsPtr, WL_Ptr, WH_Ptr, L_Ptr);
        std::shared_ptr<Eigen::MatrixXd> cpts_tempPtr(new Eigen::MatrixXd(cpts_temp));
        new_extents = GetNeighborhoodExtent(cpts_tempPtr, verticesPtr, "min");
        contraction_rate = new_extents.sum() / extents.sum();
        contraction_history.push_back(contraction_rate);
        std::cout << "Neighborhood extent has been updated! Current extent is: " << new_extents.sum() << "; " << "the contract ratio is: " << contraction_rate << std::endl;
        if (contraction_history[i] - contraction_history.back() < 0.01 || std::isnan(contraction_history.back())) {
            std::cout << "Touch the threshold! Iteration " << i + 2 << " terminated! Total contraction iteration time: " << i + 1 << std::endl;
            break;
        }

        // Update cptsPtr
        *cptsPtr = *cpts_tempPtr;
    }

    return cptsPtr;
}


int main() {
    std::string input_file = "horse_v1987";
    std::string suffix = ".xyz";
    // "horse_v1987"; "Pheno4D_v5000"; "C20-1_v4476"; "C45-3_v4784"; "C46-4_v4463"; "C70-5_v4806"
    int max_iteration_time = 20;

    std::string input_file_path = "../data/" + input_file + suffix;
    std::string output_file_path;
    if (max_iteration_time == 20) {
        output_file_path = "../data/Output/" + input_file + "_skeleton.ply";
    } else {
        output_file_path = "../data/Output/" + input_file + "_skeleton_" + std::to_string(max_iteration_time) + ".ply";
    }

    int k_neighbors = 8;
    double initial_WC = 1.0;
    double inital_sL = 3.0;

    std::shared_ptr<PointCloud> cloudPtr(new PointCloud);
    LoadPointCloud(input_file_path, cloudPtr);
    cloudPtr->RemoveDuplicatedPoints();
    std::cout << "[1] data loaded!" << std::endl;

    Eigen::Vector3d center;
    double normalization_scaling;
    Normalize(cloudPtr, center, normalization_scaling);
    std::cout << "[2] Normalized!" << std::endl;

    std::shared_ptr<std::vector<Point>> verticesPtr(new std::vector<Point>);
    FindOneRingNeighbors(cloudPtr, k_neighbors, verticesPtr);
    std::cout << "[3] One Ring neighbors were found!" << std::endl;

    const int n = static_cast<int>(cloudPtr->points_.size());
    std::shared_ptr<Eigen::MatrixXd> ptsPtr(new Eigen::MatrixXd(n, 3));
    for (int i = 0; i < n; ++i) {
        ptsPtr->row(i) = cloudPtr->points_.at(i);
    }
    std::cout << "[4] Point cloud converted to Eigen::Matrix!" << std::endl;

    std::shared_ptr<double> initial_WL_Ptr(new double);
    std::shared_ptr<Eigen::SparseMatrix<double>> initial_WH_Ptr(new Eigen::SparseMatrix<double>(n, n));
    std::shared_ptr<Eigen::SparseMatrix<double>> initial_L_Ptr(new Eigen::SparseMatrix<double>(n, n));
    InitializeParameters(ptsPtr, verticesPtr, initial_WL_Ptr, initial_WH_Ptr, initial_WC, initial_L_Ptr);
    std::cout << "[5] Parameters initialized!" << std::endl; // TODO: Accelerate the initialization process, OpenMP?

    std::shared_ptr<Eigen::MatrixXd> skeletonPtr = ContractionIteration(ptsPtr, verticesPtr, initial_WL_Ptr,
                                                                        initial_WC, initial_WH_Ptr,
                                                                        initial_L_Ptr, inital_sL, max_iteration_time);
    std::cout << "[6] Skeletonization completed!" << std::endl;

    Unnormalize(skeletonPtr, center, normalization_scaling);
    std::cout << "[7] Unnormalized!" << std::endl;

    WritePLYFile(output_file_path, skeletonPtr);
    std::cout << "[8] Skeleton points saved!" << std::endl;

    return 0;
}
