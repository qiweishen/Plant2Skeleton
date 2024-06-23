#include "Skeleton.h"
#include "Tools.h"


void Skeleton::Convert2Matrix() {
    for (int i = 0; i < cloudPtr_->points_.size(); ++i) {
        ptsPtr_->row(i) = cloudPtr_->points_[i];
    }
}


//void Skeleton::Project2PCA(Eigen::Vector3d& center, std::shared_ptr<PointCloud>& plane_cloudPtr,
//                           std::shared_ptr<PointCloud>& cloudPtr, std::shared_ptr<PointCloud>& projected_cloudPtr) {
//    Eigen::Vector3d mean;
//    Eigen::Matrix3d covariance;
//    std::tie(mean, covariance) = cloudPtr->ComputeMeanAndCovariance();
//    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
//    Eigen::Matrix3d eigen_vectors = eigen_solver.eigenvectors(); // The columns are inverted, col(2).value > col(1).value > col(0).value
//
//    const int n = static_cast<int>(cloudPtr->points_.size());
//    projected_cloudPtr->points_.reserve(n);
//    for (size_t i = 0; i < n; ++i) {
//        Eigen::Vector3d centered = cloudPtr->points_.at(i) - mean;
//        Eigen::Vector2d projected_pt(centered.dot(eigen_vectors.col(2)), centered.dot(eigen_vectors.col(1)));
//        projected_cloudPtr->points_.emplace_back(projected_pt[0], projected_pt[1], 0);
//    }
//}

//void Skeleton::Project2PCA(Eigen::Vector3d& center, std::shared_ptr<PointCloud>& plane_cloudPtr,
//                           std::shared_ptr<PointCloud>& cloudPtr, std::shared_ptr<PointCloud>& projected_cloudPtr) {
//    Eigen::Vector3d mean;
//    Eigen::Matrix3d covariance;
//    std::tie(mean, covariance) = plane_cloudPtr->ComputeMeanAndCovariance();
//    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
//    Eigen::Matrix3d eigen_vectors = eigen_solver.eigenvectors(); // The columns are inverted, col(2).value > col(1).value > col(0).value
//
//    const int n = static_cast<int>(cloudPtr->points_.size());
//    projected_cloudPtr->points_.reserve(n);
//    for (size_t i = 0; i < n; ++i) {
//        Eigen::Vector3d centered = cloudPtr->points_.at(i) - mean;
//        Eigen::Vector2d projected_pt(centered.dot(eigen_vectors.col(2)), centered.dot(eigen_vectors.col(1)));
//        projected_cloudPtr->points_.emplace_back(projected_pt[0], projected_pt[1], 0);
//    }
//}

//void Skeleton::Project2PCA(Eigen::Vector3d& center, std::shared_ptr<PointCloud>& plane_cloudPtr,
//                           std::shared_ptr<PointCloud>& cloudPtr, std::shared_ptr<PointCloud>& projected_cloudPtr) {
//    double radius = 0.06 * 1.6;
//    // Compute the covariance matrix for plane_cloudPtr
//    Eigen::MatrixXd Ci = Eigen::MatrixXd::Zero(3, 3);
//    for (const Eigen::Vector3d& pj : plane_cloudPtr->points_) {
//        Eigen::Vector3d diff = pj - center;
//        double distance = diff.norm();
//        double weight = exp(-pow(distance, 2) / pow(radius / 4, 2));
//        Ci += weight * diff * diff.transpose();
//    }
//    // Perform eigen decomposition once
//    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(Ci);
//    Eigen::MatrixXd eigen_vectors = eigen_solver.eigenvectors(); // The columns are inverted, col(2).value > col(1).value > col(0).value
//
//    // Project each point in cloudPtr onto the 2D plane defined by the top two eigenvectors
//    for (const Eigen::Vector3d& query_point : cloudPtr->points_) {
//        Eigen::Vector3d centered = query_point;
//        Eigen::Vector2d projected_pt(centered.dot(eigen_vectors.col(2)), centered.dot(eigen_vectors.col(1)));
//        projected_cloudPtr->points_.emplace_back(projected_pt[0], projected_pt[1], 0);
//    }
//}

void Skeleton::Project2PCA(Eigen::MatrixXd &eigen_vectors,
                           std::shared_ptr<PointCloud> &cloudPtr, std::shared_ptr<PointCloud> &projected_cloudPtr) {
    // Project each point in cloudPtr onto the 2D plane defined by the top two eigenvectors
    for (const Eigen::Vector3d &query_point: cloudPtr->points_) {
        Eigen::Vector3d centered = query_point;
        Eigen::Vector2d projected_pt(centered.dot(eigen_vectors.col(2)), centered.dot(eigen_vectors.col(1)));
        projected_cloudPtr->points_.emplace_back(projected_pt[0], projected_pt[1], 0);
    }
}


void Skeleton::ComputeProjectPlane(int index, Eigen::Vector3d &center, std::shared_ptr<PointCloud> &plane_cloudPtr) {
    double radius = 0.06 * 1.6;
    // Compute the covariance matrix for plane_cloudPtr
    Eigen::MatrixXd Ci = Eigen::MatrixXd::Zero(3, 3);
    for (const Eigen::Vector3d &pj: plane_cloudPtr->points_) {
        Eigen::Vector3d diff = pj - center;
        double distance = diff.norm();
        double weight = exp(-pow(distance, 2) / pow(radius / 4, 2));
        Ci += weight * diff * diff.transpose();
    }
    // Perform eigen decomposition once
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(Ci);
    const Eigen::MatrixXd &eigen_vectors = eigen_solver.eigenvectors(); // The columns are inverted, col(2).value > col(1).value > col(0).value
    eigen_vectorsPtr_->at(index) = eigen_vectors;
}


void Skeleton::DelaunayTriangulation(std::shared_ptr<PointCloud> &cloudPtr, CGAL_Delaunay_2 &delaunay) {
    for (const Eigen::Vector3d &point: cloudPtr->points_) {
        delaunay.insert(CGAL_Point_2(point[0], point[1]));
    }
}


Eigen::VectorXd Skeleton::GetNeighborhoodExtent(const std::shared_ptr<Eigen::MatrixXd> &ptsPtr,
                                                const std::string &method) {
    auto start = std::chrono::high_resolution_clock::now();

    Eigen::VectorXd result(ptsPtr->rows());
    result.fill(0.0); // Miraculous line, to avoid the bug of unstable contraction result
    if (method == "min") {
        for (const Point &point: *verticesPtr_) {
            double min_distance = std::numeric_limits<double>::max();
            for (const Ring &ring: point.get_ring()) {
                double distance = (ptsPtr->row(point.get_id()) - ptsPtr->row(ring.id)).norm();
                min_distance = std::min(min_distance, distance);
            }
            result(point.get_id()) = min_distance;
        }
    } else if (method == "mean") {
        for (const Point &point: *verticesPtr_) {
            for (const Ring &ring: point.get_ring()) {
                double distance = (ptsPtr->row(point.get_id()) - ptsPtr->row(ring.id)).norm();
                result(point.get_id()) += distance;
            }
            result(point.get_id()) /= int(point.get_ring().size());
        }
    } else if (method == "max") {
        for (const Point &point: *verticesPtr_) {
            double max_distance = std::numeric_limits<double>::min();
            for (const Ring &ring: point.get_ring()) {
                double distance = (ptsPtr->row(point.get_id()) - ptsPtr->row(ring.id)).norm();
                max_distance = std::max(max_distance, distance);
            }
            result(point.get_id()) = max_distance;
        }
    } else {
        throw std::runtime_error("Invalid method!");
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Neighborhood extent has been updated! Elapsed time: " << elapsed.count() << "s" << std::endl;

    return result;
}


void Skeleton::FindOneRingNeighbors(std::shared_ptr<PointCloud> &cloudPtr, bool first_flag) {
    auto start = std::chrono::high_resolution_clock::now();

    const int n = static_cast<int>(cloudPtr->points_.size());
    std::shared_ptr<KDTreeFlann> kdtree = std::make_shared<KDTreeFlann>();
    kdtree->SetGeometry(*cloudPtr);

    if (first_flag) {
        for (int i = 0; i < n; ++i) {
            if (smooth_sigma_[i] < fix_eigen_ratio_ - 0.30) { // 0.65
                Eigen::Vector3d point_xyz = cloudPtr->points_.at(i);
                std::vector<int> vertices4plane_idx;
                std::vector<double> _;
                kdtree->SearchRadius(point_xyz, 0.06 * diagonal_length_, vertices4plane_idx, _);

                assert(k_neighborsPtr_->coeffRef(i) <= vertices4plane_idx.size());

                std::shared_ptr<PointCloud> vertices4plane_cloudPtr = tool::utility::SelectByIndices(cloudPtr,
                                                                                                     vertices4plane_idx);
                ComputeProjectPlane(i, point_xyz, vertices4plane_cloudPtr);
            } else {
                continue;
            }
        }
    }

    verticesPtr_->clear();
    verticesPtr_->resize(n);
#pragma omp parallel for shared(cloudPtr, kdtree, verticesPtr_)
    for (int i = 0; i < n; ++i) {
        Eigen::Vector3d point_xyz = cloudPtr->points_.at(i);
        std::vector<int> k_neighbors_idx;
        std::vector<double> _;
        kdtree->SearchKNN(point_xyz, k_neighborsPtr_->coeffRef(i), k_neighbors_idx, _);
        std::shared_ptr<PointCloud> k_neighbors_cloudPtr = tool::utility::SelectByIndices(cloudPtr, k_neighbors_idx);
        std::shared_ptr<PointCloud> projected_cloudPtr(new PointCloud);
        Eigen::MatrixXd eigen_vectors = eigen_vectorsPtr_->at(i);
        Project2PCA(eigen_vectors, k_neighbors_cloudPtr, projected_cloudPtr);

        // Compute indices of one Ring points in the original point cloud
        std::unordered_map<CGAL_Point_2, int, CGAL_Point_Hash> one_ring_points_to_cloud_index_map;
        for (int j = 0; j < k_neighbors_idx.size(); ++j) {
            CGAL_Point_2 cgal_point(projected_cloudPtr->points_.at(j)[0], projected_cloudPtr->points_.at(j)[1]);
            one_ring_points_to_cloud_index_map[cgal_point] = k_neighbors_idx[j];
        }

        CGAL_Delaunay_2 dt;
        DelaunayTriangulation(projected_cloudPtr, dt);

        std::vector<Ring> one_ring;
        CGAL_Point_2 query_point(projected_cloudPtr->points_.at(0)[0], projected_cloudPtr->points_.at(0)[1]);
        CGAL_Vertex_handle_2 v_handle = dt.nearest_vertex(query_point);
        assert(v_handle->point() == query_point);

        CGAL_Face_circulator_2 fc = dt.incident_faces(v_handle), done(fc);
        if (fc != nullptr) {
            do {
                if (!dt.is_infinite(fc)) {
                    bool add_point_one = true;
                    bool add_point_two = true;

                    std::vector<int> ring_idx;
                    for (int j = 0; j < 3; ++j) {
                        CGAL_Point_2 p = fc->vertex(j)->point();
                        int p_id = one_ring_points_to_cloud_index_map[p];
                        ring_idx.emplace_back(p_id);
                    }

                    // Move the query point to the first position
                    auto it = std::find(ring_idx.begin(), ring_idx.end(), i);
                    if (it != ring_idx.end()) {
                        std::rotate(ring_idx.begin(), it, it + 1);
                    }

                    assert(ring_idx[0] == k_neighbors_idx[0]);

                    for (Ring &j: one_ring) {
                        if (j.id == ring_idx[1]) {
                            add_point_one = false;
                            j.neighbors.emplace_back(ring_idx[2]);
                        }
                        if (j.id == ring_idx[2]) {
                            add_point_two = false;
                            j.neighbors.emplace_back(ring_idx[1]);
                        }
                    }

                    if (add_point_one) {
                        Ring r;
                        r.id = ring_idx[1];
                        r.neighbors.emplace_back(ring_idx[2]);
                        one_ring.emplace_back(r);
                    }
                    if (add_point_two) {
                        Ring r;
                        r.id = ring_idx[2];
                        r.neighbors.emplace_back(ring_idx[1]);
                        one_ring.emplace_back(r);
                    }
                }
            } while (++fc != done);
        }
#pragma omp critical
        {
            verticesPtr_->at(i) = Point(i, one_ring);
        }
    }

    assert (verticesPtr_->size() == n);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "One ring neighbors have been found! Elapsed time: " << elapsed.count() << "s" << std::endl;
}


void Skeleton::UpdateKNeighbors() {
    const int max_neighbors = 24;
    const int min_neighbors = 8;
    for (int i = 0; i < smooth_sigma_.size(); ++i) {
        if (smooth_sigma_[i] >= fix_eigen_ratio_ - 0.05) {
            k_neighborsPtr_->coeffRef(i) = 8;
        } else if (smooth_sigma_[i] >= fix_eigen_ratio_ - 0.15) {
            continue;
        } else if (smooth_sigma_[i] < fix_eigen_ratio_ - 0.25) {
            k_neighborsPtr_->coeffRef(i) =
                    k_neighborsPtr_->coeffRef(i) + 4 > max_neighbors ? max_neighbors : k_neighborsPtr_->coeffRef(i) + 4;
        }
    }
}


void Skeleton::EstablishLaplacianMatrix(const std::shared_ptr<Eigen::MatrixXd> &ptsPtr) {
    auto start = std::chrono::high_resolution_clock::now();

    const int n = static_cast<int>(ptsPtr->rows());

    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(n * 100); // Estimate of 100 entries per row

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
    for (const Point &point: *verticesPtr_) {
        Eigen::Vector3d a = ptsPtr->row(point.get_id());
        std::vector<Ring> one_ring = point.get_ring();

        for (const Ring &ring: one_ring) {
            double cotan = 0.0;
            Eigen::Vector3d c = ptsPtr->row(ring.id);
            std::vector<int> connected_vertices = ring.neighbors;

            for (const int &index: connected_vertices) {
                Eigen::Vector3d b = ptsPtr->row(index);

                Eigen::Vector3d ba = a - b;
                Eigen::Vector3d bc = c - b;
                cotan += (bc.dot(ba)) / (bc.cross(ba).norm());
            }
            triplets.emplace_back(point.get_id(), ring.id, cotan);
        }
    }
    L_Ptr_->setZero();
    L_Ptr_->setFromTriplets(triplets.begin(), triplets.end());

#pragma omp parallel for default(none) shared(n)
    for (int i = 0; i < n; ++i) {
        double row_sum = std::abs(L_Ptr_->row(i).sum());
        if (row_sum > 10000) {
            double scale_factor = 10000 / row_sum;
            L_Ptr_->row(i) *= scale_factor;
        }
    }

    Eigen::SparseMatrix<double, Eigen::RowMajor> diagonalMatrix(n, n);
    diagonalMatrix.reserve(Eigen::VectorXi::Constant(n, 1)); // Reserve 1 nonzero per row
    // Insert zeros into the diagonal entries to establish the structure
    for (int i = 0; i < n; ++i) {
        diagonalMatrix.insert(i, i) = 0.0;
    }
#pragma omp parallel for default(none) shared(n, diagonalMatrix)
    for (int i = 0; i < n; ++i) {
        diagonalMatrix.coeffRef(i, i) = L_Ptr_->row(i).sum();
    }

    *L_Ptr_ -= diagonalMatrix;

    std::cout << L_Ptr_->coeffRef(1234, 1234) << std::endl;

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Laplacian matrix has been established! Elapsed time: " << elapsed.count() << "s" << std::endl;
}


void Skeleton::FindSpringNeighbors(const int neighbors) {
    auto start = std::chrono::high_resolution_clock::now();
    const int n = static_cast<int>(cloudPtr_->points_.size());

    std::shared_ptr<KDTreeFlann> kdtree(new KDTreeFlann);
    kdtree->SetGeometry(*cloudPtr_);

    S_neighborsPtr_->clear();
    S_neighborsPtr_->resize(n);

    for (int i = 0; i < n; ++i) {
        Eigen::Vector3d query_point = cloudPtr_->points_[i];
        std::vector<int> indices;
        std::vector<double> _;
        kdtree->SearchKNN(query_point, neighbors, indices, _);
        assert(indices.size() == neighbors && indices[0] == i);
        indices.erase(indices.begin()); // Remove the query point itself
        S_neighborsPtr_->at(i) = indices;
    }
}


//void Skeleton::EstablishSpringSystem(const std::shared_ptr<Eigen::MatrixXd>& ptsPtr, int neighbors,
//                                     const double spring_constant, const double spring_length) {
//    auto start = std::chrono::high_resolution_clock::now();
//
//    const int n = static_cast<int>(ptsPtr->rows());
//
//    std::shared_ptr<open3d::geometry::PointCloud> pts_cloudPtr(new open3d::geometry::PointCloud);
//    for (int i = 0; i < n; ++i) {
//        pts_cloudPtr->points_.emplace_back(ptsPtr->row(i));
//    }
//    std::shared_ptr<open3d::geometry::KDTreeFlann> kdtree(new open3d::geometry::KDTreeFlann);
//    kdtree->SetGeometry(*pts_cloudPtr);
//
//    neighbors = neighbors + 1; // Open3D KDTreeFlann search includes the query point itself
//    std::vector<std::vector<Eigen::Triplet<double>>> triplets_per_thread(omp_get_max_threads());
//
//    #pragma omp parallel for default(none) shared(n, ptsPtr, kdtree, neighbors, spring_length, spring_constant, triplets_per_thread)
//    for (int i = 0; i < n; ++i) {
//        Eigen::Vector3d query_point = ptsPtr->row(i);
//        std::vector<int> indices;
//        std::vector<double> _;
//        kdtree->SearchKNN(query_point, neighbors, indices, _);
//        assert(indices.size() >= 2 && indices[0] == i);
//        indices.erase(indices.begin()); // Remove the query point itself
//
//        std::vector<Eigen::Triplet<double>>& thread_triplets = triplets_per_thread[omp_get_thread_num()];
//        double spring_force = 0.0;
//        for (const int &index : indices) {
//            Eigen::Vector3d a = ptsPtr->row(index);
//            double distance = (query_point - a).norm();
//            double spring_stretch = distance - spring_length;
//            if (spring_stretch > 0.0) {
//                spring_force += spring_constant * spring_stretch;
//            }
//        }
//        thread_triplets.emplace_back(i, i, spring_force);
//    }
//
//    // Combine all thread-local triplets into the main triplet list
//    std::vector<Eigen::Triplet<double>> all_triplets;
//    for (const std::vector<Eigen::Triplet<double>>& thread_triplets : triplets_per_thread) {
//        all_triplets.insert(all_triplets.end(), thread_triplets.begin(), thread_triplets.end());
//    }
//
//    S_Ptr_->setZero();
//    S_Ptr_->setFromTriplets(all_triplets.begin(), all_triplets.end());
//
//    auto end = std::chrono::high_resolution_clock::now();
//    std::chrono::duration<double> elapsed = end - start;
//    std::cout << "Spring system has been established! Elapsed time: " << elapsed.count() << "s" << std::endl;
//}

void Skeleton::EstablishSpringSystem(const std::shared_ptr<Eigen::MatrixXd> &ptsPtr, int neighbors,
                                     const double spring_constant, const double spring_length) {
    auto start = std::chrono::high_resolution_clock::now();
    const int n = static_cast<int>(ptsPtr->rows());

    std::shared_ptr<PointCloud> pts_cloudPtr(new PointCloud);
    pts_cloudPtr->points_.reserve(n);
    for (int i = 0; i < n; ++i) {
        pts_cloudPtr->points_.emplace_back(ptsPtr->row(i));
    }
    std::shared_ptr<KDTreeFlann> kdtree(new KDTreeFlann);
    kdtree->SetGeometry(*pts_cloudPtr);

    neighbors = neighbors + 1; // Open3D KDTreeFlann search includes the query point itself
    S_Ptr_->setZero();

    for (int i = 0; i < n; ++i) {
        Eigen::Vector3d query_point = ptsPtr->row(i);
        std::vector<int> indices;
        std::vector<double> dists;
        kdtree->SearchKNN(query_point, neighbors, indices, dists);

        assert(indices.size() >= 2 && indices[0] == i);
        indices.erase(indices.begin()); // Remove the query point itself

        double spring_force = 0.0;
        for (const int &index: indices) {
            Eigen::Vector3d a = ptsPtr->row(index);
            double distance = (query_point - a).norm();
            if (distance < spring_length) {  // Using spring_length as threshold
                spring_force += spring_constant * std::abs(distance - spring_length);
            }
        }
        S_Ptr_->coeffRef(i) = spring_force;
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Spring system has been established! Elapsed time: " << elapsed.count() << "s" << std::endl;
}

//void Skeleton::EstablishSpringSystem(const std::shared_ptr<Eigen::MatrixXd>& ptsPtr, int neighbors,
//                                     const double spring_constant, const double spring_length) {
//    auto start = std::chrono::high_resolution_clock::now();
//    const int n = static_cast<int>(ptsPtr->rows());
//
//    S_Ptr_->setZero();
//    S_Ptr_->fill(0.0);
//
//    #pragma omp parallel for default(none) shared(n, ptsPtr, spring_length, spring_constant)
//    for (int i = 0; i < n; ++i) {
//        Eigen::Vector3d query_point = ptsPtr->row(i);
//        std::vector<int> indices = S_neighborsPtr_->at(i);
//
//        double spring_force = 0.0;
//        for (const int &index : indices) {
//            Eigen::Vector3d a = ptsPtr->row(index);
//            double distance = (query_point - a).norm();
//            if (distance > spring_length) {  // Using spring_length as threshold
//                spring_force += spring_constant * (distance - spring_length);
//            }
//        }
//        #pragma omp critical
//        {
//            S_Ptr_->coeffRef(i) = spring_force;
//        }
//    }
//
//    auto end = std::chrono::high_resolution_clock::now();
//    std::chrono::duration<double> elapsed = end - start;
//    std::cout << "Spring system has been established! Elapsed time: " << elapsed.count() << "s" << std::endl;
//}


void Skeleton::InitializeParameters(const std::shared_ptr<Eigen::MatrixXd> &ptsPtr) {
    const int n = static_cast<int>(ptsPtr->rows());

    WL_Ptr_->resize(n);
    WS_Ptr_->resize(n);
    WH_Ptr_->resize(n);

    // Compute initial WL, WS & WH
    WL_Ptr_->fill(9.0); // 1.0 / (10.0 * mean_extents.mean())
    WS_Ptr_->fill(3.0);
    WH_Ptr_->fill(1.0);

    Skeleton::EstablishLaplacianMatrix(ptsPtr);
}


Eigen::MatrixXd Skeleton::LaplacianContraction(const std::shared_ptr<Eigen::MatrixXd> &ptsPtr) {
    auto start = std::chrono::high_resolution_clock::now();

    const int n = static_cast<int>(ptsPtr->rows());
    Eigen::SparseMatrix<double, Eigen::RowMajor> A(2 * n, n);
    std::vector<Eigen::Triplet<double>> A_tripletList;

    // Construct the matrix A (3n x n) { A = [L.*WL; S.*WS; sparse(1:P.npts, 1:P.npts, WH)]; }
    for (int i = 0; i < n; ++i) {
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(*L_Ptr_, i); it; ++it) {
            it.valueRef() *= WL_Ptr_->coeffRef(i);
            A_tripletList.emplace_back(it.row(), it.col(), it.value());
        }

//        double S_value = S_Ptr_->coeffRef(i) * WS_Ptr_->coeffRef(i);
//        A_tripletList.emplace_back(n + i, i, S_value);

        double WH_value = WH_Ptr_->coeffRef(i);
        A_tripletList.emplace_back(n + i, i, WH_value);
    }
    A.setFromTriplets(A_tripletList.begin(), A_tripletList.end());

    // Construct vector b (3n x 3) { b = [zeros(P.npts, 3); zeros(P.npts, 3); sparse(1:P.npts, 1:P.npts, WH)*P.pts]; }
    Eigen::MatrixXd zeros = Eigen::MatrixXd::Zero(n, 3);
    Eigen::MatrixXd WHP = WH_Ptr_->asDiagonal() * (*ptsPtr);
    Eigen::MatrixXd b(2 * n, 3);
    b << zeros, WHP;

    // Solve the linear system
    Eigen::SparseMatrix<double> ATA = A.transpose() * A;
    Eigen::MatrixXd ATb = A.transpose() * b;
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(ATA);
    if (solver.info() != Eigen::Success) {
        std::cerr << "Error: Eigen decomposition failed!" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    Eigen::MatrixXd cpts = solver.solve(ATb);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Linear system has been solved! Elapsed time: " << elapsed.count() << "s" << std::endl;

    return cpts;
}


void Skeleton::ComputeSigma(const std::shared_ptr<Eigen::MatrixXd> &ptsPtr, const std::string &neighborhood) {
    auto start = std::chrono::high_resolution_clock::now();

    const int n = static_cast<int>(ptsPtr->rows());
    sigma_.clear();
    sigma_.resize(n);
    smooth_sigma_.clear();
    smooth_sigma_.resize(n);

    if (neighborhood == "one_ring") {
#pragma omp parallel for default(none) shared(n, ptsPtr)
        for (int i = 0; i < n; ++i) {
            std::shared_ptr<PointCloud> one_ring_neighbors_cloudPtr = std::make_shared<PointCloud>();
            one_ring_neighbors_cloudPtr->points_.emplace_back(ptsPtr->row(i)); // Add the query point itself
            for (Ring &ring: verticesPtr_->at(i).get_ring()) {
                one_ring_neighbors_cloudPtr->points_.emplace_back(ptsPtr->row(ring.id));
            }

            Eigen::Vector3d mean;
            Eigen::Matrix3d covariance;
            std::tie(mean, covariance) = one_ring_neighbors_cloudPtr->ComputeMeanAndCovariance();
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
            // eigen_vectors The columns are inverted, col(2).value > col(1).value > col(0).value
            Eigen::Vector3d eigen_values = eigen_solver.eigenvalues();
            double lambda_0 = eigen_values(0);
            double lambda_1 = eigen_values(1);
            double lambda_2 = eigen_values(2);
            double sigma = lambda_2 / (lambda_0 + lambda_1 + lambda_2);
            sigma_[i] = sigma;
        }

        for (int i = 0; i < n; ++i) {
            for (Ring &ring: verticesPtr_->at(i).get_ring()) {
                smooth_sigma_[i] += sigma_[ring.id];
            }
            smooth_sigma_[i] += sigma_[i]; // Add the query point itself
            smooth_sigma_[i] /= int(verticesPtr_->at(i).get_ring().size()) + 1; // +1 for the query point itself
        }
    } else if (neighborhood == "radius" || neighborhood == "knn") {
        std::shared_ptr<PointCloud> pts_cloudPtr(new PointCloud);
        for (int i = 0; i < n; ++i) {
            pts_cloudPtr->points_.emplace_back(ptsPtr->row(i));
        }
        std::shared_ptr<KDTreeFlann> kdtree(new KDTreeFlann);
        kdtree->SetGeometry(*pts_cloudPtr);

        std::vector<std::vector<int>> neighbors_indices(n);

#pragma omp parallel for default(none) shared(n, ptsPtr, neighborhood, kdtree, neighbors_indices, pts_cloudPtr)
        for (int i = 0; i < n; ++i) {
            Eigen::Vector3d query_point = ptsPtr->row(i);
            std::vector<int> indices;
            std::vector<double> _;
            if (neighborhood == "radius") {
                kdtree->SearchRadius(query_point, 0.015 * diagonal_length_, indices, _);
            } else if (neighborhood == "knn") {
                kdtree->SearchKNN(query_point, 8, indices, _);
            }
            neighbors_indices[i] = indices;

            std::shared_ptr<PointCloud> neighbors_cloudPtr = tool::utility::SelectByIndices(pts_cloudPtr, indices);
            Eigen::Vector3d mean;
            Eigen::Matrix3d covariance;
            std::tie(mean, covariance) = neighbors_cloudPtr->ComputeMeanAndCovariance();
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
            // eigen_vectors The columns are inverted, col(2).value > col(1).value > col(0).value
            Eigen::Vector3d eigen_values = eigen_solver.eigenvalues();
            double lambda_0 = eigen_values(0);
            double lambda_1 = eigen_values(1);
            double lambda_2 = eigen_values(2);
            sigma_[i] = lambda_2 / (lambda_0 + lambda_1 + lambda_2);
        }

        for (int i = 0; i < n; ++i) {
            for (int &index: neighbors_indices[i]) {
                smooth_sigma_[i] += sigma_[index];
            }
            smooth_sigma_[i] += sigma_[i];
            smooth_sigma_[i] /= int(neighbors_indices[i].size()) + 1;
        }
    } else if (neighborhood == "weighted") {
        std::shared_ptr<PointCloud> pts_cloudPtr(new PointCloud);
        for (int i = 0; i < n; ++i) {
            pts_cloudPtr->points_.emplace_back(ptsPtr->row(i));
        }
        std::shared_ptr<KDTreeFlann> kdtree(new KDTreeFlann);
        kdtree->SetGeometry(*pts_cloudPtr);

        std::vector<std::vector<int>> neighbors_indices(n);

#pragma omp parallel for default(none) shared(n, ptsPtr, kdtree, neighbors_indices)
        for (int i = 0; i < n; ++i) {
            Eigen::Vector3d query_point = ptsPtr->row(i);
            std::vector<int> indices;
            std::vector<double> _;
            double radius = 0.015 * diagonal_length_;
            kdtree->SearchRadius(query_point, radius, indices, _);

            assert(indices.size() >= 2 && indices[0] == i);
            indices.erase(indices.begin()); // Remove the query point itself

            neighbors_indices[i] = indices;
            Eigen::MatrixXd Ci = Eigen::MatrixXd::Zero(3, 3);
            // Compute Ci for Pi using its neighborhood.
            for (const int &index: indices) {
                Eigen::Vector3d pj = ptsPtr->row(index); // Get the neighboring point Pj.
                Eigen::VectorXd diff = query_point - pj;
                // Compute the weight based on the distance and a theta function.
                double weight = (query_point - pj).cwiseAbs().sum();
                weight = exp(-pow(weight, 2) / pow(radius / 4, 2));
                // Add the weighted outer product to the covariance matrix Ci.
                Ci += weight * diff * diff.transpose();
            }
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(Ci);
            // eigen_vectors The columns are inverted, col(2).value > col(1).value > col(0).value
            Eigen::Vector3d eigen_values = eigen_solver.eigenvalues();
            double lambda_0 = eigen_values(0);
            double lambda_1 = eigen_values(1);
            double lambda_2 = eigen_values(2);
            sigma_[i] = lambda_2 / (lambda_0 + lambda_1 + lambda_2);
        }
#pragma omp parallel for default(none) shared(n, neighbors_indices)
        for (int i = 0; i < n; ++i) {
            smooth_sigma_[i] += sigma_[i];
            for (const int &index: neighbors_indices[i]) {
                smooth_sigma_[i] += sigma_[index];
            }
            smooth_sigma_[i] /= int(neighbors_indices[i].size()) + 1;
        }
    } else {
        throw std::runtime_error("Invalid neighborhood method!");
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Smooth sigmas have been updated! Elapsed time: " << elapsed.count() << "s" << std::endl;
}


// The main function for Laplacian skeletonization
std::shared_ptr<Eigen::MatrixXd> Skeleton::ContractionIteration() {
    std::shared_ptr<Eigen::MatrixXd> _ = std::make_shared<Eigen::MatrixXd>(ptsPtr_->rows(), 1);
    for (int i = 0; i < ptsPtr_->rows(); ++i) {
        _->coeffRef(i, 0) = smooth_sigma_[i];
    }
    std::vector<std::pair<std::string, std::string>> __ = {{"double", "smooth_sigma"}};
    tool::io::SavePointCloudToPLY(output_path_ + "_cpts_0.ply", ptsPtr_, _, __);

    // Start first time contraction
    std::cout << "--------------------------------------------------" << std::endl;
    std::cout << "Contraction iteration time: 1" << std::endl;

    // Initialize parameters
    FindOneRingNeighbors(cloudPtr_, true);
    EstablishSpringSystem(ptsPtr_, 8, 3.0, 0.001 * diagonal_length_);
    InitializeParameters(ptsPtr_);

    // Contract first time
    Eigen::MatrixXd cpts = Skeleton::LaplacianContraction(ptsPtr_);
    std::shared_ptr<Eigen::MatrixXd> cptsPtr = std::make_shared<Eigen::MatrixXd>(cpts);
    ComputeSigma(cptsPtr, "weighted");

    // Save the contracted points
    _ = std::make_shared<Eigen::MatrixXd>(cptsPtr->rows(), 1);
    for (int i = 0; i < cptsPtr->rows(); ++i) {
        _->coeffRef(i, 0) = smooth_sigma_[i];
    }
    tool::io::SavePointCloudToPLY(output_path_ + "_cpts_1.ply", cptsPtr, _, __);

    // Update neighborhood extent
    std::vector<double> contraction_history;
    const Eigen::VectorXd extents = Skeleton::GetNeighborhoodExtent(ptsPtr_, "min");
    Eigen::VectorXd new_extents = Skeleton::GetNeighborhoodExtent(cptsPtr, "min");
    double contraction_rate = new_extents.sum() / extents.sum();
    contraction_history.emplace_back(contraction_rate);
    std::cout << "Current extent is: " << new_extents.sum() << "; "
              << "the contract ratio is: " << contraction_rate << std::endl;

    // Start the contraction iterations
    bool final_flag = false;
    for (int i = 0; i < iteration_time_ - 1; ++i) {
        std::cout << "--------------------------------------------------" << std::endl;
        std::cout << "Contraction iteration time: " << i + 2 << std::endl;

        // Update one ring neighbors
        std::shared_ptr<PointCloud> cpts_cloudPtr = std::make_shared<PointCloud>();
        for (int j = 0; j < cptsPtr->rows(); ++j) {
            cpts_cloudPtr->points_.emplace_back(cptsPtr->row(j));
        }
        if (!final_flag) {
            UpdateKNeighbors();
        }
        if (i == 0) {
            FindOneRingNeighbors(cpts_cloudPtr, true);
        } else {
            FindOneRingNeighbors(cpts_cloudPtr, false);
        }

        // Update spring system
        EstablishSpringSystem(cptsPtr, 8, 3.0, 0.001 * diagonal_length_);

        // Fix the points with high sigma --- Current Version
        if (!final_flag) {
            for (int j = 0; j < cptsPtr->rows(); ++j) {
                if (smooth_sigma_[j] >= fix_eigen_ratio_) { //  || WH_Ptr_->coeffRef(j) >= 3.0
                    double temp_WH = WH_Ptr_->coeffRef(j) * 6.0;
                    if (temp_WH > 2048) {
                        WH_Ptr_->coeffRef(j) = 2048;
                    } else {
                        WH_Ptr_->coeffRef(j) = temp_WH;
                    }
                    WL_Ptr_->coeffRef(j) = 0.0;
                } else {
                    double temp_WL = WL_Ptr_->coeffRef(j) * sL_;
                    if (temp_WL > 2048) {
                        WL_Ptr_->coeffRef(j) = 2048;
                    } else {
                        WL_Ptr_->coeffRef(j) = temp_WL;
                    }
                }
            }

            // Update Laplacian matrix
            Skeleton::EstablishLaplacianMatrix(cptsPtr);
        } else {
            // Utilize default parameters
            InitializeParameters(cptsPtr);
        }

        // Contract one more time
        Eigen::MatrixXd cpts_temp = Skeleton::LaplacianContraction(cptsPtr);
        std::shared_ptr<Eigen::MatrixXd> cpts_tempPtr = std::make_shared<Eigen::MatrixXd>(cpts_temp);
        ComputeSigma(cpts_tempPtr, "weighted");

        // Update neighborhood extent
        new_extents = Skeleton::GetNeighborhoodExtent(cpts_tempPtr, "min");
        contraction_rate = new_extents.sum() / extents.sum();
        contraction_history.emplace_back(contraction_rate);

        if (final_flag)
            std::cout << "Final extent is: " << new_extents.sum() << "; "
                      << "the final contract ratio is: " << contraction_rate << std::endl;
        else {
            std::cout << "Current extent is: " << new_extents.sum() << "; "
                      << "the contract ratio is: " << contraction_rate << std::endl;
        }

        if (final_flag) {
            // Save the contracted points
            std::shared_ptr<Eigen::MatrixXd> _ = std::make_shared<Eigen::MatrixXd>(cpts_tempPtr->rows(), 1);
            for (int j = 0; j < cpts_tempPtr->rows(); ++j) {
                _->coeffRef(j, 0) = smooth_sigma_[j];
            }
            tool::io::SavePointCloudToPLY(output_path_ + "_cpts_" + std::to_string(i + 2) + ".ply", cpts_tempPtr, _,
                                          __);

            *cptsPtr = *cpts_tempPtr;
            break;
        } else if (contraction_history[i] - contraction_history.back() <= 0.0 ||
                   std::isnan(contraction_history.back())) {
            std::cout << "Touch the threshold! Iteration " << i + 2 << " terminated! Total contraction iteration time: "
                      << i + 1 << std::endl;
            std::cout << "--------------------------------------------------" << std::endl;
            std::cout << "Begin final contraction" << std::endl;

            final_flag = true;
            k_neighborsPtr_->fill(8);

            continue;
        } else {
            // Save the contracted points
            std::shared_ptr<Eigen::MatrixXd> _ = std::make_shared<Eigen::MatrixXd>(cpts_tempPtr->rows(), 1);
            for (int j = 0; j < cpts_tempPtr->rows(); ++j) {
                _->coeffRef(j, 0) = smooth_sigma_[j];
            }
            tool::io::SavePointCloudToPLY(output_path_ + "_cpts_" + std::to_string(i + 2) + ".ply", cpts_tempPtr, _,
                                          __);

            // Update cptsPtr
            *cptsPtr = *cpts_tempPtr;
            sL_ = sL_ * 1.20 < 3.0 ? sL_ * 1.20 : 3.0;
        }
    }

    return cptsPtr;
}

