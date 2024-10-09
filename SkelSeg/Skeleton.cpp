#include "Skeleton.h"
#include "Tools.h"


void Skeleton::InitializeParameters(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr) {
    // Initialize Laplacian matrix
    EstablishLaplacianMatrix(cloudPtr);

    auto start = std::chrono::high_resolution_clock::now();

    WL_Ptr_->resize(pts_num_);
    WH_Ptr_->resize(pts_num_);

    // Initialize WL, WH
    WL_Ptr_->fill(initial_WL_);
    WH_Ptr_->fill(initial_WH_);
    for (int tip_index: tip_indices_) {
        WH_Ptr_->coeffRef(tip_index) = tip_point_WH_;
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    MyLogger.Log(std::format("Weight matrices have been initialized! Elapsed time: {:.6f}s.", elapsed.count()), 0, true,
                 false);
}


void Skeleton::EstablishLaplacianMatrix(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr) {
    L_Ptr_->setZero();

    std::shared_ptr<geometrycentral::pointcloud::PointCloud> gc_cloudPtr = std::make_shared<geometrycentral::pointcloud::PointCloud>(
            pts_num_);
    std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> gc_geom = std::make_shared<geometrycentral::pointcloud::PointPositionGeometry>(
            *gc_cloudPtr);
    std::vector<geometrycentral::Vector3> vertices_positions = tool::utility::Matrix2GCVector(cloudPtr);
    for (int i = 0; i < pts_num_; ++i) {
        gc_geom->positions[i] = vertices_positions[i];
    }
    gc_geom->kNeighborSize = k_neighbors_;

    // threshold_edge_length_ is utilized as a flag to determine whether the tip points/edge threshold are found
    // and there is no need to CollapseLargeFaces at first iteration, since the k_neighbors_ is small
    if (threshold_edge_length_ < 0.0) {
        FindTipPoints(cloudPtr, gc_cloudPtr, gc_geom);
        FindEdgeThreshold(gc_cloudPtr, gc_geom);
    } else {
        CollapseEdges(gc_cloudPtr, gc_geom);
    }

    auto start = std::chrono::high_resolution_clock::now();

    // Compute the Laplacian matrix
    gc_geom->requireLaplacian();
    *L_Ptr_ = -(2 * gc_geom->laplacian);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    MyLogger.Log(std::format("Laplacian matrix has been established! Elapsed time: {:.6f}s.", elapsed.count()), 0, true,
                 false);
}


void Skeleton::FindTipPoints(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr,
                             const std::shared_ptr<geometrycentral::pointcloud::PointCloud> &gc_cloudPtr,
                             const std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> &gc_geom) {
    auto start = std::chrono::high_resolution_clock::now();

    // Find the lowest vertex index
    Eigen::VectorXd z_values = cloudPtr->col(2);
    int min_index;
    z_values.minCoeff(&min_index);
    geometrycentral::pointcloud::PointCloudHeatSolver solver(*gc_cloudPtr, *gc_geom);
    geometrycentral::pointcloud::Point pSource = gc_cloudPtr->point(min_index);
    geometrycentral::pointcloud::PointData<double> distance = solver.computeDistance(pSource);
    std::vector<std::vector<size_t>> neighbors_indices;
    neighbors_indices = tool::utility::KNNSearch(cloudPtr, 16);
    tip_indices_.clear();
    tip_indices_.emplace_back(min_index);
    for (int i = 0; i < pts_num_; ++i) {
        double center_value = distance[i];
        std::vector<double> neighbor_values;
        for (const size_t &index: neighbors_indices[i]) {
            neighbor_values.emplace_back(distance[index]);
        }
        double max_neighbor_value = *std::max_element(neighbor_values.begin(), neighbor_values.end());
        if (center_value > max_neighbor_value) {
            tip_indices_.emplace_back(i);
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    MyLogger.Log(std::format("Tip points have been found by using Geodesic distance! Elapsed time: {:.6f}s.",
                             elapsed.count()), 0, true, false);
}


void Skeleton::FindEdgeThreshold(const std::shared_ptr<geometrycentral::pointcloud::PointCloud> &gc_cloudPtr,
                                 const std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> &gc_geom) {
    auto start = std::chrono::high_resolution_clock::now();

    // Generate the local triangles
    geometrycentral::pointcloud::PointData<std::vector<std::array<geometrycentral::pointcloud::Point, 3>>> local_tri_point = geometrycentral::pointcloud::buildLocalTriangulations(
            *gc_cloudPtr, *gc_geom, true);
    // Make a union of local triangles
    std::vector<std::vector<size_t>> all_tris = handleToFlatInds(*gc_cloudPtr, local_tri_point);
    std::vector<geometrycentral::Vector3> pos_raw(gc_cloudPtr->nPoints());
    for (size_t iP = 0; iP < pos_raw.size(); iP++) {
        pos_raw[iP] = gc_geom->positions[iP];
    }
    // Make a mesh for threshold edge detection
    std::unique_ptr<geometrycentral::surface::SurfaceMesh> temp_mesh;
    std::unique_ptr<geometrycentral::surface::VertexPositionGeometry> temp_geom;
    std::tie(temp_mesh, temp_geom) = geometrycentral::surface::makeSurfaceMeshAndGeometry(all_tris, pos_raw);
    temp_geom->requireEdgeLengths();
    Eigen::VectorXd edges_lengths = temp_geom->edgeLengths.toVector();
    std::vector<double> edges_lengths_vec(edges_lengths.data(), edges_lengths.data() + edges_lengths.size());
    // Find the long edges
    std::vector<int> indices = tool::utility::FindUpperOutlierBySTD(edges_lengths_vec);
    Eigen::VectorXd long_edges_lengths = Eigen::VectorXd::Zero(indices.size());
    for (int i = 0; i < indices.size(); ++i) {
        long_edges_lengths[i] = edges_lengths[indices[i]];
    }
    threshold_edge_length_ = long_edges_lengths.minCoeff();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    MyLogger.Log(std::format("Threshold edge length has been initialized! Elapsed time: {:.6f}s.", elapsed.count()), 0,
                 true, false);
}


void Skeleton::CollapseEdges(std::shared_ptr<geometrycentral::pointcloud::PointCloud> &gc_cloudPtr,
                             std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> &gc_geom) {
    auto start = std::chrono::high_resolution_clock::now();

    // Initialize the tuftedMesh and tuftedGeom
    std::unique_ptr<geometrycentral::surface::SurfaceMesh> tuftedMesh;
    std::unique_ptr<geometrycentral::surface::EdgeLengthGeometry> tuftedGeom;

    // Generate the local triangles
    geometrycentral::pointcloud::PointData<std::vector<std::array<geometrycentral::pointcloud::Point, 3>>> local_tri_point = geometrycentral::pointcloud::buildLocalTriangulations(
            *gc_cloudPtr, *gc_geom, true);

    // Make a union of local triangles
    std::vector<std::vector<size_t>> all_tris = handleToFlatInds(*gc_cloudPtr, local_tri_point);
    std::vector<geometrycentral::Vector3> pos_raw(gc_cloudPtr->nPoints());
#pragma omp parallel for
    for (size_t iP = 0; iP < pos_raw.size(); iP++) {
        pos_raw[iP] = gc_geom->positions[iP];
    }

    // Make a mesh for edge collapse detection
    std::unique_ptr<geometrycentral::surface::SurfaceMesh> temp_mesh;
    std::unique_ptr<geometrycentral::surface::VertexPositionGeometry> temp_geom;
    std::tie(temp_mesh, temp_geom) = geometrycentral::surface::makeSurfaceMeshAndGeometry(all_tris, pos_raw);

    // Find the Edge to be collapsed, and its adjacent faces
    temp_geom->requireEdgeLengths();
    geometrycentral::surface::EdgeData<double> edge_lengths = temp_geom->edgeLengths;
    std::vector<std::vector<size_t>> faces_to_ignore;
#pragma omp parallel for
    for (size_t i = 0; i < temp_mesh->nVertices(); i++) {
        geometrycentral::surface::Vertex vert = temp_mesh->vertex(i);
        std::vector<geometrycentral::surface::Edge> adjacent_edges;
        std::vector<double> adjacent_edges_length;
        for (geometrycentral::surface::Edge e: vert.adjacentEdges()) {
            adjacent_edges.emplace_back(e);
            adjacent_edges_length.emplace_back(edge_lengths[e]);
        }
        std::vector<int> edge_indices;
        for (int j = 0; j < adjacent_edges_length.size(); ++j) {
            if (adjacent_edges_length[j] >= threshold_edge_length_) {
                edge_indices.emplace_back(j);
            }
        }
        if (edge_indices.empty()) {
            continue;
        } else {
            for (int edge_index: edge_indices) {
                geometrycentral::surface::Edge edge = adjacent_edges[edge_index];
                // Mark the faces that consist of the long edge
                for (geometrycentral::surface::Face f: edge.adjacentFaces()) {
                    std::vector<size_t> face;
                    for (geometrycentral::surface::Vertex v: f.adjacentVertices()) {
                        face.emplace_back(v.getIndex());
                    }
#pragma omp critical
                    {
                        faces_to_ignore.emplace_back(face);
                    }
                }
            }
        }
    }

    // Remove the ignored faces
    std::unordered_set<std::vector<size_t>, VectorHash> ignore_set(faces_to_ignore.begin(), faces_to_ignore.end());
    all_tris.erase(std::remove_if(all_tris.begin(), all_tris.end(),
                                  [&ignore_set](const std::vector<size_t> &face) {
                                      return ignore_set.find(face) != ignore_set.end();
                                  }), all_tris.end());

    // TODO: Check the logic here
    // Check for unreferenced vertices
    size_t max_index = pos_raw.size();
    std::vector<bool> referenced(max_index, false);
#pragma omp parallel
    {
        std::vector<bool> local_referenced(max_index, false);
#pragma omp for nowait
        for (const std::vector<size_t> &tri: all_tris) {
            for (size_t vert_idx: tri) {
                if (vert_idx < max_index) {
                    local_referenced[vert_idx] = true;
                }
            }
        }
#pragma omp critical
        {
            for (size_t i = 0; i < max_index; ++i) {
                if (local_referenced[i]) {
                    referenced[i] = true;
                }
            }
        }
    }
    std::vector<size_t> unreferenced_vertices;
    for (size_t i = 0; i < max_index; ++i) {
        if (!referenced[i]) {
            unreferenced_vertices.push_back(i);
        }
    }

    // Build a triangle with two nearest vertices to deal with the unreferenced vertices
    if (!unreferenced_vertices.empty()) {
        geometrycentral::NearestNeighborFinder finder(pos_raw);
        for (size_t vertex: unreferenced_vertices) {
            std::vector<size_t> nearest_indices = finder.kNearestNeighbors(vertex, 2);
            std::vector<size_t> nearest_triangle = {vertex, nearest_indices[0], nearest_indices[1]};
            all_tris.emplace_back(nearest_triangle);
        }
    }

    // Make a mesh, read off its
    std::unique_ptr<geometrycentral::surface::VertexPositionGeometry> posGeom;
    std::tie(tuftedMesh, posGeom) = geometrycentral::surface::makeSurfaceMeshAndGeometry(all_tris, pos_raw);
    posGeom->requireEdgeLengths();
    geometrycentral::surface::EdgeData<double> tuftedEdgeLengths = posGeom->edgeLengths;
    // Mollify
    mollifyIntrinsic(*tuftedMesh, tuftedEdgeLengths, 1e-5);
    // Build the cover
    buildIntrinsicTuftedCover(*tuftedMesh, tuftedEdgeLengths);
    flipToDelaunay(*tuftedMesh, tuftedEdgeLengths);
    // Create the geometry object
    tuftedGeom.reset(new geometrycentral::surface::EdgeLengthGeometry(*tuftedMesh, tuftedEdgeLengths));
    // Set tuftedTriangulationQ.require() to true
    gc_geom->requireTuftedTriangulation();
    // Replace the tuftedMesh and tuftedGeom
    gc_geom->tuftedMesh = std::move(tuftedMesh);
    gc_geom->tuftedGeom = std::move(tuftedGeom);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    MyLogger.Log(std::format("Long edges have been ignored! Elapsed time: {:.6f}s.", elapsed.count()), 0, true, false);
}


std::tuple<Eigen::SparseMatrix<double>, Eigen::MatrixXd>
Skeleton::DistanceConstraint(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr) {
    auto start = std::chrono::high_resolution_clock::now();

    // Initialize PointCloud and Geometry
    auto gc_cloudPtr = std::make_shared<geometrycentral::pointcloud::PointCloud>(pts_num_);
    auto gc_geom = std::make_shared<geometrycentral::pointcloud::PointPositionGeometry>(*gc_cloudPtr);

    // Convert and set vertex positions
    std::vector<geometrycentral::Vector3> vertices_positions = tool::utility::Matrix2GCVector(cloudPtr);
    for (int i = 0; i < pts_num_; ++i) {
        gc_geom->positions[i] = vertices_positions[i];
    }
    gc_geom->kNeighborSize = k_neighbors_distance_constraint_;
    gc_geom->requireLaplacian();
    gc_geom->tuftedGeom->requireEdgeLengths();

    Eigen::VectorXd edge_lengths = gc_geom->tuftedGeom->edgeLengths.toVector();

    // Find edges and their constraints
    std::vector<std::pair<int, int>> edges;
    std::vector<geometrycentral::Vector3> constraint_lengths;
    Eigen::SparseMatrix<double> laplacian = gc_geom->laplacian;
    for (int k = 0; k < laplacian.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(laplacian, k); it; ++it) {
            if (it.row() < it.col()) { // Process each edge once
                geometrycentral::Vector3 point_i = gc_geom->positions[it.row()];
                geometrycentral::Vector3 point_j = gc_geom->positions[it.col()];
                double distance = (point_i - point_j).norm();

                if (distance > max_distance_) {
                    geometrycentral::Vector3 e_i_j = (max_distance_ / distance) * (point_i - point_j);
                    constraint_lengths.emplace_back(e_i_j);
                } else {
                    geometrycentral::Vector3 e_i_j = (1.0) * (point_i - point_j);
                    constraint_lengths.emplace_back(e_i_j);
                }
                edges.emplace_back(it.row(), it.col());
            }
        }
    }

    // Convert constraint lengths to Eigen::MatrixXd
    Eigen::MatrixXd e(edges.size(), 3);
    for (size_t i = 0; i < edges.size(); ++i) {
        e.row(int(i)) = Eigen::Vector3d{constraint_lengths[i].x, constraint_lengths[i].y, constraint_lengths[i].z};
    }

    // Create edge matrix H
    Eigen::SparseMatrix<double> H(edges.size(), pts_num_);
    std::vector<Eigen::Triplet<double>> tripletList;
    for (size_t i = 0; i < edges.size(); ++i) {
        tripletList.emplace_back(i, edges[i].first, 1.0);
        tripletList.emplace_back(i, edges[i].second, -1.0);
    }
    H.setFromTriplets(tripletList.begin(), tripletList.end());

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    MyLogger.Log(std::format("Distance constraints have been built! Elapsed time: {:.6f}s.", elapsed.count()), 0, true,
                 false);

    return std::make_tuple(H, e);
}


Eigen::MatrixXd Skeleton::LaplacianContraction(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr) {
    auto start = std::chrono::high_resolution_clock::now();

    Eigen::SparseMatrix<double> H;
    Eigen::MatrixXd e;
    std::tie(H, e) = DistanceConstraint(cloudPtr);

    // Construct the matrix A (2pts_num_+H x pts_num_) { A = [L.*WL; sparse(1:P.npts, 1:P.npts, WH); sparse(1:P.npts, 1:P.npts, H)]}
    std::vector<Eigen::Triplet<double>> A_tripletList;
    for (int i = 0; i < pts_num_; ++i) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(*L_Ptr_, i); it; ++it) {
            double value = it.value() * WL_Ptr_->coeffRef(it.row());
            A_tripletList.emplace_back(it.row(), it.col(), value);
        }
        A_tripletList.emplace_back(pts_num_ + i, i, WH_Ptr_->coeffRef(i));
    }
    for (int i = 0; i < H.outerSize(); ++i) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(H, i); it; ++it) {
            double value = it.value() * WD_;
            A_tripletList.emplace_back(2 * pts_num_ + it.row(), it.col(), value);
        }
    }
    Eigen::SparseMatrix<double> A(2 * pts_num_ + H.rows(), pts_num_);
    A.setFromTriplets(A_tripletList.begin(), A_tripletList.end());

    // Construct vector b (2pts_num_+e x 3) { b = [zeros(P.npts, 3); [(1:P.npts, 1:P.npts, WH)*P.pts]; e]}
    Eigen::MatrixXd zeros = Eigen::MatrixXd::Zero(pts_num_, 3);
    Eigen::MatrixXd WHP = WH_Ptr_->asDiagonal() * (*cloudPtr);
    e *= WD_;
    Eigen::MatrixXd b(2 * pts_num_ + e.rows(), 3);
    b << zeros, WHP, e;

    // Solve the linear system
    Eigen::SparseMatrix<double> ATA = A.transpose() * A;
    Eigen::MatrixXd ATb = A.transpose() * b;
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver;
    solver.compute(ATA);
    if (solver.info() != Eigen::Success) {
        MyLogger.Log(" Eigen decomposition failed!", 1, true, true);
        std::exit(EXIT_FAILURE);
    }
    Eigen::MatrixXd cpts = solver.solve(ATb);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    MyLogger.Log(std::format("Linear system has been solved! Elapsed time: {:.6f}s.", elapsed.count()), 0, true, false);

    return cpts;
}


void Skeleton::GetMeanVertexDualArea(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr) {
    auto start = std::chrono::high_resolution_clock::now();

    std::shared_ptr<geometrycentral::pointcloud::PointCloud> gc_cloudPtr = std::make_shared<geometrycentral::pointcloud::PointCloud>(
            pts_num_);
    std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> gc_geom = std::make_shared<geometrycentral::pointcloud::PointPositionGeometry>(
            *gc_cloudPtr);
    std::vector<geometrycentral::Vector3> vertices_positions = tool::utility::Matrix2GCVector(cloudPtr);
    for (int i = 0; i < pts_num_; ++i) {
        gc_geom->positions[i] = vertices_positions[i];
    }
    // Fixed value, for stable computation
    gc_geom->kNeighborSize = k_neighbors_dual_area_;

    gc_geom->requireTuftedTriangulation();
    geometrycentral::surface::IntrinsicGeometryInterface &geometry = *(gc_geom->tuftedGeom);
    geometry.requireVertexDualAreas();
    geometrycentral::surface::VertexData<double> dual_areas = geometry.vertexDualAreas;
    double total_area = dual_areas.toVector().mean();
    faces_area_.emplace_back(total_area);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    MyLogger.Log(std::format("Mean vertex dual area has been updated! Elapsed time: {:.6f}s.", elapsed.count()), 0,
                 true,
                 false);
}


void Skeleton::ComputeSigma(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, const std::string &neighborhood) {
    auto start = std::chrono::high_resolution_clock::now();

    sigmaPtr_->clear();
    sigmaPtr_->resize(pts_num_);
    smooth_sigmaPtr_->clear();
    smooth_sigmaPtr_->resize(pts_num_);

    std::vector<std::vector<size_t>> neighbors_indices;
    if (neighborhood == "knn" || neighborhood == "sigma_radius_") {
        if (neighborhood == "knn") {
            neighbors_indices = tool::utility::KNNSearch(cloudPtr, sigma_k_);
        } else if (neighborhood == "sigma_radius_") {
            neighbors_indices = tool::utility::RadiusSearch(cloudPtr, sigma_radius_);
        }
#pragma omp parallel for
        for (int i = 0; i < pts_num_; ++i) {
            std::vector<size_t> indices;
            // The first element is the query point itself
            indices.emplace_back(i);
            indices.insert(indices.end(), neighbors_indices[i].begin(), neighbors_indices[i].end());

            // If point does not meet these criteria, means it is quite isolated among its neighbors, just set to fix_eigen_ratio_
            if (!(indices.size() >= 2 && indices[0] == i)) {
                sigmaPtr_->at(i) = fix_eigen_ratio_;
                continue;
            }

            Eigen::MatrixXd neighbors_cloud = tool::utility::SelectByIndices(cloudPtr, neighbors_indices[i], 3);
            Eigen::Matrix3d covariance = tool::utility::ComputeCovariance(neighbors_cloud);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
            // For eigen_vectors, the columns are inverted, col(2).value > col(1).value > col(0).value
            Eigen::Vector3d eigen_values = eigen_solver.eigenvalues();
            double lambda_0 = eigen_values(0);
            double lambda_1 = eigen_values(1);
            double lambda_2 = eigen_values(2);
            sigmaPtr_->at(i) = lambda_2 / (lambda_0 + lambda_1 + lambda_2);
        }
#pragma omp parallel for
        for (int i = 0; i < pts_num_; ++i) {
            for (size_t &index: neighbors_indices[i]) {
                smooth_sigmaPtr_->at(i) += sigmaPtr_->at(index);
            }
            smooth_sigmaPtr_->at(i) += sigmaPtr_->at(i);
            smooth_sigmaPtr_->at(i) /= int(neighbors_indices[i].size()) + 1;
        }
    } else if (neighborhood == "weighted") {
        neighbors_indices = tool::utility::RadiusSearch(cloudPtr, sigma_radius_);
#pragma omp parallel for
        for (int i = 0; i < pts_num_; ++i) {
            Eigen::Vector3d query_point = cloudPtr->row(i);
            std::vector<size_t> indices = neighbors_indices[i];

            // If point does not meet these criteria, means it is quite isolated among its neighbors, just set to fix_eigen_ratio_
            if (!(indices.size() >= 2 && indices[0] == i)) {
                sigmaPtr_->at(i) = fix_eigen_ratio_;
                continue;
            }

            indices.erase(indices.begin()); // Remove the query point itself

            neighbors_indices[i] = indices;
            Eigen::MatrixXd Ci = Eigen::MatrixXd::Zero(3, 3);
            // Compute Ci for Pi using its neighborhood.
            for (const size_t &index: indices) {
                Eigen::Vector3d pj = cloudPtr->row(int(index)); // Get the neighboring point Pj.
                Eigen::VectorXd diff = query_point - pj;
                // Compute the weight based on the distance and a theta function.
                double weight = (query_point - pj).cwiseAbs().sum();
                weight = exp(-pow(weight, 2) / pow(sigma_radius_ / 4, 2));
                // Add the weighted outer product to the covariance matrix Ci.
                Ci += weight * diff * diff.transpose();
            }
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(Ci);
            // eigen_vectors The columns are inverted, col(2).value > col(1).value > col(0).value
            Eigen::Vector3d eigen_values = eigen_solver.eigenvalues();
            double lambda_0 = eigen_values(0);
            double lambda_1 = eigen_values(1);
            double lambda_2 = eigen_values(2);
            sigmaPtr_->at(i) = lambda_2 / (lambda_0 + lambda_1 + lambda_2);
        }
#pragma omp parallel for
        for (int i = 0; i < pts_num_; ++i) {
            smooth_sigmaPtr_->at(i) += sigmaPtr_->at(i);
            for (const size_t &index: neighbors_indices[i]) {
                smooth_sigmaPtr_->at(i) += sigmaPtr_->at(index);
            }
            smooth_sigmaPtr_->at(i) /= int(neighbors_indices[i].size()) + 1;
        }
    } else {
        MyLogger.Log("Invalid neighborhood searching method for Sigma!", 1, true, true);
        std::exit(EXIT_FAILURE);
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    MyLogger.Log(std::format("Smooth sigmas have been updated! Elapsed time: {:.6f}s.", elapsed.count()), 0, true,
                 false);
}


void Skeleton::UpdateKNeighbors() {
    k_neighbors_ = k_neighbors_ + delta_k_ > max_k_ ? max_k_ : k_neighbors_ + delta_k_;
    MyLogger.Log(std::format("k-nearest neighbors value has been updated! Current: {}.", k_neighbors_), 0, true, false);
}


// The main function for Laplacian skeletonization
std::shared_ptr<Eigen::MatrixXd> Skeleton::ContractionIteration() {
    // Save the original points
    tool::io::SavePointCloudToPLY(cloudPtr_, output_path_ / "_cpts_0.ply", binary_format_);

    // Start first time contraction
    MyLogger.Log("--------------------------------------------------", 0, true, false);
    MyLogger.Log("Contraction iteration time: 1", 0, true, false);

    // Initialize parameters
    InitializeParameters(cloudPtr_);

    // Contract for the first time
    Eigen::MatrixXd cpts = LaplacianContraction(cloudPtr_);
    std::shared_ptr<Eigen::MatrixXd> cptsPtr = std::make_shared<Eigen::MatrixXd>(cpts);

    // Check the contraction rate
    std::vector<double> contraction_history;
    GetMeanVertexDualArea(cloudPtr_);
    GetMeanVertexDualArea(cptsPtr);
    double contraction_rate = faces_area_.back() / faces_area_[0];
    contraction_history.emplace_back(contraction_rate);
    MyLogger.Log(
            std::format("Current mean area is: {:.6f}; the contract ratio is: {:.6f}.", faces_area_.back(),
                        contraction_rate),
            0, true, false);

    // Compute sigma for the contracted points
    ComputeSigma(cptsPtr, "weighted");

    // Save the contracted points
    std::vector<double> smooth_sigma;
    smooth_sigma.resize(pts_num_);
    for (int i = 0; i < cptsPtr->rows(); ++i) {
        smooth_sigma.at(i) = smooth_sigmaPtr_->at(i);
    }
    tool::io::SavePointCloudToPLY(cptsPtr, output_path_ / "_cpts_1.ply", smooth_sigma, "smooth_sigma", binary_format_);

    // Start the contraction iterations
    for (int i = 0; i < max_iteration_time_ - 1; ++i) {
        MyLogger.Log("--------------------------------------------------", 0, true, false);
        MyLogger.Log(std::format("Contraction iteration time: {}", i + 2), 0, true, false);

        // Update k_neighbors
        UpdateKNeighbors();

        // Update Laplacian matrix
        EstablishLaplacianMatrix(cptsPtr);

        // Fix the points with high sigma --- Current Version
        for (int j = 0; j < pts_num_; ++j) {
            if (WH_Ptr_->coeffRef(j) == tip_point_WH_ || WH_Ptr_->coeffRef(j) == contracted_point_WH_) {
                continue; // Skip the tip points
            } else if (smooth_sigmaPtr_->at(j) >= fix_eigen_ratio_) {
                WL_Ptr_->coeffRef(j) = contracted_point_WL_;
                WH_Ptr_->coeffRef(j) = contracted_point_WH_;
            } else if (smooth_sigmaPtr_->at(j) >= fix_eigen_ratio_ - 0.10) {
                continue;
            } else {
                WL_Ptr_->coeffRef(j) = WL_Ptr_->coeffRef(j) * sL_ > max_WL_ ? max_WL_ : WL_Ptr_->coeffRef(j) * sL_;
            }
        }

        // Contract one more time
        Eigen::MatrixXd cpts_temp = LaplacianContraction(cptsPtr);
        std::shared_ptr<Eigen::MatrixXd> cpts_tempPtr = std::make_shared<Eigen::MatrixXd>(cpts_temp);

        // Check the contraction rate
        GetMeanVertexDualArea(cpts_tempPtr);
        contraction_rate = faces_area_.back() / faces_area_[0];
        contraction_history.emplace_back(contraction_rate);
        MyLogger.Log(std::format("Current mean area is: {:.6f}; the contract ratio is: {:.6f}.", faces_area_.back(),
                                 contraction_rate), 0, true, false);
        if (contraction_history[i] - contraction_history.back() <= contraction_threshold_ ||
            std::isnan(contraction_history.back())) {
            MyLogger.Log(std::format(
                    "Touch the threshold! Iteration {} terminated! Total valid contraction iteration time: {}.", i + 2,
                    i + 1), 0, true, false);
            break;
        }

        // Update sigma
        ComputeSigma(cpts_tempPtr, "weighted");

        // Save the contracted points
        smooth_sigma.clear();
        smooth_sigma.resize(pts_num_);
        for (int j = 0; j < cpts_tempPtr->rows(); ++j) {
            smooth_sigma.at(j) = smooth_sigmaPtr_->at(j);
        }
        tool::io::SavePointCloudToPLY(cpts_tempPtr, output_path_ / ("_cpts_" + std::to_string(i + 2) + ".ply"),
                                      smooth_sigma, "smooth_sigma", binary_format_);

        // Update cptsPtr
        *cptsPtr = *cpts_tempPtr;
    }

    return cptsPtr;
}
