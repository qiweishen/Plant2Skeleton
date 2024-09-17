#include "Skeleton.h"
#include "Tools.h"

#include "geometrycentral/pointcloud/point_cloud_heat_solver.h"


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
        WH_Ptr_->coeffRef(tip_index) = 9.0;
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Weight matrices have been initialized! Elapsed time: " << elapsed.count() << "s" << std::endl;
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


    // Find the lowest vertex index
    // TODO: Set a FLAG
    if (threshold_edge_length_ < 0.0) {
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


        // --------------------------------------------------------------------------
        // Debug
        // --------------------------------------------------------------------------
//        // For visualization
//        std::shared_ptr<Eigen::MatrixXd> tip_points = std::make_shared<Eigen::MatrixXd>(tip_indices_.size(), 3);
//        for (int i = 0; i < tip_indices_.size(); ++i) {
//            tip_points->row(i) = cloudPtr->row(tip_indices_[i]);
//        }
//        tool::visualize::DrawPointClouds("Cloud", {{"Vertex", cloudPtr}});
//        polyscope::getPointCloud("Vertex")->addScalarQuantity("geodesic distance", distance);
//        tool::visualize::DrawPointClouds("Tip Points", {{"Tip", tip_points}});
//        std::shared_ptr<Eigen::MatrixXd> source_points = std::make_shared<Eigen::MatrixXd>(1, 3);
//        source_points->row(0) = cloudPtr->row(min_index);
//        tool::visualize::DrawPointClouds("Source Points", {{"Source", source_points}});
        // --------------------------------------------------------------------------
        // Debug
        // --------------------------------------------------------------------------


    }


    // --------------------------------------------------------------------------
    // Debug
    // --------------------------------------------------------------------------
//    tool::visualize::DrawTuftedMesh("Tufted Mesh", gc_geom);
    // --------------------------------------------------------------------------
    // Debug
    // --------------------------------------------------------------------------


    // No need to CollapseLargeFaces at first, since the k_neighbors_ is small
    if (threshold_edge_length_ < 0.0) {
        FindEdgeThreshold(gc_cloudPtr, gc_geom);
    } else {
        CollapseEdges(gc_cloudPtr, gc_geom);
    }


    // --------------------------------------------------------------------------
    // Debug
    // --------------------------------------------------------------------------
//    tool::visualize::DrawTuftedMesh("Tufted Mesh - After Face Ignore", gc_geom);
    // --------------------------------------------------------------------------
    // Debug
    // --------------------------------------------------------------------------


    auto start = std::chrono::high_resolution_clock::now();

    // Compute the Laplacian matrix
    gc_geom->requireLaplacian();
    *L_Ptr_ = -(2 * gc_geom->laplacian);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Laplacian matrix has been established! Elapsed time: " << elapsed.count() << "s" << std::endl;
}


void Skeleton::FindEdgeThreshold(std::shared_ptr<geometrycentral::pointcloud::PointCloud> &gc_cloudPtr,
                                 std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> &gc_geom) {
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
    std::cout << "Threshold edge length has been initialized! Elapsed time: " << elapsed.count() << "s" << std::endl;
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
    std::cout << "Large faces have been ignored! Final Stage Elapsed time: " << elapsed.count() << "s" << std::endl;
}


std::tuple<Eigen::SparseMatrix<double>, Eigen::MatrixXd>
Skeleton::LengthConstraint(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr) {
    // Initialize PointCloud and Geometry
    auto gc_cloudPtr = std::make_shared<geometrycentral::pointcloud::PointCloud>(pts_num_);
    auto gc_geom = std::make_shared<geometrycentral::pointcloud::PointPositionGeometry>(*gc_cloudPtr);

    // Convert and set vertex positions
    std::vector<geometrycentral::Vector3> vertices_positions = tool::utility::Matrix2GCVector(cloudPtr);
    for (int i = 0; i < pts_num_; ++i) {
        gc_geom->positions[i] = vertices_positions[i];
    }
    gc_geom->kNeighborSize = 4;
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

                if (distance > 0.005 * diagonal_length_) {
                    geometrycentral::Vector3 e_i_j = (0.005 * diagonal_length_ / distance) * (point_i - point_j);
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

    return std::make_tuple(H, e);
}


// TODO: Return std::shared_ptr<Eigen::MatrixXd>
Eigen::MatrixXd Skeleton::LaplacianContraction(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr) {
    auto start = std::chrono::high_resolution_clock::now();

    Eigen::SparseMatrix<double> H;
    Eigen::MatrixXd e;
    std::tie(H, e) = LengthConstraint(cloudPtr);

    // Construct the matrix A (3pts_num_ x pts_num_) { A = [L.*WL; sparse(1:P.npts, 1:P.npts, S); sparse(1:P.npts, 1:P.npts, WH)]; }
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
            double value = it.value() * 9.0;
            A_tripletList.emplace_back(2 * pts_num_ + it.row(), it.col(), value);
        }
    }
    Eigen::SparseMatrix<double> A(2 * pts_num_ + H.rows(), pts_num_);
    A.setFromTriplets(A_tripletList.begin(), A_tripletList.end());

    // Construct vector b (2pts_num_ x 3) { b = [zeros(P.npts, 3); zeros(P.npts, 3); sparse(1:P.npts, 1:P.npts, WH)*P.pts]; }
    Eigen::MatrixXd zeros = Eigen::MatrixXd::Zero(pts_num_, 3);
    Eigen::MatrixXd WHP = WH_Ptr_->asDiagonal() * (*cloudPtr);
    e = 9.0 * e;
    Eigen::MatrixXd b(2 * pts_num_ + e.rows(), 3);
    b << zeros, WHP, e;

    // Solve the linear system
    Eigen::SparseMatrix<double> ATA = A.transpose() * A;
    Eigen::MatrixXd ATb = A.transpose() * b;
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver;
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

// TODO: Return std::shared_ptr<Eigen::MatrixXd>
//Eigen::MatrixXd Skeleton::LaplacianContraction(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr) {
//    auto start = std::chrono::high_resolution_clock::now();
//
//    // Construct the matrix A (3pts_num_ x pts_num_) { A = [L.*WL; sparse(1:P.npts, 1:P.npts, S); sparse(1:P.npts, 1:P.npts, WH)]; }
//    std::vector<Eigen::Triplet<double>> A_tripletList;
//    for (int i = 0; i < pts_num_; ++i) {
//        for (Eigen::SparseMatrix<double>::InnerIterator it(*L_Ptr_, i); it; ++it) {
//            double value = it.value() * WL_Ptr_->coeffRef(it.row());
//            A_tripletList.emplace_back(it.row(), it.col(), value);
//        }
//        A_tripletList.emplace_back(pts_num_ + i, i, WH_Ptr_->coeffRef(i));
//    }
//    Eigen::SparseMatrix<double> A(2 * pts_num_, pts_num_);
//    A.setFromTriplets(A_tripletList.begin(), A_tripletList.end());
//
//    // Construct vector b (2pts_num_ x 3) { b = [zeros(P.npts, 3); zeros(P.npts, 3); sparse(1:P.npts, 1:P.npts, WH)*P.pts]; }
//    Eigen::MatrixXd zeros = Eigen::MatrixXd::Zero(pts_num_, 3);
//    Eigen::MatrixXd WHP = WH_Ptr_->asDiagonal() * (*cloudPtr);
//    Eigen::MatrixXd b(2 * pts_num_, 3);
//    b << zeros, WHP;
//
//    // Solve the linear system
//    Eigen::SparseMatrix<double> ATA = A.transpose() * A;
//    Eigen::MatrixXd ATb = A.transpose() * b;
//    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver;
//    solver.compute(ATA);
//    if (solver.info() != Eigen::Success) {
//        std::cerr << "Error: Eigen decomposition failed!" << std::endl;
//        std::exit(EXIT_FAILURE);
//    }
//    Eigen::MatrixXd cpts = solver.solve(ATb);
//
//    auto end = std::chrono::high_resolution_clock::now();
//    std::chrono::duration<double> elapsed = end - start;
//    std::cout << "Linear system has been solved! Elapsed time: " << elapsed.count() << "s" << std::endl;
//
//    return cpts;
//}


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
    // Fixed value, for computation efficiency
    gc_geom->kNeighborSize = 8;

    gc_geom->requireTuftedTriangulation();
    geometrycentral::surface::IntrinsicGeometryInterface &geometry = *(gc_geom->tuftedGeom);
    geometry.requireVertexDualAreas();
    geometrycentral::surface::VertexData<double> dual_areas = geometry.vertexDualAreas;
    double total_area = dual_areas.toVector().mean();
    faces_area_.emplace_back(total_area);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Mean vertex dual area has been updated! Elapsed time: " << elapsed.count() << "s" << std::endl;
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
#pragma omp parallel for default(none) shared(neighbors_indices, cloudPtr)
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
#pragma omp parallel for default(none) shared(neighbors_indices)
        for (int i = 0; i < pts_num_; ++i) {
            for (size_t &index: neighbors_indices[i]) {
                smooth_sigmaPtr_->at(i) += sigmaPtr_->at(index);
            }
            smooth_sigmaPtr_->at(i) += sigmaPtr_->at(i);
            smooth_sigmaPtr_->at(i) /= int(neighbors_indices[i].size()) + 1;
        }
    } else if (neighborhood == "weighted") {
        neighbors_indices = tool::utility::RadiusSearch(cloudPtr, sigma_radius_);
#pragma omp parallel for default(none) shared(neighbors_indices, cloudPtr, sigma_radius_)
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
#pragma omp parallel for default(none) shared(neighbors_indices)
        for (int i = 0; i < pts_num_; ++i) {
            smooth_sigmaPtr_->at(i) += sigmaPtr_->at(i);
            for (const size_t &index: neighbors_indices[i]) {
                smooth_sigmaPtr_->at(i) += sigmaPtr_->at(index);
            }
            smooth_sigmaPtr_->at(i) /= int(neighbors_indices[i].size()) + 1;
        }
    } else {
        std::cerr << "Invalid neighborhood searching method for Sigma!" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Smooth sigmas have been updated! Elapsed time: " << elapsed.count() << "s" << std::endl;
}


void Skeleton::UpdateKNeighbors() {
    const int max_neighbors = 32;
    k_neighbors_ = k_neighbors_ + 4 > max_neighbors ? max_neighbors : k_neighbors_ + 4;
}


// The main function for Laplacian skeletonization
std::shared_ptr<Eigen::MatrixXd> Skeleton::ContractionIteration() {
    // Save the original points
    std::shared_ptr<Eigen::MatrixXd> _ = std::make_shared<Eigen::MatrixXd>(pts_num_, 1);
    for (int i = 0; i < pts_num_; ++i) {
        _->coeffRef(i, 0) = 0.0;
    }
    std::vector<std::pair<std::string, std::string>> __ = {{"double", "smooth_sigma"}};
    tool::io::SavePointCloudToPLY(output_path_ + "_cpts_0.ply", cloudPtr_, _, __);

    // Start first time contraction
    std::cout << "--------------------------------------------------" << std::endl;
    std::cout << "Contraction iteration time: 1" << std::endl;

    // Initialize parameters
    InitializeParameters(cloudPtr_);

    // Contract for the first time
    Eigen::MatrixXd cpts = LaplacianContraction(cloudPtr_);
    std::shared_ptr<Eigen::MatrixXd> cptsPtr = std::make_shared<Eigen::MatrixXd>(cpts);


    // --------------------------------------------------------------------------
    // Debug
    // --------------------------------------------------------------------------
//    tool::visualize::DrawPointClouds("Contraction", {{"Original",    cloudPtr_},
//                                                     {"Iteration 1", cptsPtr}});
    // --------------------------------------------------------------------------
    // Debug
    // --------------------------------------------------------------------------


    // Check the contraction rate
    std::vector<double> contraction_history;
    GetMeanVertexDualArea(cloudPtr_);
    GetMeanVertexDualArea(cptsPtr);
    double contraction_rate = faces_area_.back() / faces_area_[0];
    contraction_history.emplace_back(contraction_rate);
    std::cout << "Current mean area is: " << faces_area_.back() << "; "
              << "the contract ratio is: " << contraction_rate << std::endl;

    // Compute sigma for the contracted points
    ComputeSigma(cptsPtr, "weighted");

    // Save the contracted points
    _ = std::make_shared<Eigen::MatrixXd>(pts_num_, 1);
    for (int i = 0; i < cptsPtr->rows(); ++i) {
        _->coeffRef(i, 0) = smooth_sigmaPtr_->at(i);
    }
    tool::io::SavePointCloudToPLY(output_path_ + "_cpts_1.ply", cptsPtr, _, __);

    // Start the contraction iterations
    for (int i = 0; i < max_iteration_time_ - 1; ++i) {
        std::cout << "--------------------------------------------------" << std::endl;
        std::cout << "Contraction iteration time: " << i + 2 << std::endl;

        // Update k_neighbors
        UpdateKNeighbors();

        // Update Laplacian matrix
        EstablishLaplacianMatrix(cptsPtr);

        // Fix the points with high sigma --- Current Version
        for (int j = 0; j < pts_num_; ++j) {
            if (WH_Ptr_->coeffRef(j) == 9.0) {
                continue; // Skip the tip points
            } else if (smooth_sigmaPtr_->at(j) >= fix_eigen_ratio_ || WH_Ptr_->coeffRef(j) == 3.0) {
                WH_Ptr_->coeffRef(j) = 3.0;
                WL_Ptr_->coeffRef(j) = 0.0;
            } else if (smooth_sigmaPtr_->at(j) >= fix_eigen_ratio_ - 0.10) {
                continue;
            } else {
                WL_Ptr_->coeffRef(j) = WL_Ptr_->coeffRef(j) * sL_ > 9.0 ? 9.0 : WL_Ptr_->coeffRef(j) * sL_;
            }
        }

        // Contract one more time
        Eigen::MatrixXd cpts_temp = LaplacianContraction(cptsPtr);
        std::shared_ptr<Eigen::MatrixXd> cpts_tempPtr = std::make_shared<Eigen::MatrixXd>(cpts_temp);


        // --------------------------------------------------------------------------
        // Debug
        // --------------------------------------------------------------------------
//        tool::visualize::DrawPointClouds("Contraction", {{"Iteration " + std::to_string(i + 2), cpts_tempPtr}});
        // --------------------------------------------------------------------------
        // Debug
        // --------------------------------------------------------------------------


        // Check the contraction rate
        GetMeanVertexDualArea(cpts_tempPtr);
        contraction_rate = faces_area_.back() / faces_area_[0];
        contraction_history.emplace_back(contraction_rate);
        std::cout << "Current mean area is: " << faces_area_.back() << "; "
                  << "the contract ratio is: " << contraction_rate << std::endl;
        if (contraction_history[i] - contraction_history.back() <= 1e-6 || std::isnan(contraction_history.back())) {
            std::cout << "Touch the threshold! Iteration " << i + 2
                      << " terminated! Total valid contraction iteration time: "
                      << i + 1 << std::endl;
            break;
        }

        // Update sigma
        ComputeSigma(cpts_tempPtr, "weighted");

        // Save the contracted points
        _ = std::make_shared<Eigen::MatrixXd>(pts_num_, 1);
        for (int j = 0; j < cpts_tempPtr->rows(); ++j) {
            _->coeffRef(j, 0) = smooth_sigmaPtr_->at(j);
        }
        tool::io::SavePointCloudToPLY(output_path_ + "_cpts_" + std::to_string(i + 2) + ".ply", cpts_tempPtr, _,
                                      __);

        // Update cptsPtr
        *cptsPtr = *cpts_tempPtr;
    }

    return cptsPtr;
}
