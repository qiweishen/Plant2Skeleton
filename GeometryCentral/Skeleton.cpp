#include "Skeleton.h"
#include "Tools.h"


void Skeleton::InitializeParameters(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr) {
    auto start = std::chrono::high_resolution_clock::now();

    WL_Ptr_->resize(pts_num_);
    WH_Ptr_->resize(pts_num_);

    // Initialize WL, WH
    WL_Ptr_->fill(initial_WL_);
    WH_Ptr_->fill(initial_WH_);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Weight matrices have been initialized! Elapsed time: " << elapsed.count() << "s" << std::endl;

    // Initialize Laplacian matrix
    EstablishLaplacianMatrix(cloudPtr);

    // Initialize Spring energy
    EstablishSpringEnergy(cloudPtr, 4, 3.0, 0.0015 * diagonal_length_);
}


void Skeleton::EstablishLaplacianMatrix(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr) {
    auto start = std::chrono::high_resolution_clock::now();

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


    // --------------------------------------------------------------------------
    // Debug
    // --------------------------------------------------------------------------
//    tool::visualize::DrawUnionLocalTriangles("Union Local Triangles", gc_cloudPtr, gc_geom);
    tool::visualize::DrawTuftedMesh("Tufted Mesh", gc_geom);
    // --------------------------------------------------------------------------
    // Debug
    // --------------------------------------------------------------------------


    // No need to IgnoreLargeFaces at first, since the k_neighbors_ is small
    if (threshold_edge_length_ < 0.0) {
        FindThresholdEdge(gc_cloudPtr, gc_geom);
    } else {
        IgnoreLargeFaces(gc_cloudPtr, gc_geom);
        tool::visualize::DrawPointClouds("Cloud", {{"Skeleton", cloudPtr}});
        tool::visualize::DrawTuftedMesh("Tufted Mesh - After Face Ignore", gc_geom);
    }


    // --------------------------------------------------------------------------
    // Debug
    // --------------------------------------------------------------------------
//    tool::visualize::DrawTuftedMesh("Tufted Mesh - After Face Ignore", gc_geom);
    // --------------------------------------------------------------------------
    // Debug
    // --------------------------------------------------------------------------


    // Compute the Laplacian matrix
    gc_geom->requireLaplacian();
    *L_Ptr_ = -(2 * gc_geom->laplacian);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Laplacian matrix has been established! Elapsed time: " << elapsed.count() << "s" << std::endl;
}


void Skeleton::FindThresholdEdge(std::shared_ptr<geometrycentral::pointcloud::PointCloud> &gc_cloudPtr,
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
    Eigen::VectorXd edge_lengths = temp_geom->edgeLengths.toVector();
    // Compute the 95th percentile of the edge lengths
    threshold_edge_length_ = tool::utility::Compute95thPercentile(edge_lengths);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Threshold edge length has been initialized! Elapsed time: " << elapsed.count() << "s" << std::endl;
}


void Skeleton::IgnoreLargeFaces(std::shared_ptr<geometrycentral::pointcloud::PointCloud> &gc_cloudPtr,
                                std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> &gc_geom) {
    auto start = std::chrono::high_resolution_clock::now();

    std::unique_ptr<geometrycentral::surface::SurfaceMesh> tuftedMesh;
    std::unique_ptr<geometrycentral::surface::EdgeLengthGeometry> tuftedGeom;
    // Generate the local triangles
    geometrycentral::pointcloud::PointData<std::vector<std::array<geometrycentral::pointcloud::Point, 3>>> local_tri_point = geometrycentral::pointcloud::buildLocalTriangulations(
            *gc_cloudPtr, *gc_geom, true);
    // Make a union of local triangles
    std::vector<std::vector<size_t>> all_tris = handleToFlatInds(*gc_cloudPtr, local_tri_point);
    std::vector<geometrycentral::Vector3> pos_raw(gc_cloudPtr->nPoints());
    for (size_t iP = 0; iP < pos_raw.size(); iP++) {
        pos_raw[iP] = gc_geom->positions[iP];
    }
    // Make a mesh for edge collapse detection
    std::unique_ptr<geometrycentral::surface::SurfaceMesh> temp_mesh;
    std::unique_ptr<geometrycentral::surface::VertexPositionGeometry> temp_geom;
    std::tie(temp_mesh, temp_geom) = geometrycentral::surface::makeSurfaceMeshAndGeometry(all_tris, pos_raw);

    temp_geom->requireEdgeLengths();
    geometrycentral::surface::EdgeData<double> edge_lengths = temp_geom->edgeLengths;
    // Find the Face to ignore
    std::vector<std::vector<size_t>> ignore_faces;
    for (geometrycentral::surface::Edge e : temp_mesh->edges()) {
        if (edge_lengths[e] > threshold_edge_length_) {
            for (geometrycentral::surface::Face f : e.adjacentFaces()) {
                std::vector<size_t> face;
                for (geometrycentral::surface::Vertex v : f.adjacentVertices()) {
                    face.emplace_back(v.getIndex());
                }
                ignore_faces.emplace_back(face);
            }
        }
    }
    // Remove the ignore faces (contains duplicate faces)
    for (const std::vector<size_t> &face: ignore_faces) {
        all_tris.erase(std::remove(all_tris.begin(), all_tris.end(), face), all_tris.end());
    }

    // Check for the unreferenced vertices
    size_t max_index = pos_raw.size();
    std::vector<bool> referenced(max_index, false);
    for (const std::vector<size_t>& tri : all_tris) {
        for (size_t vertex : tri) {
            if (vertex < max_index) {
                referenced[vertex] = true;
            }
        }
    }
    std::vector<size_t> unreferenced_vertices;
    for (size_t i = 0; i < max_index; ++i) {
        if (!referenced[i]) {
            unreferenced_vertices.push_back(i);
        }
    }
    // How to deal with the unreferenced vertices? Remove? Or build a triangle with two nearest vertices?
    if (!unreferenced_vertices.empty()) {
        geometrycentral::NearestNeighborFinder finder(pos_raw);
        for (size_t vertex : unreferenced_vertices) {
            std::vector<size_t> nearest_indices = finder.kNearestNeighbors(vertex, 2);
            std::vector<size_t> nearest_triangle = {vertex, nearest_indices[0], nearest_indices[1]};
            all_tris.emplace_back(nearest_triangle);
        }
    }

    // --------------------------------------------------------------------------
    // Debug
    // --------------------------------------------------------------------------
//    polyscope::init();
//    polyscope::view::setUpDir(polyscope::UpDir::ZUp);
//    polyscope::view::setFrontDir(polyscope::FrontDir::XFront);
//    polyscope::registerPointCloud("Original", tool::utility::Matrix2GCVector(cloudPtr_));
//    polyscope::registerPointCloud("Debug-Pos", pos_raw);
//    polyscope::registerSurfaceMesh("Debug-Tris", pos_raw, all_tris);
//    polyscope::show();
    // --------------------------------------------------------------------------
    // Debug
    // --------------------------------------------------------------------------


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
    std::cout << "Large faces have been ignored! Elapsed time: " << elapsed.count() << "s" << std::endl;
}


void Skeleton::EstablishSpringEnergy(const std::shared_ptr<Eigen::MatrixXd> &ptsPtr, int neighbors,
                                     const double spring_constant, const double spring_length) {
    auto start = std::chrono::high_resolution_clock::now();

    S_Ptr_->setZero();

    std::vector<std::vector<size_t>> neighbors_indices;
    neighbors_indices = tool::utility::KNNSearch(ptsPtr, size_t(neighbors));

    for (int i = 0; i < pts_num_; ++i) {
        Eigen::Vector3d query_point = ptsPtr->row(i);
        std::vector<size_t> indices = neighbors_indices[i];

        Eigen::Vector3d force = Eigen::Vector3d::Zero();
        for (const size_t &index: indices) {
            Eigen::Vector3d a = ptsPtr->row(int(index));
            Eigen::Vector3d query_a = a - query_point;
            double distance = query_a.norm();
            if (distance > spring_length) {
                Eigen::Vector3d direction = query_a.normalized();
                force += spring_constant * std::abs(distance - spring_length) * direction;
            }
        }
        S_Ptr_->row(i) = force;
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Spring system has been established! Elapsed time: " << elapsed.count() << "s" << std::endl;
}


Eigen::MatrixXd Skeleton::LaplacianContraction(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr) {
    auto start = std::chrono::high_resolution_clock::now();

    // Construct the matrix A (3pts_num_ x pts_num_) { A = [L.*WL; sparse(1:P.npts, 1:P.npts, S); sparse(1:P.npts, 1:P.npts, WH)]; }
//    Eigen::VectorXd threes = Eigen::VectorXd(pts_num_);
//    threes.fill(3.0);
    std::vector<Eigen::Triplet<double>> A_tripletList;
    for (int i = 0; i < pts_num_; ++i) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(*L_Ptr_, i); it; ++it) {
            double value = it.value() * WL_Ptr_->coeffRef(it.row());
            A_tripletList.emplace_back(it.row(), it.col(), value);
        }
//        A_tripletList.emplace_back(pts_num_ + i, i, threes.coeffRef(i));
        A_tripletList.emplace_back(pts_num_ + i, i, WH_Ptr_->coeffRef(i));
    }
    Eigen::SparseMatrix<double> A(2 * pts_num_, pts_num_);
    A.setFromTriplets(A_tripletList.begin(), A_tripletList.end());

    // Construct vector b (2pts_num_ x 3) { b = [zeros(P.npts, 3); zeros(P.npts, 3); sparse(1:P.npts, 1:P.npts, WH)*P.pts]; }
    Eigen::MatrixXd zeros = Eigen::MatrixXd::Zero(pts_num_, 3);
//    Eigen::MatrixXd SP = threes.asDiagonal() * ((*cloudPtr) + (*S_Ptr_));
    Eigen::MatrixXd WHP = WH_Ptr_->asDiagonal() * (*cloudPtr);
    Eigen::MatrixXd b(2 * pts_num_, 3);
//    b << zeros, SP, WHP;
    b << zeros, WHP;

    // Solve the linear system
    Eigen::SparseMatrix<double> ATA = A.transpose() * A;
    Eigen::MatrixXd ATb = A.transpose() * b;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
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

    // TODO: Adjust the radius automatically based on assertion
    const double radius = 0.015 * diagonal_length_;
    const size_t k = 8;

    std::vector<std::vector<size_t>> neighbors_indices;
    if (neighborhood == "knn" || neighborhood == "radius") {
        if (neighborhood == "knn") {
            neighbors_indices = tool::utility::KNNSearch(cloudPtr, k);
        } else if (neighborhood == "radius") {
            neighbors_indices = tool::utility::RadiusSearch(cloudPtr, radius);
        }
#pragma omp parallel for default(none) shared(neighbors_indices, cloudPtr)
        for (int i = 0; i < pts_num_; ++i) {
            std::vector<size_t> indices;
            // The first element is the query point itself
            indices.emplace_back(i);
            indices.insert(indices.end(), neighbors_indices[i].begin(), neighbors_indices[i].end());

            assert(indices.size() >= 2 && indices[0] == i);

            Eigen::MatrixXd neighbors_cloudPtr = tool::utility::SelectByIndices(cloudPtr, neighbors_indices[i]);
            Eigen::Matrix3d covariance = tool::utility::ComputeCovariance(neighbors_cloudPtr);
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
        neighbors_indices = tool::utility::RadiusSearch(cloudPtr, radius);
#pragma omp parallel for default(none) shared(neighbors_indices, cloudPtr, radius)
        for (int i = 0; i < pts_num_; ++i) {
            Eigen::Vector3d query_point = cloudPtr->row(i);
            std::vector<size_t> indices = neighbors_indices[i];

            assert(indices.size() >= 2 && indices[0] == i);
            indices.erase(indices.begin()); // Remove the query point itself

            neighbors_indices[i] = indices;
            Eigen::MatrixXd Ci = Eigen::MatrixXd::Zero(3, 3);
            // Compute Ci for Pi using its neighborhood.
            for (const size_t &index: indices) {
                Eigen::Vector3d pj = cloudPtr->row(int(index)); // Get the neighboring point Pj.
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

    // Check the contraction rate
    std::vector<double> contraction_history;
    GetMeanVertexDualArea(cloudPtr_);
    GetMeanVertexDualArea(cptsPtr);
    double contraction_rate = faces_area_.back() / faces_area_[0];
    contraction_history.emplace_back(contraction_rate);
    std::cout << "Current mean area is: " << faces_area_.back() << "; "
              << "the contract ratio is: " << contraction_rate << std::endl;


    // --------------------------------------------------------------------------
    // Debug
    // --------------------------------------------------------------------------
//    tool::visualize::DrawPointClouds("Contraction", {{"Original", cloudPtr_}, {"Iteration 1", cptsPtr}});
    // --------------------------------------------------------------------------
    // Debug
    // --------------------------------------------------------------------------

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

        // Update Spring energy
        EstablishSpringEnergy(cptsPtr, 4, 3.0, 0.0015 * diagonal_length_);

        // Fix the points with high sigma --- Current Version
        for (int j = 0; j < pts_num_; ++j) {
            if (smooth_sigmaPtr_->at(j) >= fix_eigen_ratio_ || WH_Ptr_->coeffRef(j) == 3.0) {
                WH_Ptr_->coeffRef(j) = 3.0;
                WL_Ptr_->coeffRef(j) = 0.0;
//            } else {
//                double temp_WL = WL_Ptr_->coeffRef(j) * sL_;
//                if (temp_WL > 2048) {
//                    WL_Ptr_->coeffRef(j) = 2048;
//                } else {
//                    WL_Ptr_->coeffRef(j) = temp_WL;
//                }
            }
        }

        // Contract one more time
        Eigen::MatrixXd cpts_temp = LaplacianContraction(cptsPtr);
        std::shared_ptr<Eigen::MatrixXd> cpts_tempPtr = std::make_shared<Eigen::MatrixXd>(cpts_temp);

        // Check the contraction rate
        GetMeanVertexDualArea(cpts_tempPtr);
        contraction_rate = faces_area_.back() / faces_area_[0];
        contraction_history.emplace_back(contraction_rate);
        std::cout << "Current mean area is: " << faces_area_.back() << "; "
                  << "the contract ratio is: " << contraction_rate << std::endl;
        if (contraction_history[i] - contraction_history.back() <= 0.0 || std::isnan(contraction_history.back())) {
            std::cout << "Touch the threshold! Iteration " << i + 2
                      << " terminated! Total valid contraction iteration time: "
                      << i + 1 << std::endl;
            break;
        }


        // --------------------------------------------------------------------------
        // Debug
        // --------------------------------------------------------------------------
//        tool::visualize::DrawPointClouds("Contraction", {{"Iteration " + std::to_string(i + 2), cpts_tempPtr}});
        // --------------------------------------------------------------------------
        // Debug
        // --------------------------------------------------------------------------


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
        sL_ = sL_ * 1.20 < 3.0 ? sL_ * 1.20 : 3.0; // TODO: sL_ is necessary?
    }

    return cptsPtr;
}
