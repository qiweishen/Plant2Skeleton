#include "skeleton.h"

#include "tools.h"



void Skeleton::InitializeParameters(const Eigen::MatrixXd &cloud) {
	// Initialize Laplacian matrix
	EstablishLaplacianMatrix(cloud);

	Timer timer;

	// Initialize WL, WH
	WL_.fill(initial_WL_);
	WH_.fill(initial_WH_);
	for (int tip_index: tip_indices_) {
		WH_.coeffRef(tip_index) = tip_point_WH_;
	}

	double elapsed = timer.elapsed<Timer::TimeUnit::Seconds>();
	Logger::Instance().Log(std::format("Weight matrices have been initialized! Elapsed time: {:.6f}s", elapsed), LogLevel::INFO, IndentLevel::ONE,
						   true, false);
}


void Skeleton::EstablishLaplacianMatrix(const Eigen::MatrixXd &cloud) {
	L_.setZero();

	geometrycentral::pointcloud::PointCloud gc_cloud(pts_num_);
	geometrycentral::pointcloud::PointPositionGeometry gc_geom(gc_cloud);
	std::vector<geometrycentral::Vector3> vertices_positions = tool::utility::Matrix2Vector<geometrycentral::Vector3>(cloud);
	for (int i = 0; i < pts_num_; ++i) {
		gc_geom.positions[i] = vertices_positions[i];
	}
	if (threshold_edge_length_ < 0.0) {
		FindTipPoints(cloud, gc_cloud, gc_geom);
	}

	UpdateNeighborhood(gc_cloud, gc_geom);

	// threshold_edge_length_ is utilized as a flag to determine whether the tip points are found, with default kNeighborSize = 30
	// and there is no need to CollapseEdges at first iteration, since the k_neighbors_ is small
	if (threshold_edge_length_ < 0.0) {
		FindEdgeThreshold(gc_cloud, gc_geom);
	} else {
		CollapseEdges(gc_cloud, gc_geom);
	}

	Timer timer;

	// Compute the Laplacian matrix
	gc_geom.requireLaplacian();
	L_ = -(2 * gc_geom.laplacian);

	double elapsed = timer.elapsed<Timer::TimeUnit::Seconds>();
	Logger::Instance().Log(std::format("Laplacian matrix has been established! Elapsed time: {:.6f}s", elapsed), LogLevel::INFO, IndentLevel::ONE,
						   true, false);
}


void Skeleton::FindTipPoints(const Eigen::MatrixXd &cloud, geometrycentral::pointcloud::PointCloud &gc_cloud,
							 geometrycentral::pointcloud::PointPositionGeometry &gc_geom) {
	Timer timer;

	// Find the lowest vertex index
	Eigen::VectorXd z_values = cloud.col(2);
	int min_index;
	z_values.minCoeff(&min_index);
	// Compute the geodesic distance from the lowest vertex
	// In the PointCloudHeatSolver function, below quantities are required:
	//		geom.requireNeighbors();
	//		geom.requireTuftedTriangulation();
	//		geom.tuftedGeom->requireEdgeLengths();
	//		geom.requireTangentCoordinates();
	geometrycentral::pointcloud::PointCloudHeatSolver solver(gc_cloud, gc_geom);
	geometrycentral::pointcloud::Point pSource = gc_cloud.point(min_index);
	geometrycentral::pointcloud::PointData<double> distance = solver.computeDistance(pSource);
	// Identify the tip points by using KNN and the geodesic distance
	std::vector<std::vector<size_t>> neighbors_indices = tool::utility::RadiusSearch(cloud, radius_neighbor_);
	tip_indices_.clear();
	tip_indices_.emplace_back(min_index);
	for (int i = 0; i < pts_num_; ++i) {
		double center_value = distance[i];
		std::vector<double> neighbor_values;
		neighbor_values.reserve(neighbors_indices[i].size());
		for (const size_t &index: neighbors_indices[i]) {
			if (index == i) {
				continue;  // Skip the center point
			}
			neighbor_values.emplace_back(distance[index]);
		}
		if (double max_neighbor_value = *std::ranges::max_element(neighbor_values); center_value > max_neighbor_value) {
			tip_indices_.emplace_back(i);
		}
	}

	double elapsed = timer.elapsed<Timer::TimeUnit::Seconds>();
	Logger::Instance().Log(std::format("Tip points have been found by using Geodesic distance! Elapsed time: {:.6f}s", elapsed), LogLevel::INFO,
						   IndentLevel::ONE, true, false);
}


void Skeleton::UpdateNeighborhood(geometrycentral::pointcloud::PointCloud &gc_cloud, geometrycentral::pointcloud::PointPositionGeometry &gc_geom) {
	if (threshold_edge_length_ < 0) {  // First iteration
		geometrycentral::addition::UnrequireAllQuantities(gc_geom);
		gc_geom.purgeQuantities();
		if (use_knn_search_) {
			gc_geom.kNeighborSize = k_neighbors_;
			Logger::Instance().Log(std::format("k-nearest neighbors value has been updated! Current: k = {}", k_neighbors_), LogLevel::INFO,
								   IndentLevel::ONE, true, false);
		} else {
			gc_geom.neighborsQ.computed = true;
			gc_geom.neighborsQ.requireCount = 1;
			gc_geom.neighbors = std::make_unique<geometrycentral::pointcloud::Neighborhoods>(gc_cloud, gc_geom.positions, radius_neighbor_);
			Logger::Instance().Log(std::format("Radius neighbors value has been updated! Current: {:.4f} unit", radius_neighbor_), LogLevel::INFO,
								   IndentLevel::ONE, true, false);
		}
	} else {
		if (use_knn_search_) {
			k_neighbors_ = k_neighbors_ + delta_k_ > max_k_ ? max_k_ : k_neighbors_ + delta_k_;
			gc_geom.kNeighborSize = k_neighbors_;
			Logger::Instance().Log(std::format("k-nearest neighbors value has been updated! Current: k = {}", k_neighbors_), LogLevel::INFO,
								   IndentLevel::ONE, true, false);
		} else {
			radius_neighbor_ = radius_neighbor_ + delta_radius_ > max_radius_ ? max_radius_ : radius_neighbor_ + delta_radius_;
			gc_geom.neighborsQ.computed = true;
			gc_geom.neighborsQ.requireCount = 1;
			gc_geom.neighbors = std::make_unique<geometrycentral::pointcloud::Neighborhoods>(gc_cloud, gc_geom.positions, radius_neighbor_);
			Logger::Instance().Log(std::format("Radius neighbors value has been updated! Current: {:.4f} unit", radius_neighbor_), LogLevel::INFO,
								   IndentLevel::ONE, true, false);
		}
	}
}


void Skeleton::FindEdgeThreshold(geometrycentral::pointcloud::PointCloud &gc_cloud, geometrycentral::pointcloud::PointPositionGeometry &gc_geom) {
	Timer timer;

	// Generate the local triangles
	// In the buildLocalTriangulations function, the following quantities are required:
	//		geom.requireNeighbors();
	//		geom.requireTangentCoordinates();
	geometrycentral::pointcloud::PointData<std::vector<std::array<geometrycentral::pointcloud::Point, 3>>> local_tri_point =
			buildLocalTriangulations(gc_cloud, gc_geom, true);
	// Make a union of local triangles
	std::vector<std::vector<size_t>> all_tris = handleToFlatInds(gc_cloud, local_tri_point);
	std::vector<geometrycentral::Vector3> pos_raw(gc_cloud.nPoints());
	for (size_t iP = 0; iP < pos_raw.size(); iP++) {
		pos_raw[iP] = gc_geom.positions[iP];
	}
	auto [temp_meshPtr, temp_geomPtr] = geometrycentral::surface::makeSurfaceMeshAndGeometry(all_tris, pos_raw);
	temp_geomPtr->requireEdgeLengths();
	Eigen::VectorXd edges_lengths = temp_geomPtr->edgeLengths.toVector();
	std::vector edges_lengths_vec(edges_lengths.data(), edges_lengths.data() + edges_lengths.size());
	// Find the long edges
	std::vector<int> indices = tool::utility::FindUpperOutlierBySTD(edges_lengths_vec);
	Eigen::VectorXd long_edges_lengths = Eigen::VectorXd::Zero(static_cast<int>(indices.size()));
	for (int i = 0; i < indices.size(); ++i) {
		long_edges_lengths[i] = edges_lengths[indices[i]];
	}
	threshold_edge_length_ = long_edges_lengths.minCoeff();

	double elapsed = timer.elapsed<Timer::TimeUnit::Seconds>();
	Logger::Instance().Log(std::format("Threshold edge length has been initialized! Elapsed time: {:.6f}s", elapsed), LogLevel::INFO,
						   IndentLevel::ONE, true, false);
}


void Skeleton::CollapseEdges(geometrycentral::pointcloud::PointCloud &gc_cloud, geometrycentral::pointcloud::PointPositionGeometry &gc_geom) {
	Timer timer;

	// Initialize the tuftedMesh and tuftedGeom
	std::unique_ptr<geometrycentral::surface::SurfaceMesh> tuftedMeshPtr;
	std::unique_ptr<geometrycentral::surface::EdgeLengthGeometry> tuftedGeomPtr;

	// Generate the local triangles
	geometrycentral::pointcloud::PointData<std::vector<std::array<geometrycentral::pointcloud::Point, 3>>> local_tri_point =
			buildLocalTriangulations(gc_cloud, gc_geom, true);

	// Make a union of local triangles
	std::vector<std::vector<size_t>> all_tris = handleToFlatInds(gc_cloud, local_tri_point);
	std::vector<geometrycentral::Vector3> pos_raw(gc_cloud.nPoints());
	for (size_t iP = 0; iP < pos_raw.size(); iP++) {
		pos_raw[iP] = gc_geom.positions[iP];
	}

	// Make a mesh for edge collapse detection
	std::unique_ptr<geometrycentral::surface::SurfaceMesh> temp_meshPtr;
	std::unique_ptr<geometrycentral::surface::VertexPositionGeometry> temp_geomPtr;
	std::tie(temp_meshPtr, temp_geomPtr) = geometrycentral::surface::makeSurfaceMeshAndGeometry(all_tris, pos_raw);

	// Find the Edge to be collapsed, and its adjacent faces
	temp_geomPtr->requireEdgeLengths();
	geometrycentral::surface::EdgeData<double> edge_lengths = temp_geomPtr->edgeLengths;
	std::vector<std::vector<size_t>> faces_to_ignore;
#pragma omp parallel for default(none) shared(temp_meshPtr, edge_lengths, faces_to_ignore)
	for (size_t i = 0; i < temp_meshPtr->nVertices(); i++) {
		geometrycentral::surface::Vertex vert = temp_meshPtr->vertex(i);
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
		} else {
			for (int edge_index: edge_indices) {
				// Mark the faces that consist of the long edge
				for (geometrycentral::surface::Edge edge = adjacent_edges[edge_index]; geometrycentral::surface::Face f: edge.adjacentFaces()) {
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
	std::erase_if(all_tris, [&ignore_set](const std::vector<size_t> &face) { return ignore_set.contains(face); });

	// TODO: Check the logic here
	// Check for unreferenced vertices
	size_t max_index = pos_raw.size();
	std::vector referenced(max_index, false);
#pragma omp parallel default(none) shared(all_tris, max_index, referenced)
	{
		std::vector local_referenced(max_index, false);
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
			std::vector nearest_triangle = { vertex, nearest_indices[0], nearest_indices[1] };
			all_tris.emplace_back(nearest_triangle);
		}
	}

	// Make a mesh, read off its
	std::unique_ptr<geometrycentral::surface::VertexPositionGeometry> posGeomPtr;
	std::tie(tuftedMeshPtr, posGeomPtr) = geometrycentral::surface::makeSurfaceMeshAndGeometry(all_tris, pos_raw);
	posGeomPtr->requireEdgeLengths();
	geometrycentral::surface::EdgeData<double> tuftedEdgeLengths = posGeomPtr->edgeLengths;
	// Mollify
	mollifyIntrinsic(*tuftedMeshPtr, tuftedEdgeLengths, 1e-5);
	// Build the cover
	buildIntrinsicTuftedCover(*tuftedMeshPtr, tuftedEdgeLengths);
	flipToDelaunay(*tuftedMeshPtr, tuftedEdgeLengths);
	// Create the geometry object
	tuftedGeomPtr = std::make_unique<geometrycentral::surface::EdgeLengthGeometry>(*tuftedMeshPtr, tuftedEdgeLengths);
	// Set tuftedTriangulationQ.require() to true, tricky
	gc_geom.tuftedTriangulationQ.computed = true;
	// Actually, TuftedTriangulation did not compute
	gc_geom.requireTuftedTriangulation();
	// Replace the tuftedMesh and tuftedGeom
	gc_geom.tuftedMesh = std::move(tuftedMeshPtr);
	gc_geom.tuftedGeom = std::move(tuftedGeomPtr);

	double elapsed = timer.elapsed<Timer::TimeUnit::Seconds>();
	Logger::Instance().Log(std::format("Long edges have been ignored! Elapsed time: {:.6f}s", elapsed), LogLevel::INFO, IndentLevel::ONE, true,
						   false);
}


std::tuple<Eigen::SparseMatrix<double>, Eigen::MatrixXd> Skeleton::DistanceConstraint(const Eigen::MatrixXd &cloud) {
	Timer timer;

	// Initialize PointCloud and Geometry
	auto gc_cloudPtr = std::make_unique<geometrycentral::pointcloud::PointCloud>(pts_num_);
	auto gc_geomPtr = std::make_unique<geometrycentral::pointcloud::PointPositionGeometry>(*gc_cloudPtr);

	// Convert and set vertex positions
	std::vector<geometrycentral::Vector3> vertices_positions = tool::utility::Matrix2Vector<geometrycentral::Vector3>(cloud);
	for (int i = 0; i < pts_num_; ++i) {
		gc_geomPtr->positions[i] = vertices_positions[i];
	}
	gc_geomPtr->kNeighborSize = k_neighbors_distance_constraint_;
	gc_geomPtr->requireLaplacian();
	gc_geomPtr->tuftedGeom->requireEdgeLengths();

	Eigen::VectorXd edge_lengths = gc_geomPtr->tuftedGeom->edgeLengths.toVector();

	// Find edges and their constraints
	std::vector<std::pair<int, int>> edges;
	std::vector<geometrycentral::Vector3> constraint_lengths;
	Eigen::SparseMatrix<double> laplacian = gc_geomPtr->laplacian;
	for (int k = 0; k < laplacian.outerSize(); ++k) {
		for (Eigen::SparseMatrix<double>::InnerIterator it(laplacian, k); it; ++it) {
			if (it.row() < it.col()) {	// Process each edge once
				geometrycentral::Vector3 point_i = gc_geomPtr->positions[it.row()];
				geometrycentral::Vector3 point_j = gc_geomPtr->positions[it.col()];

				if (double distance = (point_i - point_j).norm(); distance > max_distance_) {
					geometrycentral::Vector3 e_i_j = max_distance_ / distance * (point_i - point_j);
					constraint_lengths.emplace_back(e_i_j);
				} else {
					geometrycentral::Vector3 e_i_j = 1.0 * (point_i - point_j);
					constraint_lengths.emplace_back(e_i_j);
				}
				edges.emplace_back(it.row(), it.col());
			}
		}
	}

	// Convert constraint lengths to Eigen::MatrixXd
	Eigen::MatrixXd e(edges.size(), 3);
	for (size_t i = 0; i < edges.size(); ++i) {
		e.row(static_cast<int>(i)) = Eigen::Vector3d{ constraint_lengths[i].x, constraint_lengths[i].y, constraint_lengths[i].z };
	}

	// Create edge matrix H
	Eigen::SparseMatrix<double> H(static_cast<int>(edges.size()), pts_num_);
	std::vector<Eigen::Triplet<double>> tripletList;
	for (size_t i = 0; i < edges.size(); ++i) {
		tripletList.emplace_back(i, edges[i].first, 1.0);
		tripletList.emplace_back(i, edges[i].second, -1.0);
	}
	H.setFromTriplets(tripletList.begin(), tripletList.end());

	double elapsed = timer.elapsed<Timer::TimeUnit::Seconds>();
	Logger::Instance().Log(std::format("Distance constraints have been built! Elapsed time: {:.6f}s", elapsed), LogLevel::INFO, IndentLevel::ONE,
						   true, false);

	return std::make_tuple(H, e);
}


Eigen::MatrixXd Skeleton::LaplacianContraction(const Eigen::MatrixXd &cloud) {
	Timer timer;

	auto [H, e] = DistanceConstraint(cloud);

	// Construct the matrix A (2pts_num_+H x pts_num_) { A = [L.*WL; sparse(1:P.npts, 1:P.npts, WH); sparse(1:P.npts, 1:P.npts, H)]}
	std::vector<Eigen::Triplet<double>> A_tripletList;
	for (int i = 0; i < pts_num_; ++i) {
		for (Eigen::SparseMatrix<double>::InnerIterator it(L_, i); it; ++it) {
			double value = it.value() * WL_.coeffRef(it.row());
			A_tripletList.emplace_back(it.row(), it.col(), value);
		}
		A_tripletList.emplace_back(pts_num_ + i, i, WH_.coeffRef(i));
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
	Eigen::MatrixXd WHP = WH_.asDiagonal() * cloud;
	e *= WD_;
	Eigen::MatrixXd b(2 * pts_num_ + e.rows(), 3);
	b << zeros, WHP, e;

	// Solve the linear system
	Eigen::SparseMatrix<double> ATA = A.transpose() * A;
	Eigen::MatrixXd ATb = A.transpose() * b;
	Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver;
	solver.compute(ATA);
	if (solver.info() != Eigen::Success) {
		Logger::Instance().Log("Eigen decomposition failed!", LogLevel::ERROR);
	}
	Eigen::MatrixXd cpts = solver.solve(ATb);

	double elapsed = timer.elapsed<Timer::TimeUnit::Seconds>();
	Logger::Instance().Log(std::format("Linear system has been solved! Elapsed time: {:.6f}s", elapsed), LogLevel::INFO, IndentLevel::ONE, true,
						   false);
	return cpts;
}


void Skeleton::GetMeanVertexDualArea(const Eigen::MatrixXd &cloud) {
	Timer timer;

	auto gc_cloudPtr = std::make_unique<geometrycentral::pointcloud::PointCloud>(pts_num_);
	auto gc_geomPtr = std::make_unique<geometrycentral::pointcloud::PointPositionGeometry>(*gc_cloudPtr);
	std::vector<geometrycentral::Vector3> vertices_positions = tool::utility::Matrix2Vector<geometrycentral::Vector3>(cloud);
	for (int i = 0; i < pts_num_; ++i) {
		gc_geomPtr->positions[i] = vertices_positions[i];
	}

	gc_geomPtr->requireTuftedTriangulation();
	geometrycentral::surface::IntrinsicGeometryInterface &geometry = *gc_geomPtr->tuftedGeom;
	geometry.requireVertexDualAreas();
	geometrycentral::surface::VertexData<double> dual_areas = geometry.vertexDualAreas;
	double total_area = dual_areas.toVector().mean();
	faces_area_.emplace_back(total_area);

	double elapsed = timer.elapsed<Timer::TimeUnit::Seconds>();
	Logger::Instance().Log(std::format("Mean vertex dual area has been updated! Elapsed time: {:.6f}s", elapsed), LogLevel::INFO, IndentLevel::ONE,
						   true, false);
}


void Skeleton::ComputeSigma(const Eigen::MatrixXd &cloud) {
	Timer timer;

	sigma_.clear();
	sigma_.resize(pts_num_);
	smooth_sigma_.clear();
	smooth_sigma_.resize(pts_num_);

	std::vector<std::vector<size_t>> neighbors_indices;
	neighbors_indices = tool::utility::RadiusSearch(cloud, sigma_radius_);
#pragma omp parallel for default(none) shared(neighbors_indices, cloud, Eigen::Dynamic)
	for (int i = 0; i < pts_num_; ++i) {
		Eigen::Vector3d query_point = cloud.row(i);
		std::vector<size_t> indices = neighbors_indices[i];

		// If point does not meet these criteria, means it is quite isolated among its neighbors, just set to fix_eigen_ratio_
		if (!(indices.size() >= 2)) {
			sigma_.at(i) = fix_eigen_ratio_;
			continue;
		}

		// Remove the query point itself
		indices.erase(std::remove(indices.begin(), indices.end(), i), indices.end());

		neighbors_indices[i] = indices;
		Eigen::MatrixXd Ci = Eigen::MatrixXd::Zero(3, 3);
		// Compute Ci for Pi using its neighborhood.
		for (const size_t &index: indices) {
			Eigen::Vector3d pj = cloud.row(static_cast<int>(index));  // Get the neighboring point Pj.
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
		sigma_.at(i) = lambda_2 / (lambda_0 + lambda_1 + lambda_2);
	}
#pragma omp parallel for default(none) shared(neighbors_indices)
	for (int i = 0; i < pts_num_; ++i) {
		smooth_sigma_.at(i) += sigma_.at(i);
		for (const size_t &index: neighbors_indices[i]) {
			smooth_sigma_.at(i) += sigma_.at(index);
		}
		smooth_sigma_.at(i) /= static_cast<int>(neighbors_indices[i].size()) + 1;
	}

	double elapsed = timer.elapsed<Timer::TimeUnit::Seconds>();
	Logger::Instance().Log(std::format("Smooth sigmas have been updated! Elapsed time: {:.6f}s", elapsed), LogLevel::INFO, IndentLevel::ONE, true,
						   false);
}


// The main function for Laplacian skeletonization
Eigen::MatrixXd Skeleton::ContractionIteration() {
	// Save the original points
	tool::io::SavePointCloudToPLY(cloud_, output_path_ / "cpts_0.ply");

	// Start first time contraction
	Logger::Instance().Log("Contraction iteration time: 1", LogLevel::INFO, IndentLevel::ONE, true, false);

	// Initialize parameters
	InitializeParameters(cloud_);

	// Contract for the first time
	Eigen::MatrixXd cpts = LaplacianContraction(cloud_);

	// Check the contraction rate
	std::vector<double> contraction_history;
	GetMeanVertexDualArea(cloud_);
	GetMeanVertexDualArea(cpts);
	double contraction_rate = faces_area_.back() / faces_area_[0];
	contraction_history.emplace_back(contraction_rate);
	Logger::Instance().Log(std::format("Current mean area is: {:.6f}; the contract ratio is: {:.6f}", faces_area_.back(), contraction_rate),
						   LogLevel::INFO, IndentLevel::ONE, true, false);

	// Compute sigma for the contracted points
	ComputeSigma(cpts);

	// Save the contracted points
	tool::io::SavePointCloudToPLY(cpts, output_path_ / "cpts_1.ply", smooth_sigma_, "smooth-sigma");

	// Start the contraction iterations
	for (int i = 0; i < max_iteration_time_ - 1; ++i) {
		Logger::Instance().AddLine(LogLine::DASH);
		Logger::Instance().Log(std::format("Contraction iteration time: {}", i + 2), LogLevel::INFO, IndentLevel::ONE, true, false);

		// Update Laplacian matrix
		EstablishLaplacianMatrix(cpts);

		// Fix the points with high sigma
		for (int j = 0; j < pts_num_; ++j) {
			if (WH_.coeffRef(j) == tip_point_WH_ || WH_.coeffRef(j) == contracted_point_WH_) {	// Skip the tip points and the contracted points
				continue;
			} else if (smooth_sigma_.at(j) >= fix_eigen_ratio_) {
				WL_.coeffRef(j) = contracted_point_WL_;
				WH_.coeffRef(j) = contracted_point_WH_;
			} else if (smooth_sigma_.at(j) >= fix_eigen_ratio_ - 0.10) {
				continue;
			} else {
				WL_.coeffRef(j) = WL_.coeffRef(j) * sL_ > max_WL_ ? max_WL_ : WL_.coeffRef(j) * sL_;
			}
		}

		// Contract one more time
		Eigen::MatrixXd cpts_temp = LaplacianContraction(cpts);

		// Check the contraction rate
		GetMeanVertexDualArea(cpts_temp);
		contraction_rate = faces_area_.back() / faces_area_[0];
		contraction_history.emplace_back(contraction_rate);
		Logger::Instance().Log(std::format("Current mean area is: {:.6f}; the contract ratio is: {:.6f}", faces_area_.back(), contraction_rate),
							   LogLevel::INFO, IndentLevel::ONE, true, false);
		if (contraction_history[i] - contraction_history.back() <= contraction_threshold_ || std::isnan(contraction_history.back())) {
			Logger::Instance().Log(
					std::format("Touch the threshold! Iteration {} terminated! Total valid contraction iteration time: {}", i + 2, i + 1),
					LogLevel::INFO, IndentLevel::ONE, true, false);
			break;
		}

		// Update sigma
		ComputeSigma(cpts_temp);

		// Save the contracted points
		tool::io::SavePointCloudToPLY(cpts_temp, output_path_ / std::format("cpts_{}.ply", std::to_string(i + 2)), smooth_sigma_, "smooth-sigma");

		// Update cpts
		cpts = cpts_temp;
	}

	return cpts;
}



namespace geometrycentral::addition {
	void UnrequireAllQuantities(geometrycentral::pointcloud::PointPositionGeometry &gc_geom) {
		for (geometrycentral::DependentQuantity *q: gc_geom.quantities) {
			q->requireCount = 0;
		}
	}
}  // namespace geometrycentral::addition
