#ifndef SKELETON_H
#define SKELETON_H

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <geometrycentral/pointcloud/point_cloud.h>
#include <geometrycentral/pointcloud/point_cloud_heat_solver.h>
#include <geometrycentral/pointcloud/point_position_geometry.h>
#include <geometrycentral/surface/edge_length_geometry.h>
#include <geometrycentral/surface/halfedge_factories.h>
#include <geometrycentral/surface/intrinsic_mollification.h>
#include <geometrycentral/surface/simple_idt.h>
#include <geometrycentral/surface/tufted_laplacian.h>
#include <memory>
#include <nlohmann/json.hpp>
#include <omp.h>
#include <string>
#include <unordered_set>
#include <utility/logger.h>
#include <utility/timer.h>
#include <vector>



struct VectorHash {
	std::size_t operator()(const std::vector<size_t> &v) const {
		std::size_t seed = v.size();
		for (const size_t &i: v) {
			seed ^= i + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		}
		return seed;
	}
};


class Skeleton {
public:
	explicit Skeleton(const Eigen::MatrixXd &cloud, const nlohmann::json &config) :
		cloud_(cloud),
		config_(config),
		WL_(Eigen::VectorXd(cloud.rows())),
		WH_(Eigen::VectorXd(cloud.rows())),
		sigma_(std::vector<double>(cloud.rows())),
		smooth_sigma_(std::vector<double>(cloud.rows())) {}

	Eigen::MatrixXd GetLaplacianSkeleton() {
		Logger::Instance().AddLine(LogLine::DASH);
		Logger::Instance().Log("Start Skeleton extraction", LogLevel::INFO);
		return ContractionIteration();
	}

private:
	Eigen::MatrixXd cloud_;
	nlohmann::json config_;

	const int pts_num_ = static_cast<int>(cloud_.rows());
	std::vector<int> tip_indices_;
	double threshold_edge_length_ = -1.0;
	std::vector<double> faces_area_;
	Eigen::SparseMatrix<double> L_;
	Eigen::VectorXd WL_;
	Eigen::VectorXd WH_;
	std::vector<double> sigma_;
	std::vector<double> smooth_sigma_;

	// Hyper-parameters
	// Do not recommend to change the following parameters
	const double initial_WL_ = 1.0;
	const double initial_WH_ = 1.0;
	const double sL_ = 3.0;
	const double tip_point_WH_ = 9.0;
	const double contracted_point_WL_ = 0.0;
	const double contracted_point_WH_ = 3.0;
	const double max_WL_ = 9.0;
	const int k_neighbors_distance_constraint_ = 4;
	const double WD_ = 9.0;

	// Recommend to change the following parameters in the ../configure.json file
	const double diagonal_length_ = config_["Preprocess"]["Normalize_Diagonal_Length"].get<double>();
	const bool use_knn_search_ = config_["Constraint_Laplacian_Operator"]["Use_KNN_Search"].get<bool>();
	int k_neighbors_ = config_["Constraint_Laplacian_Operator"]["Initial_k"].get<int>();
	const int delta_k_ = config_["Constraint_Laplacian_Operator"]["Delta_k"].get<int>();
	const int max_k_ = config_["Constraint_Laplacian_Operator"]["Max_k"].get<int>();
	double radius_neighbors_ = config_["Constraint_Laplacian_Operator"]["Initial_Radius_Search_Ratio"].get<double>() * diagonal_length_;
	const double delta_radius_ = config_["Constraint_Laplacian_Operator"]["Delta_Radius_Search_Ratio"].get<double>() * diagonal_length_;
	const double min_radius_ = config_["Constraint_Laplacian_Operator"]["Min_Radius_Search_Ratio"].get<double>() * diagonal_length_;
	const double fix_eigen_ratio_ = config_["Adaptive_Contraction"]["Smooth_Sigma_Threshold"].get<double>();
	const double sigma_radius_ = config_["Adaptive_Contraction"]["Sigma_Sphere_Radius_Ratio"].get<double>() * diagonal_length_;
	const double max_distance_ = config_["Adaptive_Contraction"]["Max_Distance_Ratio"].get<double>() * diagonal_length_;
	const int max_iteration_time_ = config_["Terminate_Condition"]["Max_Iteration"].get<int>();
	const double contraction_threshold_ = config_["Terminate_Condition"]["Convergence_Threshold"].get<double>();
	const std::filesystem::path output_path_ = config_["Output_Settings"]["Output_Folder_Path"].get<std::filesystem::path>() / ".iterations";


	void InitializeParameters(const Eigen::MatrixXd &cloud);

	void EstablishLaplacianMatrix(const Eigen::MatrixXd &cloud);

	void FindTipPoints();

	void UpdateNeighborhood(geometrycentral::pointcloud::PointCloud &gc_cloud, geometrycentral::pointcloud::PointPositionGeometry &gc_geom);

	void FindEdgeThreshold(geometrycentral::pointcloud::PointCloud &gc_cloud, geometrycentral::pointcloud::PointPositionGeometry &gc_geom);

	void CollapseEdges(geometrycentral::pointcloud::PointCloud &gc_cloud, geometrycentral::pointcloud::PointPositionGeometry &gc_geom);

	std::tuple<Eigen::SparseMatrix<double>, Eigen::MatrixXd> DistanceConstraint(const Eigen::MatrixXd &cloud);

	Eigen::MatrixXd LaplacianContraction(const Eigen::MatrixXd &cloud);

	void GetMeanVertexDualArea(const Eigen::MatrixXd &cloud);

	void ComputeSigma(const Eigen::MatrixXd &cloud);

	Eigen::MatrixXd ContractionIteration();
};



namespace geometrycentral::addition {
	void UnrequireAllQuantities(geometrycentral::pointcloud::PointPositionGeometry &gc_geom);
}



#endif	// SKELETON_H
