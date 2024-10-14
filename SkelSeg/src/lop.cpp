#include "lop.h"

#include "tools.h"



double GetAlpha(const Eigen::Vector3d &point_xi, const Eigen::Vector3d &point_pj, const double &h) {
	double r = (point_xi - point_pj).cwiseAbs().sum() + 1e-8;
	double numerator = exp(-pow(r, 2) / pow(h / 4, 2));
	return numerator / r;
}


Eigen::Vector3d GetFirstTerm(const Eigen::Vector3d &point_xi, const double &h, const std::vector<size_t> &neighbor_indices_raw,
							 const Eigen::MatrixXd &cloud) {
	Eigen::Vector3d numerator(0, 0, 0);
	double denominator = 0.0;
	for (const size_t &neighbor_index: neighbor_indices_raw) {
		Eigen::Vector3d point_pj = cloud.row(static_cast<int>(neighbor_index));
		double alpha = GetAlpha(point_xi, point_pj, h);
		numerator += point_pj * alpha;
		denominator += alpha;
	}
	return numerator / denominator;
}


Eigen::MatrixXd LOPCalibrate(const Eigen::MatrixXd &cloud, const Eigen::MatrixXd &skeleton_cloud, const nlohmann::json &config) {
	Logger::Instance().AddLine(LogLine::DASH);
	Logger::Instance().Log("Start LOP Iteration", LogLevel::INFO);
	Timer timer;
	const int n = skeleton_cloud.rows();
	Eigen::MatrixXd new_skeleton_cloud(n, 3);
	std::vector<geometrycentral::Vector3> original_cloud_points = tool::utility::Matrix2Vector<geometrycentral::Vector3>(cloud);
	geometrycentral::NearestNeighborFinder original_cloud_finder(original_cloud_points);
	std::vector<geometrycentral::Vector3> skeleton_cloud_points = tool::utility::Matrix2Vector<geometrycentral::Vector3>(skeleton_cloud);
	double h = config["Preprocess"]["Normalize_Diagonal_Length"].get<double>() * config["Skeleton_Building"]["LOP_Sphere_Radius_Ratio"].get<double>();
#pragma omp parallel for
	for (int i = 0; i < n; ++i) {
		Eigen::Vector3d query_point = skeleton_cloud.row(i);
		geometrycentral::Vector3 query_point_gc = skeleton_cloud_points[i];
		std::vector<size_t> original_point_cloud_indices;
		original_point_cloud_indices = original_cloud_finder.radiusSearch(query_point_gc, h);
		// No need to move the skeleton point if it does not have any neighbors in the original point cloud.
		if (original_point_cloud_indices.empty()) {
			new_skeleton_cloud.row(i) = cloud.row(i);
			continue;
		}
		Eigen::Vector3d first_term = GetFirstTerm(query_point, h, original_point_cloud_indices, cloud);
		if (first_term.hasNaN()) {
			Logger::Instance().Log(std::format("NaN value in LOP calibration! Index: {}", i), LogLevel::WARNING);
			Logger::Instance().Log(std::format("L1 term: {}, {}, {}", first_term(0), first_term(1), first_term(2)), LogLevel::ERROR);
		}
		new_skeleton_cloud.row(i) = Eigen::Vector3d(first_term);
	}
	double elapsed = timer.elapsed<Timer::TimeUnit::Seconds>();
	Logger::Instance().Log(std::format("LOP-based Calibration complete! Elapsed time: {:.6f}s", elapsed), LogLevel::INFO, IndentLevel::ONE, true,
						   false);
	return new_skeleton_cloud;
}
