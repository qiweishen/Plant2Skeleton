#ifndef EVALUATE_H
#define EVALUATE_H


#include <filesystem>

#include "tools.h"


namespace evaluate {
	namespace skeleton {
		namespace internal {
			double PointToSegmentDistance(const Eigen::Vector3d &point, const Eigen::Vector3d &segA, const Eigen::Vector3d &segB) {
				Eigen::Vector3d v = segB - segA;
				Eigen::Vector3d w = point - segA;

				double c1 = w.dot(v);
				if (c1 <= 0.0)
					return (point - segA).norm();

				double c2 = v.dot(v);
				if (c2 <= c1)
					return (point - segB).norm();

				double b = c1 / c2;
				Eigen::Vector3d Pb = segA + b * v;
				return (point - Pb).norm();
			}
		}  // namespace internal


		double QuantitativeSkeletonDistance(Eigen::MatrixXd &cloud, Eigen::MatrixXd &skeleton) {
			std::vector<std::vector<double>> original_points = tool::utility::Matrix2Vector<std::vector<double>>(cloud);
			std::vector<std::vector<double>> skeleton_points = tool::utility::Matrix2Vector<std::vector<double>>(skeleton);
			KDTree kdtree(skeleton_points);
			std::vector<double> distances;
			for (const std::vector<double> &vert: original_points) {
				std::vector<double> pt = kdtree.nearest_point(vert);
				double temp = (Eigen::Vector3d{ pt[0], pt[1], pt[2] } - Eigen::Vector3d{ vert[0], vert[1], vert[2] }).norm();
				distances.push_back(temp);
			}

			return std::accumulate(distances.begin(), distances.end(), 0.0) / static_cast<int>(distances.size());
		}


		double QuantitativeSkeletonDistance(Eigen::MatrixXd &cloud, Boost_Graph &skeleton) {
			std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges;
			auto edgePair = boost::edges(skeleton);
			for (auto it = edgePair.first; it != edgePair.second; ++it) {
				auto source = boost::source(*it, skeleton);
				auto target = boost::target(*it, skeleton);
				const Eigen::Vector3d &pointA = skeleton[source];
				const Eigen::Vector3d &pointB = skeleton[target];
				edges.emplace_back(pointA, pointB);
			}

			std::vector<std::vector<double>> midpoints;
			for (const auto &edge: edges) {
				const Eigen::Vector3d &pointA = edge.first;
				const Eigen::Vector3d &pointB = edge.second;
				Eigen::Vector3d midpoint = (pointA + pointB) / 2.0;
				midpoints.push_back({ midpoint.x(), midpoint.y(), midpoint.z() });
			}

			KDTree kdtree(midpoints);
			std::vector<std::vector<double>> original_points = tool::utility::Matrix2Vector<std::vector<double>>(cloud);

			size_t candidate_num = std::ceil(static_cast<int>(boost::num_vertices(skeleton)) * 0.05);
			std::vector<double> distances;
			for (const std::vector<double> &vert: original_points) {
				std::vector<size_t> candidate_pts_idx = kdtree.nearest_indices(vert, candidate_num);
				size_t closest_idx = 0;
				double min_distance = std::numeric_limits<double>::max();
				for (const size_t &pt_idx: candidate_pts_idx) {
					const std::vector<double> &pt = midpoints[pt_idx];
					double temp = (Eigen::Vector3d{ pt[0], pt[1], pt[2] } - Eigen::Vector3d{ vert[0], vert[1], vert[2] }).norm();
					if (temp < min_distance) {
						min_distance = temp;
						closest_idx = pt_idx;
					}
				}

				const Eigen::Vector3d point(vert[0], vert[1], vert[2]);
				const Eigen::Vector3d segA = edges[closest_idx].first;
				const Eigen::Vector3d segB = edges[closest_idx].second;
				double temp = internal::PointToSegmentDistance(point, segA, segB);
				distances.push_back(temp);
			}

			return std::accumulate(distances.begin(), distances.end(), 0.0) / static_cast<int>(distances.size());
		}
	}  // namespace skeleton



	namespace segmentation {
		// TODO: Integrate it with .py files in the SkelSeg/scripts
		void QuantitativeSegmentMetrics() {}
	}  // namespace segmentation



	namespace utility {
		Eigen::MatrixXd LoadPointCloudFromPLY(const std::filesystem::path &file_path) {
			std::ifstream ply_ifs{ file_path };
			if (!ply_ifs) {
				Logger::Instance().Log(std::format("Failed to open file ({}) for reading", file_path.string()), LogLevel::ERROR);
			}
			std::vector<Eigen::Vector3d> cloud_vertices;
			plywoot::IStream ply_in{ ply_ifs };
			while (ply_in.hasElement()) {
				if (const plywoot::PlyElement element{ ply_in.element() }; element.name() == "vertex") {
					using VertexLayout = plywoot::reflect::Layout<plywoot::reflect::Pack<double, 3>>;
					cloud_vertices = ply_in.readElement<Eigen::Vector3d, VertexLayout>();
				} else {
					ply_in.skipElement();
				}
			}
			Eigen::MatrixXd cloud = tool::utility::Vector2Matrix(cloud_vertices);
			return cloud;
		}


		Boost_Graph LoadGraphFromPLY(const std::filesystem::path &file_path) {
			std::ifstream ply_ifs{ file_path };
			if (!ply_ifs) {
				Logger::Instance().Log(std::format("Failed to open file ({}) for reading", file_path.string()), LogLevel::ERROR);
			}
			std::vector<Eigen::Vector3d> cloud_vertices;
			std::vector<Eigen::Vector2i> edge_indices;
			plywoot::IStream ply_in{ ply_ifs };
			while (ply_in.hasElement()) {
				if (const plywoot::PlyElement element{ ply_in.element() }; element.name() == "vertex") {
					using VertexLayout = plywoot::reflect::Layout<plywoot::reflect::Pack<double, 3>>;
					cloud_vertices = ply_in.readElement<Eigen::Vector3d, VertexLayout>();
				} else if (element.name() == "edge") {
					using EdgeLayout = plywoot::reflect::Layout<plywoot::reflect::Pack<int, 2>>;
					edge_indices = ply_in.readElement<Eigen::Vector2i, EdgeLayout>();
				}
			}
			Boost_Graph graph;
			for (const Eigen::Vector3d &vertex: cloud_vertices) {
				boost::add_vertex(vertex, graph);
			}
			for (const Eigen::Vector2i &edge: edge_indices) {
				boost::add_edge(edge.x(), edge.y(), graph);
			}
			return graph;
		}
	}  // namespace utility
}  // namespace evaluate



#endif	// EVALUATE_H
