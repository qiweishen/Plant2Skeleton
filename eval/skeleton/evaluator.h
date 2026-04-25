#ifndef EVALUATE_H
#define EVALUATE_H


#include <algorithm>
#include <filesystem>

#include "tools.h"


namespace evaluate {
	namespace skeleton {
		struct Metrics {
			double forward_mean;	// cloud → skeleton: mean distance
			double forward_90th;	// cloud → skeleton: 90th percentile
			double reverse_mean;	// skeleton → cloud: mean distance
			double chamfer;			// (forward_mean + reverse_mean) / 2
		};


		namespace internal {
			inline double PointToSegmentDistance(const Eigen::Vector3d &point, const Eigen::Vector3d &segA, const Eigen::Vector3d &segB) {
				Eigen::Vector3d v = segB - segA;
				Eigen::Vector3d w = point - segA;

				double c1 = w.dot(v);
				if (c1 <= 0.0) {
					return (point - segA).norm();
				}

				double c2 = v.dot(v);
				if (c2 <= c1) {
					return (point - segB).norm();
				}

				double b = c1 / c2;
				Eigen::Vector3d Pb = segA + b * v;
				return (point - Pb).norm();
			}


			/**
			 * @brief Adaptively sample points along edges, returning samples and their edge indices.
			 * Longer edges get more samples to ensure adequate KDTree representation.
			 */
			inline void SampleEdges(
				const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > &edges,
				double sampling_interval,
				std::vector<std::vector<double> > &samples,
				std::vector<size_t> &sample_to_edge
			) {
				for (size_t i = 0; i < edges.size(); ++i) {
					const Eigen::Vector3d &A = edges[i].first;
					const Eigen::Vector3d &B = edges[i].second;
					double length = (B - A).norm();
					size_t num_samples = std::max(static_cast<size_t>(2), static_cast<size_t>(std::ceil(length / sampling_interval)) + 1);
					for (size_t s = 0; s < num_samples; ++s) {
						double t = static_cast<double>(s) / static_cast<double>(num_samples - 1);
						Eigen::Vector3d pt = A + t * (B - A);
						samples.push_back({ pt.x(), pt.y(), pt.z() });
						sample_to_edge.push_back(i);
					}
				}
			}
		}  // namespace internal


		inline Metrics QuantitativeSkeletonDistance(Eigen::MatrixXd &cloud, Boost_Graph &skeleton, double sampling_interval = 0.0) {
			// Extract edges
			std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > edges;
			auto edgePair = boost::edges(skeleton);
			for (auto it = edgePair.first; it != edgePair.second; ++it) {
				auto source = boost::source(*it, skeleton);
				auto target = boost::target(*it, skeleton);
				edges.emplace_back(skeleton[source], skeleton[target]);
			}

			// Compute adaptive sampling interval if not specified
			if (sampling_interval <= 0.0) {
				double total_length = 0.0;
				for (const auto &edge : edges) {
					total_length += (edge.second - edge.first).norm();
				}
				sampling_interval = total_length / static_cast<double>(edges.size()) / 6.0;
			}

			// Adaptive edge sampling for KDTree
			std::vector<std::vector<double> > samples;
			std::vector<size_t> sample_to_edge;
			internal::SampleEdges(edges, sampling_interval, samples, sample_to_edge);

			KDTree sample_kdtree(samples);
			std::vector<std::vector<double> > cloud_points = tool::utility::Matrix2Vector<std::vector<double> >(cloud);

			// ── Forward: cloud → skeleton ──
			constexpr size_t k_candidates = 5;
			std::vector<double> forward_distances;
			forward_distances.reserve(cloud_points.size());
			for (const std::vector<double> &vert : cloud_points) {
				std::vector<size_t> candidate_idx = sample_kdtree.nearest_indices(vert, k_candidates);

				// Collect unique candidate edges
				std::vector<size_t> candidate_edges;
				for (size_t idx : candidate_idx) {
					size_t edge_idx = sample_to_edge[idx];
					if (std::ranges::find(candidate_edges, edge_idx) == candidate_edges.end()) {
						candidate_edges.push_back(edge_idx);
					}
				}

				const Eigen::Vector3d point(vert[0], vert[1], vert[2]);
				double min_dist = std::numeric_limits<double>::max();
				for (size_t edge_idx : candidate_edges) {
					double dist = internal::PointToSegmentDistance(point, edges[edge_idx].first, edges[edge_idx].second);
					min_dist = std::min(min_dist, dist);
				}
				forward_distances.push_back(min_dist);
			}

			std::ranges::sort(forward_distances);
			double forward_mean = std::accumulate(forward_distances.begin(), forward_distances.end(), 0.0)
								  / static_cast<double>(forward_distances.size());
			double forward_90th = forward_distances[static_cast<size_t>(forward_distances.size() * 0.9)];

			// ── Reverse: skeleton → cloud ──
			KDTree cloud_kdtree(cloud_points);
			std::vector<double> reverse_distances;
			reverse_distances.reserve(samples.size());
			for (const std::vector<double> &sample : samples) {
				std::vector<double> nearest = cloud_kdtree.nearest_point(sample);
				double dist = (Eigen::Vector3d{ nearest[0], nearest[1], nearest[2] }
							   - Eigen::Vector3d{ sample[0], sample[1], sample[2] }).norm();
				reverse_distances.push_back(dist);
			}

			double reverse_mean = std::accumulate(reverse_distances.begin(), reverse_distances.end(), 0.0)
								  / static_cast<double>(reverse_distances.size());

			return Metrics{
				.forward_mean = forward_mean,
				.forward_90th = forward_90th,
				.reverse_mean = reverse_mean,
				.chamfer = (forward_mean + reverse_mean) / 2.0,
			};
		}
	}  // namespace skeleton



	namespace utility {
		inline Eigen::MatrixXd LoadPointCloudFromPLY(const std::filesystem::path &file_path) {
			std::ifstream ply_ifs{ file_path };
			if (!ply_ifs) {
				Logger::Instance().Log(fmt::format("Failed to open file ({}) for reading", file_path.string()), LogLevel::ERROR);
			}
			std::vector<Eigen::Vector3d> cloud_vertices;
			plywoot::IStream ply_in{ ply_ifs };
			while (ply_in.hasElement()) {
				if (const plywoot::PlyElement element{ ply_in.element() }; element.name() == "vertex") {
					using VertexLayout = plywoot::reflect::Layout<plywoot::reflect::Pack<double, 3> >;
					cloud_vertices = ply_in.readElement<Eigen::Vector3d, VertexLayout>();
				} else {
					ply_in.skipElement();
				}
			}

			// Filter out vertices with NaN coordinates
			std::erase_if(cloud_vertices, [](const Eigen::Vector3d &v) {
				return !v.allFinite();
			});

			Eigen::MatrixXd cloud = tool::utility::Vector2Matrix(cloud_vertices);
			return cloud;
		}


		inline Boost_Graph LoadGraphFromPLY(const std::filesystem::path &file_path) {
			std::ifstream ply_ifs{ file_path };
			if (!ply_ifs) {
				Logger::Instance().Log(fmt::format("Failed to open file ({}) for reading", file_path.string()), LogLevel::ERROR);
			}
			std::vector<Eigen::Vector3d> cloud_vertices;
			std::vector<Eigen::Vector2i> edge_indices;
			plywoot::IStream ply_in{ ply_ifs };
			while (ply_in.hasElement()) {
				if (const plywoot::PlyElement element{ ply_in.element() }; element.name() == "vertex") {
					using VertexLayout = plywoot::reflect::Layout<plywoot::reflect::Pack<double, 3> >;
					cloud_vertices = ply_in.readElement<Eigen::Vector3d, VertexLayout>();
				} else if (element.name() == "edge") {
					using EdgeLayout = plywoot::reflect::Layout<plywoot::reflect::Pack<int, 2> >;
					edge_indices = ply_in.readElement<Eigen::Vector2i, EdgeLayout>();
				}
			}

			// Build old→new index mapping, skipping NaN vertices
			std::vector<int> index_remap(cloud_vertices.size(), -1);
			std::vector<Eigen::Vector3d> valid_vertices;
			for (size_t i = 0; i < cloud_vertices.size(); ++i) {
				if (cloud_vertices[i].allFinite()) {
					index_remap[i] = static_cast<int>(valid_vertices.size());
					valid_vertices.push_back(cloud_vertices[i]);
				}
			}

			Boost_Graph graph;
			for (const Eigen::Vector3d &vertex : valid_vertices) {
				boost::add_vertex(vertex, graph);
			}
			for (const Eigen::Vector2i &edge : edge_indices) {
				int src = edge.x(), tgt = edge.y();
				if (src >= 0 && src < static_cast<int>(index_remap.size()) &&
					tgt >= 0 && tgt < static_cast<int>(index_remap.size()) &&
					index_remap[src] >= 0 && index_remap[tgt] >= 0) {
					boost::add_edge(index_remap[src], index_remap[tgt], graph);
				}
			}
			return graph;
		}
	}  // namespace utility
}  // namespace evaluate


#endif	// EVALUATE_H
