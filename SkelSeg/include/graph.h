#ifndef GRAPH_H
#define GRAPH_H


#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/properties.hpp>
#include <ranges>

#include "skeleton.h"



typedef boost::property<boost::edge_weight_t, double> Boost_EdgeWeightProperty;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Eigen::Vector3d, Boost_EdgeWeightProperty> Boost_Graph;
typedef boost::graph_traits<Boost_Graph>::adjacency_iterator Boost_AdjIt;
typedef boost::graph_traits<Boost_Graph>::vertex_descriptor Boost_Vertex;
typedef boost::graph_traits<Boost_Graph>::vertex_iterator Boost_VertexIt;
typedef boost::graph_traits<Boost_Graph>::edge_descriptor Boost_Edge;
typedef boost::graph_traits<Boost_Graph>::edge_iterator Boost_EdgeIt;
typedef boost::property_map<Boost_Graph, boost::edge_weight_t>::type Boost_WeightMap;



class Graph {
public:
	explicit Graph(const Eigen::MatrixXd &cloud, const nlohmann::json &config) : cloud_(cloud), config_(config) {}

	Boost_Graph GetInitialGraph() {
		Timer timer;
		GetSkeletonGraph();
		double elapsed = timer.elapsed<Timer::TimeUnit::Seconds>();
		Logger::Instance().Log(std::format("Initial graph has been generated! Elapsed time: {:.6f}s", elapsed), LogLevel::INFO, IndentLevel::ONE,
							   true, false);
		return graph_;
	}

	Boost_Graph GetMST() {
		Timer timer;
		while (graph_.m_vertices.empty()) {
			Logger::Instance().Log("Initial graph has not been generated yet, start generating initial graph", LogLevel::WARNING, IndentLevel::ONE,
								   true, false);
			GetSkeletonGraph();
		}
		ComputeMST();
		double elapsed = timer.elapsed<Timer::TimeUnit::Seconds>();
		Logger::Instance().Log(std::format("MST graph has been computed! Elapsed time: {}s", elapsed), LogLevel::INFO, IndentLevel::ONE, true, false);
		return mst_;
	}

	Boost_Graph GetPrunedMST() {
		while (mst_.m_vertices.empty()) {
			Logger::Instance().Log("MST graph has not been computed yet, start computing MST graph", LogLevel::WARNING);
			GetMST();
		}
		PruneMST();
		return pruned_mst_;
	}

	// Shoot is the first class in the "result" vector
	std::tuple<std::vector<int>, std::vector<int>> SegmentSkeleton() {
		Logger::Instance().AddLine(LogLine::DASH);
		Logger::Instance().Log("Start skeleton segmentation");
		Timer timer;
		while (pruned_mst_.m_vertices.empty()) {
			Logger::Instance().Log("Pruned MST graph has not been computed yet, start pruning MST graph", LogLevel::WARNING);
			GetPrunedMST();
		}
		GetRootNode();
		GetLeaves();
		GetShoot();
		std::vector<std::vector<Boost_Vertex>> classes = leafs_;
		classes.insert(classes.begin(), shoot_);
		std::vector<int> vertex_sematic_labels(num_vertices(pruned_mst_));
		std::vector<int> vertex_instance_labels(num_vertices(pruned_mst_));
		for (int i = 0; i < classes.size(); ++i) {
			for (const Boost_Vertex &index: classes[i]) {
				if (i == 0) {
					vertex_sematic_labels.at(index) = 0;	// Semantic label for shoot
					vertex_instance_labels.at(index) = -1;	// Instance label for shoot
					continue;
				}
				vertex_sematic_labels.at(index) = 1;		// Semantic label for leaf
				vertex_instance_labels.at(index) = i - 1;
			}
		}
		double elapsed = timer.elapsed<Timer::TimeUnit::Seconds>();
		Logger::Instance().Log(std::format("Skeleton segmentation has been finished! Elapsed time: {}s", elapsed), LogLevel::INFO, IndentLevel::ONE,
							   true, false);
		return std::make_pair(vertex_sematic_labels, vertex_instance_labels);
	}

private:
	Eigen::MatrixXd cloud_;
	nlohmann::json config_;

	Boost_Graph graph_;
	Boost_Graph mst_;
	Boost_Graph pruned_mst_;
	Boost_Vertex root_{};
	std::vector<std::vector<Boost_Vertex>> leafs_;
	std::vector<Boost_Vertex> shoot_;

	// Hyper-parameters
	// Recommend to change the following parameters in the ../configue.json file
	const double diagonal_length_ = config_["Preprocess"]["Normalize_Diagonal_Length"].get<double>();
	std::filesystem::path output_folder_path_ = config_["Output_Settings"]["Output_Folder_Path"].get<std::filesystem::path>();

	void GetSkeletonGraph();

	void ComputeMST();

	void PruneMST();

	void GetRootNode();

	void GetLeaves();

	void GetShoot();
};



#endif	// GRAPH_H
