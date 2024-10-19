#include "graph.h"

#include "tools.h"



void Graph::GetSkeletonGraph() {
	Logger::Instance().AddLine(LogLine::DASH);
	Logger::Instance().Log("Start generate initial graph", LogLevel::INFO);
	Timer timer;

	graph_.clear();

	std::vector<std::vector<double>> cloud_vertices = tool::utility::Matrix2Vector<std::vector<double>>(cloud_);
	KDTree kdtree(cloud_vertices);

	double radius = 0.001 * diagonal_length_;  // Start from a small radius

	// Use the skeleton points as the vertices of the graph
	for (int i = 0; i < cloud_.rows(); ++i) {
		boost::add_vertex(cloud_.row(i), graph_);
	}

	int num = std::numeric_limits<int>::max();
	while (num > 1) {
		// Increase the search radius of each node to make connections
		for (int i = 0; i < cloud_.rows(); ++i) {
			std::vector<size_t> indices;
			indices = kdtree.neighborhood_indices(cloud_vertices[i], radius);
			indices.erase(indices.begin());

			for (const size_t &index: indices) {
				if (auto [fst, snd] = edge(i, index, graph_); !snd) {
					boost::add_edge(i, index, graph_);
				}
			}
		}
		// Check the connected components
		std::vector component(num_vertices(graph_), 0);
		num = connected_components(graph_, component.data());
		radius += 0.001 * diagonal_length_;
	}

	Boost_WeightMap weight_map = get(boost::edge_weight, graph_);
	Boost_Graph::edge_iterator ei, ei_end;

	// Euclidean distance based on the current point cloud
	for (std::tie(ei, ei_end) = edges(graph_); ei != ei_end; ++ei) {
		Boost_Vertex u = source(*ei, graph_);
		Boost_Vertex v = target(*ei, graph_);
		double dist = (graph_[u] - graph_[v]).norm();  //(a - b).norm();
		put(weight_map, *ei, dist);
	}

	double elapsed = timer.elapsed<Timer::TimeUnit::Seconds>();
	Logger::Instance().Log(std::format("Initial graph has been generated! Elapsed time: {:.6f}s", elapsed), LogLevel::INFO, IndentLevel::ONE, true,
						   false);
}


void Graph::ComputeMST() {
	Logger::Instance().AddLine(LogLine::DASH);
	Logger::Instance().Log("Start compute MST", LogLevel::INFO);
	Timer timer;

	mst_.clear();

	// Compute the MST edges
	std::vector<Boost_Edge> mst_edges;
	kruskal_minimum_spanning_tree(graph_, std::back_inserter(mst_edges));
	// Copy vertices
	for (int i = 0; i < graph_.m_vertices.size(); ++i) {
		boost::add_vertex(graph_[i], mst_);
	}
	// Copy the MST edges and their weights to the new graph
	for (const Boost_Edge &edge: mst_edges) {
		Boost_Vertex u = source(edge, graph_);
		Boost_Vertex v = target(edge, graph_);
		Boost_WeightMap weight_map = get(boost::edge_weight, graph_);
		double weight = get(weight_map, edge);
		// Ensure the edge property is correctly constructed if more properties exist
		boost::add_edge(u, v, Boost_EdgeWeightProperty(weight), mst_);
	}

	double elapsed = timer.elapsed<Timer::TimeUnit::Seconds>();
	Logger::Instance().Log(std::format("MST has been computed! Elapsed time: {:.6f}s", elapsed), LogLevel::INFO, IndentLevel::ONE, true, false);
}


void Graph::PruneMST() {
	Logger::Instance().AddLine(LogLine::DASH);
	Logger::Instance().Log("Start prune MST", LogLevel::INFO);
	Timer timer;

	pruned_mst_.clear();

	Boost_Graph g = mst_;
	std::vector<size_t> degrees(num_vertices(g), 0);
	Boost_VertexIt vi, vi_end;
	for (std::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi) {
		degrees[*vi] = degree(*vi, g);
	}

	std::vector removal_map(num_vertices(g), -1);
	int num_remaining = 0;
	// Find and remove edges
	std::vector<Boost_Vertex> vertices_to_remove;
	for (std::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi) {
		if (degrees[*vi] == 1) {
			Boost_AdjIt ai, ai_end;
			for (std::tie(ai, ai_end) = adjacent_vertices(*vi, g); ai != ai_end; ++ai) {
				if (degrees[*ai] > 2) {
					Boost_Edge e;
					bool exists;
					std::tie(e, exists) = edge(*vi, *ai, g);
					if (exists) {
						remove_edge(e, g);
						vertices_to_remove.emplace_back(*vi);
						break;
					}
					removal_map[*vi] = num_remaining++;
				}
			}
		}
	}

	std::ranges::sort(vertices_to_remove);
	vertices_to_remove.erase(std::ranges::unique(vertices_to_remove).begin(), vertices_to_remove.end());
	for (unsigned long &it: std::ranges::reverse_view(vertices_to_remove)) {
		clear_vertex(it, g);
		remove_vertex(it, g);
	}

	// Build the pruned MST
	std::map<Boost_Vertex, Boost_Vertex> index_map;
	// Add vertices
	for (std::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi) {
		Boost_Vertex new_vertex = boost::add_vertex(pruned_mst_);
		pruned_mst_.m_vertices[new_vertex] = g[*vi];
		index_map[*vi] = new_vertex;
	}
	// Add edges
	Boost_EdgeIt ei, ei_end;
	for (std::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei) {
		Boost_Vertex u = index_map[source(*ei, g)];
		Boost_Vertex v = index_map[target(*ei, g)];
		auto [fst, snd] = boost::add_edge(u, v, pruned_mst_);
		pruned_mst_[fst] = g[*ei];
	}

	double elapsed = timer.elapsed<Timer::TimeUnit::Seconds>();
	Logger::Instance().Log(std::format("MST has been pruned! Elapsed time: {:.6f}s", elapsed), LogLevel::INFO, IndentLevel::ONE, true, false);
}


void Graph::GetRootNode() {
	root_ = -1;

	Boost_Graph g = pruned_mst_;
	auto [fst, snd] = vertices(g);
	double lowestZ = std::numeric_limits<double>::max();

	for (auto vp = fst; vp != snd; ++vp) {
		if (Eigen::Vector3d currentPos = g[*vp]; currentPos.z() < lowestZ) {
			lowestZ = currentPos.z();
			root_ = *vp;
		}
	}

	assert(root_ != -1);
}


void Graph::GetLeaves() {
	leafs_.clear();
	Boost_Graph g = pruned_mst_;

	std::vector is_leaf(num_vertices(g), false);
	std::vector is_junction(num_vertices(g), false);

	Boost_VertexIt vi, vi_end;
	for (std::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi) {
		if (size_t degree = boost::degree(*vi, g); degree == 1) {
			is_leaf[*vi] = true;
		} else if (degree > 2) {
			is_junction[*vi] = true;
		}
	}

	// Collect paths from each leaf to the nearest junction
	for (std::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi) {
		if (is_leaf[*vi] && *vi != root_) {
			std::vector<Boost_Vertex> path;
			Boost_Vertex current = *vi;
			path.emplace_back(current);

			// Traverse until a junction node is found
			while (!is_junction[current]) {
				Boost_AdjIt ai, ai_end;
				std::tie(ai, ai_end) = adjacent_vertices(current, g);
				for (; ai != ai_end; ++ai) {
					if (std::ranges::find(path, *ai) == path.end()) {
						current = *ai;
						path.emplace_back(current);
						break;
					}
				}
			}
			// Remove the last node as it is a junction
			path.pop_back();
			leafs_.emplace_back(path);
		}
	}
}


void Graph::GetShoot() {
	shoot_.clear();

	std::unordered_set<Boost_Vertex> leaf_indices;
	for (const std::vector<Boost_Vertex> &leaf: leafs_) {
		for (Boost_Vertex i: leaf) {
			leaf_indices.insert(i);
		}
	}

	Boost_Graph g = pruned_mst_;
	Boost_VertexIt vi, vi_end;
	for (std::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi) {
		if (!leaf_indices.contains(*vi)) {
			shoot_.emplace_back(*vi);
		}
	}
}
