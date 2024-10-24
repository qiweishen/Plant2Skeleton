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

	// Copy the MST to a modifiable graph
	boost::copy_graph(mst_, pruned_mst_);

	// Get the vertex index mapping
	auto index_map = boost::get(boost::vertex_index, pruned_mst_);
	size_t num_vertices_in_graph = num_vertices(pruned_mst_);
	size_t min_branch_length = std::ceil(min_branch_length_ratio_ * static_cast<int>(num_vertices_in_graph));

	// Initialize degrees of each vertex
	std::vector<size_t> degrees(num_vertices_in_graph);
	Boost_VertexIt vi, vi_end;
	for (std::tie(vi, vi_end) = vertices(pruned_mst_); vi != vi_end; ++vi) {
		size_t index = index_map[*vi];
		degrees[index] = degree(*vi, pruned_mst_);
	}

	// Mark vertices to remove
	std::vector<bool> to_remove(num_vertices_in_graph, false);
	// Track visited vertices
	std::vector<bool> visited(num_vertices_in_graph, false);

	// Iterate over each leaf node
	for (std::tie(vi, vi_end) = vertices(pruned_mst_); vi != vi_end; ++vi) {
		size_t index = index_map[*vi];
		if (degrees[index] == 1 && !visited[index]) {
			// Start traversal from the leaf node
			std::vector<Boost_Vertex> path;
			size_t path_length = 0;
			Boost_Vertex current = *vi;
			Boost_Vertex previous = Boost_Graph::null_vertex();
			bool should_remove = true;

			while (degrees[index_map[current]] <= 2) {
				path.push_back(current);
				visited[index_map[current]] = true;

				if (path_length >= min_branch_length) {
					should_remove = false;
					break;
				}

				// Find the next vertex
				Boost_AdjIt ai, ai_end;
				Boost_Vertex next = Boost_Graph::null_vertex();
				std::tie(ai, ai_end) = adjacent_vertices(current, pruned_mst_);
				for (; ai != ai_end; ++ai) {
					if (*ai != previous) {
						next = *ai;
						break;
					}
				}
				if (next == Boost_Graph::null_vertex()) {
					// No next vertex, end of path
					break;
				}

				previous = current;
				current = next;
				path_length += 1;
			}

			if (should_remove) {
				for (Boost_Vertex v: path) {
					// Mark vertex for removal
					to_remove[index_map[v]] = true;
				}
			}
		}
	}

	// Collect and remove marked vertices
	std::vector<Boost_Vertex> vertices_to_remove;
	for (std::tie(vi, vi_end) = vertices(pruned_mst_); vi != vi_end; ++vi) {
		size_t index = index_map[*vi];
		if (to_remove[index]) {
			vertices_to_remove.push_back(*vi);
		}
	}
	// Remove vertices in reverse order to avoid invalidating indices
	std::sort(vertices_to_remove.begin(), vertices_to_remove.end(), std::greater<>());
	for (Boost_Vertex v: vertices_to_remove) {
		clear_vertex(v, pruned_mst_);
		remove_vertex(v, pruned_mst_);
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
