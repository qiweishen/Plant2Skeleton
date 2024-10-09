#include "Graph.h"
#include "Tools.h"


void Graph::GetSkeletonGraph() {
    MyLogger.Log("--------------------------------------------------", 0, true, false);
    MyLogger.Log("Start generate initial graph", 0, true, true);
    auto start = std::chrono::high_resolution_clock::now();

    graphPtr_->clear();

    std::vector<geometrycentral::Vector3> cloud_points = tool::utility::Matrix2GCVector(cloudPtr_);
    geometrycentral::NearestNeighborFinder cloud_finder(cloud_points);

    double radius = 0.001 * diagonal_length_; // Start from a small radius

    // Use the skeleton points as the vertices of the graph
    for (int i = 0; i < cloudPtr_->rows(); ++i) {
        boost::add_vertex(cloudPtr_->row(i), *graphPtr_);
    }

    int num = std::numeric_limits<int>::max();
    while (num > 1) {
        // Increase the search radius of each node to make connections
        for (int i = 0; i < cloudPtr_->rows(); ++i) {
            std::vector<size_t> indices;
            indices = cloud_finder.radiusSearch(cloud_points[i], radius);
            indices.erase(indices.begin());

            for (const size_t &index: indices) {
                std::pair<boost::graph_traits<Boost_Graph>::edge_descriptor, bool> edge_check = boost::edge(i, index,
                                                                                                            *graphPtr_);
                if (!edge_check.second) {
                    boost::add_edge(i, index, *graphPtr_);
                }
            }
        }
        // Check the connected components
        std::vector<int> component(boost::num_vertices(*graphPtr_), 0);
        num = boost::connected_components(*graphPtr_, component.data());
        radius += 0.001 * diagonal_length_;
    }

    Boost_WeightMap weight_map = boost::get(boost::edge_weight, *graphPtr_);
    Boost_Graph::edge_iterator ei, ei_end;

    // Euclidean distance based on the current point cloud
    for (std::tie(ei, ei_end) = boost::edges(*graphPtr_); ei != ei_end; ++ei) {
        Boost_Vertex u = boost::source(*ei, *graphPtr_);
        Boost_Vertex v = boost::target(*ei, *graphPtr_);
        double dist = ((*graphPtr_)[u] - (*graphPtr_)[v]).norm(); //(a - b).norm();
        boost::put(weight_map, *ei, dist);
    }

    tool::io::SaveSkeletonGraphToPLY(graphPtr_, output_folder_path_ / "_Initial_Graph.ply");

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    MyLogger.Log(std::format("Initial has been graph generated! Elapsed time: {:.6f}s.", elapsed.count()), 0, true, false);
}


void Graph::ComputeMST() {
    MyLogger.Log("--------------------------------------------------", 0, true, false);
    MyLogger.Log("Start compute MST", 0, true, true);
    auto start = std::chrono::high_resolution_clock::now();

    mstPtr_->clear();

    // Compute the MST edges
    std::vector<Boost_Edge> mst_edges;
    boost::kruskal_minimum_spanning_tree(*graphPtr_, std::back_inserter(mst_edges));
    // Copy vertices
    for (int i = 0; i < graphPtr_->m_vertices.size(); ++i) {
        add_vertex((*graphPtr_)[i], *mstPtr_);
    }
    // Copy the MST edges and their weights to the new graph
    for (const Boost_Edge &edge: mst_edges) {
        Boost_Vertex u = boost::source(edge, *graphPtr_);
        Boost_Vertex v = boost::target(edge, *graphPtr_);
        Boost_WeightMap weight_map = boost::get(boost::edge_weight, *graphPtr_);
        double weight = boost::get(weight_map, edge);
        // Ensure the edge property is correctly constructed if more properties exist
        boost::add_edge(u, v, Boost_EdgeWeightProperty(weight), *mstPtr_);
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    MyLogger.Log(std::format("MST has been computed! Elapsed time: {:.6f}s.", elapsed.count()), 0, true, false);
}


void Graph::PruneMST() {
    MyLogger.Log("--------------------------------------------------", 0, true, false);
    MyLogger.Log("Start prune MST", 0, true, true);
    auto start = std::chrono::high_resolution_clock::now();

    pruned_mstPtr_->clear();

    Boost_Graph g = *mstPtr_;
    std::vector<size_t> degrees(boost::num_vertices(g), 0);
    Boost_VertexIt vi, vi_end;
    for (std::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi) {
        degrees[*vi] = boost::degree(*vi, g);
    }

    std::vector<int> removal_map(boost::num_vertices(g), -1);
    int num_remaining = 0;
    // Find and remove edges
    std::vector<Boost_Vertex> vertices_to_remove;
    for (std::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi) {
        if (degrees[*vi] == 1) {
            Boost_AdjIt ai, ai_end;
            for (std::tie(ai, ai_end) = boost::adjacent_vertices(*vi, g); ai != ai_end; ++ai) {
                if (degrees[*ai] > 2) {
                    Boost_Edge e;
                    bool exists;
                    std::tie(e, exists) = boost::edge(*vi, *ai, g);
                    if (exists) {
                        boost::remove_edge(e, g);
                        vertices_to_remove.emplace_back(*vi);
                        break;
                    } else {
                        removal_map[*vi] = num_remaining++;
                    }
                }
            }
        }
    }

    std::sort(vertices_to_remove.begin(), vertices_to_remove.end());
    vertices_to_remove.erase(std::unique(vertices_to_remove.begin(), vertices_to_remove.end()),
                             vertices_to_remove.end());
    for (unsigned long &it: std::ranges::reverse_view(vertices_to_remove)) {
        boost::clear_vertex(it, g);
        boost::remove_vertex(it, g);
    }

    // Build the pruned MST
    std::map<Boost_Vertex, Boost_Vertex> index_map;
    // Add vertices
    for (std::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi) {
        Boost_Vertex new_vertex = boost::add_vertex(*pruned_mstPtr_);
        pruned_mstPtr_->m_vertices[new_vertex] = g[*vi];
        index_map[*vi] = new_vertex;
    }
    // Add edges
    Boost_EdgeIt ei, ei_end;
    for (std::tie(ei, ei_end) = boost::edges(g); ei != ei_end; ++ei) {
        Boost_Vertex u = index_map[boost::source(*ei, g)];
        Boost_Vertex v = index_map[boost::target(*ei, g)];
        auto new_edge = boost::add_edge(u, v, *pruned_mstPtr_);
        (*pruned_mstPtr_)[new_edge.first] = g[*ei];
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    MyLogger.Log(std::format("MST has been pruned! Elapsed time: {:.6f}s.", elapsed.count()), 0, true, false);
}


void Graph::GetRootNode() {
    root_ = -1;

    Boost_Graph g = *pruned_mstPtr_;
    auto vertices = boost::vertices(g);
    double lowestZ = std::numeric_limits<double>::max();

    for (auto vp = vertices.first; vp != vertices.second; ++vp) {
        Eigen::Vector3d currentPos = g[*vp];
        if (currentPos.z() < lowestZ) {
            lowestZ = currentPos.z();
            root_ = *vp;
        }
    }

    assert(root_ != -1);
}


void Graph::GetLeaves() {
    leafs_.clear();
    Boost_Graph g = *pruned_mstPtr_;

    std::vector<bool> is_leaf(boost::num_vertices(g), false);
    std::vector<bool> is_junction(boost::num_vertices(g), false);

    Boost_VertexIt vi, vi_end;
    for (std::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi) {
        size_t degree = boost::degree(*vi, g);
        if (degree == 1) {
            is_leaf[*vi] = true;
            Eigen::Vector3d point = g[*vi];
        } else if (degree > 2) {
            is_junction[*vi] = true;
            Eigen::Vector3d point = g[*vi];
        }
    }

    // Collect paths from each leaf to the nearest junction
    for (std::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi) {
        if (is_leaf[*vi] && *vi != root_) {
            std::vector<Boost_Vertex> path;
            Boost_Vertex current = *vi;
            path.emplace_back(current);

            // Traverse until a junction node is found
            while (!is_junction[current]) {
                Boost_AdjIt ai, ai_end;
                std::tie(ai, ai_end) = boost::adjacent_vertices(current, g);
                for (; ai != ai_end; ++ai) {
                    if (std::find(path.begin(), path.end(), *ai) == path.end()) {
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

    Boost_Graph g = *pruned_mstPtr_;
    Boost_VertexIt vi, vi_end;
    for (std::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi) {
        if (leaf_indices.find(*vi) == leaf_indices.end()) {
            shoot_.emplace_back(*vi);
        }
    }
}
