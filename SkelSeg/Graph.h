#ifndef CPPTEST_GRAPH_H
#define CPPTEST_GRAPH_H

#include "Skeleton.h"

#include <ranges>

#include <boost/graph/properties.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>


typedef boost::property<boost::edge_weight_t, double> Boost_EdgeWeightProperty;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Eigen::Vector3d, Boost_EdgeWeightProperty> Boost_Graph;
typedef boost::graph_traits<Boost_Graph>::adjacency_iterator Boost_AdjIt;
typedef boost::graph_traits<Boost_Graph>::vertex_descriptor Boost_Vertex;
typedef boost::graph_traits<Boost_Graph>::vertex_iterator Boost_VertexIt;
typedef boost::graph_traits<Boost_Graph>::edge_descriptor Boost_Edge;
typedef boost::graph_traits<Boost_Graph>::edge_iterator Boost_EdgeIt;
typedef boost::property_map<Boost_Graph, boost::edge_weight_t>::type Boost_WeightMap;


struct JunctionFoundException : std::exception {
    const char *what() const throw() {
        return "Junction found";
    }
};

struct bfs_visitor : public boost::default_bfs_visitor {
    Boost_Vertex end_vertex;
    bool &found;

    bfs_visitor(Boost_Vertex v, bool &found) : end_vertex(v), found(found) {}

    template<typename Vertex, typename Graph>
    void discover_vertex(Vertex u, const Graph &g) {
        if (u == end_vertex) {
            found = true;
            throw u; // Throw to stop the BFS when the junction is reached
        }
    }
};


class Graph {
public:
    explicit Graph(std::shared_ptr<Eigen::MatrixXd> &cloudPtr, double diagonal_length) :
            cloudPtr_(cloudPtr),
            diagonal_length_(diagonal_length),
            graphPtr_(std::make_shared<Boost_Graph>()),
            mstPtr_(std::make_shared<Boost_Graph>()),
            pruned_mstPtr_(std::make_shared<Boost_Graph>()) {}

    std::shared_ptr<Boost_Graph> GetMST() {
        GetSkeletonGraph();
        ComputeMST();
        return mstPtr_;
//        return graphPtr_;
    }

    std::shared_ptr<Boost_Graph> GetPrunedMST() {
        if (mstPtr_ == nullptr) {
            std::cerr << "MST graph is not computed yet" << std::endl;
            return nullptr;
        }
        PruneMST();
        return pruned_mstPtr_;
    }

    // Shoot is the first class in the "result" vector
    std::vector<std::vector<Boost_Vertex>> SegmentSkeleton() {
        std::cout << "--------------------------------------------------" << std::endl;
        std::cout << "Start segment skeleton" << std::endl;
        if (pruned_mstPtr_ == nullptr) {
            std::cerr << "Pruned MST graph is not computed yet" << std::endl;
            return {};
        }
        GetRootNode();
        GetLeaves();
        GetShoot();
        std::vector<std::vector<Boost_Vertex>> result = leafs_;
        result.insert(result.begin(), shoot_);
        return result;
    }

private:
    std::shared_ptr<Eigen::MatrixXd> cloudPtr_;
    double diagonal_length_;

    std::shared_ptr<Boost_Graph> graphPtr_;
    std::shared_ptr<Boost_Graph> mstPtr_;
    std::shared_ptr<Boost_Graph> pruned_mstPtr_;
    Boost_Vertex root_{};
    std::vector<std::vector<Boost_Vertex>> leafs_;
    std::vector<Boost_Vertex> shoot_;

    void GetSkeletonGraph();

    void ComputeMST();

    void PruneMST();

    void GetRootNode();

    void GetLeaves();

    void GetShoot();
};


#endif //CPPTEST_GRAPH_H
