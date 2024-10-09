#ifndef GRAPH_H
#define GRAPH_H


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



class Graph {
public:
    explicit Graph(std::shared_ptr<Eigen::MatrixXd> &cloudPtr, nlohmann::json &config) :
            cloudPtr_(cloudPtr),
            config_(config),
            graphPtr_(std::make_shared<Boost_Graph>()),
            mstPtr_(std::make_shared<Boost_Graph>()),
            pruned_mstPtr_(std::make_shared<Boost_Graph>()) {}

    std::shared_ptr<Boost_Graph> GetMST() {
        GetSkeletonGraph();
        ComputeMST();
        return mstPtr_;
    }

    std::shared_ptr<Boost_Graph> GetPrunedMST() {
        if (mstPtr_ == nullptr) {
            MyLogger.Log("MST graph is not computed yet.", 1, true, true);
            GetMST();
            GetPrunedMST();
        }
        PruneMST();
        return pruned_mstPtr_;
    }

    // Shoot is the first class in the "result" vector
    std::vector<std::vector<Boost_Vertex>> SegmentSkeleton() {
        MyLogger.Log("--------------------------------------------------", 0, true, false);
        MyLogger.Log("Start segment skeleton", 0, true, true);
        auto start = std::chrono::high_resolution_clock::now();
        if (pruned_mstPtr_ == nullptr) {
            MyLogger.Log("Pruned MST graph is not computed yet.", 1, true, true);
            GetPrunedMST();
        }
        GetRootNode();
        GetLeaves();
        GetShoot();
        std::vector<std::vector<Boost_Vertex>> result = leafs_;
        result.insert(result.begin(), shoot_);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        MyLogger.Log(std::format("Skeleton segmentation finished! Elapsed time: {}s.", elapsed.count()), 0, true, false);
        return result;
    }

private:
    std::shared_ptr<Eigen::MatrixXd> cloudPtr_;
    nlohmann::json config_;

    const double diagonal_length_ = config_["Preprocess"]["Normalize_Diagonal_Length"].get<double>();
    std::filesystem::path output_folder_path_ = config_["Output_Settings"]["Output_Folder_Path"].get<std::filesystem::path>();

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



#endif // GRAPH_H
