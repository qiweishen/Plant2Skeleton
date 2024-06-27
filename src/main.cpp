#include "Tools.h"
#include "Skeleton.h"
#include "Graph.h"
#include "LOP.h"

#include <filesystem>


int main() {
    std::vector<std::string> file_names = {"C20-2"}; // "C20-2", "C20-4", "Ca46-1", "Ca46-5", "Ca11-2"
    for (const std::string &file_name: file_names) {
//        std::string file_path = "../data/labelled_binary/" + file_name + ".ply";
        std::string file_path = "../../data/Skeleton/" + file_name + "/" + file_name + "_Downsampled.ply";
        std::string folder_path = "../../data/Skeleton/" + file_name + "/" + "New" + "/";
        if (std::filesystem::exists(folder_path)) {
            std::filesystem::remove_all(folder_path);
        }
        std::filesystem::create_directory(folder_path);


//        std::shared_ptr<PointCloud> raw_cloudPtr = std::make_shared<PointCloud>();
//        std::shared_ptr<Eigen::MatrixXd> raw_propertyPtr = std::make_shared<Eigen::MatrixXd>();
//        std::vector<std::pair<std::string, std::string>> raw_property_names = std::vector<std::pair<std::string, std::string>>();
//        tool::io::LoadPointCloudFromPLY(file_path, raw_cloudPtr, raw_propertyPtr, raw_property_names);
//        std::pair<Eigen::Vector3d, double> _ = tool::preprocess::Normalize(raw_cloudPtr, 1.6);


        std::shared_ptr<PointCloud> cloudPtr = std::make_shared<PointCloud>();
        std::shared_ptr<Eigen::MatrixXd> __ = std::make_shared<Eigen::MatrixXd>();
        std::vector<std::pair<std::string, std::string>> ___;
        tool::io::LoadPointCloudFromPLY(file_path, cloudPtr, __, ___);


//        tool::debug::SaveSphereToPLY(cloudPtr->points_[58], 0.02 * 1.6, folder_path + "002sphere.ply");


        // Laplacian-based contraction
//        Skeleton skeleton(cloudPtr, 30, 0.95, 1.6, folder_path + file_name);
//        std::shared_ptr<PointCloud> skeletonPtr = skeleton.GetLaplacianSkeleton();
//        tool::io::SavePointCloudToPLY(folder_path + file_name + "_PCA_knn.ply", skeletonPtr, _, __);
//
//
        std::shared_ptr<PointCloud> skeletonPtr = std::make_shared<PointCloud>();
        tool::io::LoadPointCloudFromPLY("../../data/Skeleton/C20-2/TuftedMeshLaplacian-2/C20-2_cpts_9.ply", skeletonPtr, __, ___);
        // Down-sample the skeleton
        std::shared_ptr<PointCloud> skeleton_points_cloudPtr = skeletonPtr->VoxelDownSample(0.005 * 1.6);


        // LOP to calibrate the down-sampled skeleton
        std::shared_ptr<PointCloud> final_skeleton_points_cloudPtr = LOPIterate(cloudPtr, skeleton_points_cloudPtr);
        std::shared_ptr<Eigen::MatrixXd> ____ = std::make_shared<Eigen::MatrixXd>();
        std::vector<std::pair<std::string, std::string>> _____;
        tool::io::SavePointCloudToPLY(folder_path + file_name + "_LOP.ply", final_skeleton_points_cloudPtr, ____, _____);


        // Compute MST
        Graph graph(final_skeleton_points_cloudPtr, 1.6);
        std::shared_ptr<Boost_Graph> skeleton_mst_graphPtr = graph.GetMST();


        // Prune the MST
        std::shared_ptr<Boost_Graph> skeleton_mst_pruned_graphPtr = graph.GetPrunedMST();
        tool::io::SaveGraphToPLY(folder_path + file_name + "_Skeleton_MST.ply", skeleton_mst_pruned_graphPtr);


        // Segment the skeleton
        std::vector<std::vector<Boost_Vertex>> classes = graph.SegmentSkeleton();


        // Retrieve the points
        std::vector<std::vector<int>> estimated_labels = tool::utility::RetrievePoints(classes, skeleton_mst_pruned_graphPtr, cloudPtr);
        std::ofstream result_file(folder_path + file_name + "_result.xyz");
        for (int i = 0; i < estimated_labels.size(); ++i) {
            for (const int &index : estimated_labels[i]) {
                result_file << index << " " << cloudPtr->points_[index][0] << " " << cloudPtr->points_[index][1] << " " << cloudPtr->points_[index][2] << " " << i << std::endl;
            }
        }
        result_file.close();
    }


    return EXIT_SUCCESS;
}
