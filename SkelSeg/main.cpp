#include "Tools.h"
#include "Skeleton.h"
#include "LOP.h"
#include "Graph.h"
#include "utility/Random.h"

#include <filesystem>
#include <easy3d/core/point_cloud.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/algo/point_cloud_simplification.h>
#include <easy3d/kdtree/kdtree_search.h>
#include <easy3d/kdtree/kdtree_search_eth.h>


int main() {
    std::string directory_path = "/Users/shenqiwei/Documents/Windows/Skeleton_Compare/Test_Input/";
    // /Users/shenqiwei/Desktop/Plant2Skeleton/data/Dataset/Ours/
    // /Users/shenqiwei/Documents/Windows/Skeleton_Compare/Test_Input/
    // /Users/shenqiwei/Desktop/Plant2Skeleton/data/Dataset/Pheno4D/
    // /Users/shenqiwei/Documents/Windows/Skeleton_Compare/All_Input_10240/PLY/
    std::vector<std::string> ply_files;
    for (const auto &entry: std::filesystem::directory_iterator(directory_path)) {
        if (entry.is_regular_file() && entry.path().extension() == ".ply") {
            ply_files.push_back(entry.path().string());
        }
    }
    for (const std::string &ply_file: ply_files) {
        std::filesystem::path path_obj(ply_file);
        std::string file_name = path_obj.filename().string();
        file_name = file_name.substr(0, file_name.find("_Input.ply"));

//        if (file_name.find("C20-2") == std::string::npos) {
//            continue;
//        }

        std::string folder_path =
                "/Users/shenqiwei/Documents/Windows/Skeleton_Compare/Test_Input/XYZ/" + file_name +
                "/";
        if (std::filesystem::exists(folder_path)) {
            std::filesystem::remove_all(folder_path);
        }
        std::filesystem::create_directory(folder_path);

        std::cout << "**************************************************" << std::endl;
        std::cout << file_name << " Start!" << std::endl;
        std::cout << "--------------------------------------------------" << std::endl;




//        // Load the point cloud
//        easy3d::PointCloud *easy3d_cloud = easy3d::PointCloudIO::load(ply_file);
//        if (!easy3d_cloud) {
//            std::cerr << "failed to load model. Please make sure the file exists and format is correct." << std::endl;
//            return EXIT_FAILURE;
//        }
//        // Downsample the point cloud to 10,240 | 20,480 using uniformly downsampling
//        unsigned int num_points = 10240;
//        std::vector<easy3d::PointCloud::Vertex> indices_to_delete;
//        easy3d::KdTreeSearch *kdtree = new easy3d::KdTreeSearch_ETH(easy3d_cloud);
//        double initial_epsilon =
//                easy3d::PointCloudSimplification::average_space(easy3d_cloud, kdtree, 6, false, 10000) * 10.0;
//        while (!(num_points <= easy3d_cloud->n_vertices() - indices_to_delete.size() &&
//                 num_points + 256 >= easy3d_cloud->n_vertices() - indices_to_delete.size())) {
//            if (num_points < easy3d_cloud->n_vertices() - indices_to_delete.size()) {
//                initial_epsilon *= 1.1;
//            } else {
//                initial_epsilon *= 0.9;
//            }
//            indices_to_delete = easy3d::PointCloudSimplification::uniform_simplification(easy3d_cloud,
//                                                                                         float(initial_epsilon),
//                                                                                         kdtree);
//        }
//        // Remove the points
//        for (easy3d::PointCloud::Vertex &i: indices_to_delete) {
//            easy3d_cloud->delete_vertex(i);
//        }
//        // Remove deleted vertices
//        easy3d_cloud->collect_garbage();
//        indices_to_delete = easy3d::PointCloudSimplification::uniform_simplification(easy3d_cloud, num_points);
//        // Remove the points
//        for (easy3d::PointCloud::Vertex &i: indices_to_delete) {
//            easy3d_cloud->delete_vertex(i);
//        }
//        // Remove deleted vertices
//        easy3d_cloud->collect_garbage();
//        assert(easy3d_cloud->n_vertices() == easy3d_cloud->vertices_size() && easy3d_cloud->n_vertices() == num_points);
//        std::shared_ptr<Eigen::MatrixXd> cloudPtr = std::make_shared<Eigen::MatrixXd>(easy3d_cloud->n_vertices(), 3);
//        // We only need semantic_label, instance_label
//        std::shared_ptr<Eigen::MatrixXd> propertyPtr = std::make_shared<Eigen::MatrixXd>(easy3d_cloud->n_vertices(), 2);
//        easy3d::PointCloud::VertexProperty<easy3d::vec3> points = easy3d_cloud->get_vertex_property<easy3d::vec3>(
//                "v:point");
//        std::vector<std::string> aa = easy3d_cloud->vertex_properties();
//        easy3d::PointCloud::VertexProperty<int> semantic = easy3d_cloud->get_vertex_property<int>("v:semantic_label");
//        easy3d::PointCloud::VertexProperty<int> instance = easy3d_cloud->get_vertex_property<int>("v:instance_label");
//        for (easy3d::PointCloud::Vertex vert: easy3d_cloud->vertices()) {
//            cloudPtr->row(vert.idx()) = Eigen::Vector3d(points[vert].x, points[vert].y, points[vert].z);
//            propertyPtr->row(vert.idx()) = Eigen::Vector2d(semantic[vert], instance[vert]);
//        }
//        // Free the memory
//        delete easy3d_cloud;
//        delete kdtree;
//        // Normalize the point cloud
//        Eigen::Vector3d center;
//        double normalization_scaling;
//        std::pair(center, normalization_scaling) = tool::preprocess::Normalize(cloudPtr, 1.6);
//
//
//        // Save the input point cloud
//        std::vector<std::pair<std::string, std::string>> property_names = {{"int", "semantic_label"},
//                                                                           {"int", "instance_label"}};
//        tool::io::SavePointCloudToPLY("/Users/shenqiwei/Documents/Windows/Skeleton_Compare/Test_Input/" + file_name + "_Input.ply", cloudPtr, propertyPtr,
//                                      property_names);



//        int k_neighbors_ = 4;
//        while (k_neighbors_ <= 40) {
//            // Load the point cloud
//            std::shared_ptr<Eigen::MatrixXd> cloudPtr = std::make_shared<Eigen::MatrixXd>();
//            std::shared_ptr<Eigen::MatrixXd> _ = std::make_shared<Eigen::MatrixXd>();
//            std::vector<std::pair<std::string, std::string>> property_names;
//            tool::io::LoadPointCloudFromPLY(ply_file, cloudPtr, _, property_names);
//
//            std::shared_ptr<Eigen::SparseMatrix<double>> L_Ptr_ = std::make_shared<Eigen::SparseMatrix<double>>(
//                    cloudPtr->rows(), cloudPtr->rows());
//            L_Ptr_->setZero();
//            std::shared_ptr<geometrycentral::pointcloud::PointCloud> gc_cloudPtr = std::make_shared<geometrycentral::pointcloud::PointCloud>(
//                    cloudPtr->rows());
//            std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> gc_geom = std::make_shared<geometrycentral::pointcloud::PointPositionGeometry>(
//                    *gc_cloudPtr);
//            std::vector<geometrycentral::Vector3> vertices_positions = tool::utility::Matrix2GCVector(cloudPtr);
//            for (int i = 0; i < cloudPtr->rows(); ++i) {
//                gc_geom->positions[i] = vertices_positions[i];
//            }
//            gc_geom->kNeighborSize = k_neighbors_;
//            // Compute the Laplacian matrix
//            gc_geom->requireLaplacian();
//            *L_Ptr_ = -(2 * gc_geom->laplacian);
//            int totalNonZeroPositions = 0;
//            int numRows = L_Ptr_->rows();
//            // Iterate over each row
//            for (int row = 0; row < numRows; ++row) {
//                int nonZeroCount = 0;
//                // Iterate over non-zero elements in the row
//                for (Eigen::SparseMatrix<double>::InnerIterator it(*L_Ptr_, row); it; ++it) {
//                    ++nonZeroCount;
//                }
//                totalNonZeroPositions += nonZeroCount;
//            }
//            // Compute the mean number of non-zero positions per row
//            double meanNonZeroPositions = static_cast<double>(totalNonZeroPositions) / numRows;
//            std::cout << "i = " << k_neighbors_ << ": Mean number of non-zero positions per row: " << meanNonZeroPositions << std::endl;
//            k_neighbors_ += 1;
//        }


        // Load the point cloud
        std::shared_ptr<Eigen::MatrixXd> cloudPtr = std::make_shared<Eigen::MatrixXd>();
        std::shared_ptr<Eigen::MatrixXd> _ = std::make_shared<Eigen::MatrixXd>();
        std::vector<std::pair<std::string, std::string>> property_names;
        tool::io::LoadPointCloudFromPLY(ply_file, cloudPtr, _, property_names);


        // Laplacian-based contraction
        Skeleton skeleton(cloudPtr, 8, 0.90, 1.6, folder_path + file_name);
        std::shared_ptr<Eigen::MatrixXd> skeletonPtr = skeleton.GetLaplacianSkeleton();


////        // Visualize the skeleton points
////        std::vector<std::pair<std::string, std::shared_ptr<Eigen::MatrixXd>>> cloud_viz = {{"Original", cloudPtr},
////                                                                                           {"Skeleton", skeletonPtr}};
////        if (polyscope::isInitialized()) {
////            polyscope::shutdown(); // Clear previous visualization
////        }
////        tool::visualize::DrawPointClouds("Laplacian", cloud_viz);
//
//
//        // Downsample the skeleton points using farthest point downsampling
//        std::vector<size_t> selected_indices = tool::utility::FarthestPointDownSample(skeletonPtr, 1024);
//        std::shared_ptr<Eigen::MatrixXd> skeleton_points_cloudPtr = std::make_shared<Eigen::MatrixXd>(
//                tool::utility::SelectByIndices(skeletonPtr, selected_indices, 3));
//        tool::io::SavePointCloudToPLY(folder_path + file_name + "_Downsample.ply", skeleton_points_cloudPtr,
//                                      std::make_shared<Eigen::MatrixXd>(),
//                                      std::vector<std::pair<std::string, std::string>>());
//
//        // Utilize LOP to calibrate skeleton
//        std::shared_ptr<Eigen::MatrixXd> final_skeleton_points_cloudPtr = LOPIterate(cloudPtr,
//                                                                                     skeleton_points_cloudPtr);
//        tool::io::SavePointCloudToPLY(folder_path + file_name + "_LOP.ply", final_skeleton_points_cloudPtr,
//                                      std::make_shared<Eigen::MatrixXd>(),
//                                      std::vector<std::pair<std::string, std::string>>());
//
//        // Compute MST
//        Graph graph(final_skeleton_points_cloudPtr, 1.6);
//        std::shared_ptr<Boost_Graph> skeleton_mst_graphPtr = graph.GetMST();
//        tool::io::SaveGraphToPLY(folder_path + file_name + "_MST-Raw.ply", skeleton_mst_graphPtr);
//        // Prune the MST
//        std::shared_ptr<Boost_Graph> skeleton_mst_pruned_graphPtr = graph.GetPrunedMST();
//        tool::io::SaveGraphToPLY(folder_path + file_name + "_MST-Pruned.ply", skeleton_mst_pruned_graphPtr);
//
//
//        // Segment the skeleton
//        std::vector<std::vector<Boost_Vertex>> classes = graph.SegmentSkeleton();
//        // Retrieve the points
//        std::vector<std::vector<int>> estimated_labels = tool::utility::NearestProjectFromBoostVerts(classes,
//                                                                                                     skeleton_mst_pruned_graphPtr,
//                                                                                                     cloudPtr);
//
//        // Write the result to the file
//        std::vector<std::tuple<int, double, double, double, int>> data;
//        for (int i = 0; i < estimated_labels.size(); ++i) {
//            for (const int &index: estimated_labels[i]) {
//                data.emplace_back(index, cloudPtr->row(index)[0], cloudPtr->row(index)[1], cloudPtr->row(index)[2], i);
//            }
//        }
//        // Sort data by index
//        std::sort(data.begin(), data.end(),
//                  [](const std::tuple<int, double, double, double, int> &a,
//                     const std::tuple<int, double, double, double, int> &b) {
//                      return std::get<0>(a) < std::get<0>(b);
//                  });
//        // Write the sorted data to the file
//        std::shared_ptr<Eigen::MatrixXd> resultPtr = std::make_shared<Eigen::MatrixXd>(cloudPtr->rows(), 3);
//        std::shared_ptr<Eigen::MatrixXd> predicted_labelsPtr = std::make_shared<Eigen::MatrixXd>(cloudPtr->rows(), 2);
//        property_names = {{"int", "predicted_semantic_label"},
//                          {"int", "predicted_instance_label"}};
//        for (const std::tuple<int, double, double, double, int> &entry: data) {
//            resultPtr->row(std::get<0>(entry)) << std::get<1>(entry), std::get<2>(entry), std::get<3>(entry);
//            if (std::get<4>(entry) == 0.0) {
//                predicted_labelsPtr->row(std::get<0>(entry)) << 0.0, -1.0;
//            } else {
//                predicted_labelsPtr->row(std::get<0>(entry)) << 1.0, std::get<4>(entry) - 1.0;
//            }
//        }
//        tool::io::SavePointCloudToPLY(folder_path + file_name + "_Result.ply", resultPtr, predicted_labelsPtr,
//                                      property_names);




        std::cout << "--------------------------------------------------" << std::endl;
        std::cout << ply_file << " Finish!" << std::endl;
        std::cout << "**************************************************" << std::endl;
    }

    return EXIT_SUCCESS;
}
