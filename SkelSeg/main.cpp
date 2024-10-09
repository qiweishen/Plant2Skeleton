#include "Tools.h"
#include "Skeleton.h"
#include "LOP.h"
#include "Graph.h"
#include "utility/logger/logger.h"
#include "utility/config_validator/config_validator.h"
#include "deps/nlohmann/json.hpp"

#include <filesystem>



void MainProcess(const std::filesystem::path &input_file_path, nlohmann::json &config) {
    std::filesystem::path output_folder_path = config["Output_Settings"]["Output_Folder_Path"].get<std::filesystem::path>();
    std::string file_name = input_file_path.filename().string();
    file_name = file_name.substr(0, file_name.find('.'));
    bool binary_format = config["Output_Settings"]["Output_File_DataFormat"].get<std::string>() == "Binary";
    Logger::Instance().AddLine(LogLine::STAR);
    Logger::Instance().Log(std::format("{} Start Processing!", file_name));
    Logger::Instance().AddLine(LogLine::STAR);


    // Load the point cloud
    std::shared_ptr<Eigen::MatrixXd> cloudPtr = std::make_shared<Eigen::MatrixXd>();
    tool::preprocess::PreparePointCloud(cloudPtr, input_file_path, config);


    // Laplacian-based contraction
    Skeleton skeleton(cloudPtr, config);
    std::shared_ptr<Eigen::MatrixXd> skeletonPtr = skeleton.GetLaplacianSkeleton();


    // Down sample the skeleton points using farthest point down sampling
    std::vector<size_t> selected_indices = tool::utility::FarthestPointDownSample(skeletonPtr,
                                                                                  config["Skeleton_Building"]["Down_Sample_Number"].get<int>());
    std::shared_ptr<Eigen::MatrixXd> skeleton_points_cloudPtr = std::make_shared<Eigen::MatrixXd>(
            tool::utility::SelectByIndices(skeletonPtr, selected_indices, 3));
    tool::io::SavePointCloudToPLY(skeleton_points_cloudPtr, output_folder_path / "_Downsample.ply",
                                  binary_format);


    // Utilize modified LOP operator to calibrate the skeleton points
    std::shared_ptr<Eigen::MatrixXd> final_skeleton_points_cloudPtr = LOPCalibrate(cloudPtr,
                                                                                   skeleton_points_cloudPtr, config);
    tool::io::SavePointCloudToPLY(final_skeleton_points_cloudPtr, output_folder_path / (file_name + "_LOP.ply"),
                                  binary_format);


    // Compute MST
    Graph graph(final_skeleton_points_cloudPtr, config);
    std::shared_ptr<Boost_Graph> skeleton_mst_graphPtr = graph.GetMST();
    tool::io::SaveSkeletonGraphToPLY(skeleton_mst_graphPtr, output_folder_path / "_MST-Raw.ply",
                                     binary_format);
    // Prune the MST
    std::shared_ptr<Boost_Graph> skeleton_mst_pruned_graphPtr = graph.GetPrunedMST();
    tool::io::SaveSkeletonGraphToPLY(skeleton_mst_pruned_graphPtr, output_folder_path / (file_name + "_MST-Pruned.ply"),
                                     binary_format);


    // Segment the skeleton
    std::vector<std::vector<Boost_Vertex>> classes = graph.SegmentSkeleton();
    std::vector<int> skeleton_segment_labels(skeleton_mst_pruned_graphPtr->m_vertices.size());
    for (int i = 0; i < classes.size(); ++i) {
        for (const Boost_Vertex &index: classes[i]) {
            skeleton_segment_labels.at(index) = i;
        }
    }
    tool::io::SaveSkeletonGraphToPLY(skeleton_mst_pruned_graphPtr,
                                     output_folder_path / (file_name + "_MST-Segmented.ply"),
                                     skeleton_segment_labels, binary_format);
    // Detect Stem-Leaf points based on the skeleton segmentation
    std::vector<std::vector<int>> estimated_labels = tool::utility::NearestProjectFromBoostVerts(classes,
                                                                                                 skeleton_mst_pruned_graphPtr,
                                                                                                 cloudPtr);


    // Write the result to the file
    std::vector<std::tuple<int, double, double, double, int>> data;
    for (int i = 0; i < estimated_labels.size(); ++i) {
        for (const int &index: estimated_labels[i]) {
            data.emplace_back(index, cloudPtr->row(index)[0], cloudPtr->row(index)[1], cloudPtr->row(index)[2], i);
        }
    }
    // Sort data by index
    std::sort(data.begin(), data.end(),
              [](const std::tuple<int, double, double, double, int> &a,
                 const std::tuple<int, double, double, double, int> &b) {
                  return std::get<0>(a) < std::get<0>(b);
              });
    // Write the sorted data to the file
    std::shared_ptr<Eigen::MatrixXd> resultPtr = std::make_shared<Eigen::MatrixXd>(cloudPtr->rows(), 3);
    std::vector<int> predicted_semantic_labels(cloudPtr->rows());
    std::vector<int> predicted_instance_labels(cloudPtr->rows());
    std::pair<std::string, std::string> label_names = {"pred_semantic", "pred_instance"};
    for (const std::tuple<int, double, double, double, int> &entry: data) {
        resultPtr->row(std::get<0>(entry)) << std::get<1>(entry), std::get<2>(entry), std::get<3>(entry);
        if (std::get<4>(entry) == 0.0) { // Shoot
            predicted_semantic_labels.at(std::get<0>(entry)) = 0; // Semantic label for shoot
            predicted_instance_labels.at(std::get<0>(entry)) = -1; // Instance label for shoot
        } else {
            predicted_semantic_labels.at(std::get<0>(entry)) = 1; // Semantic label for leaf
            predicted_instance_labels.at(std::get<0>(entry)) = static_cast<int>(std::get<4>(entry) -
                                                                                1.0); // Instance label for leaf
        }
    }
    tool::io::SavePointCloudToPLY(resultPtr, output_folder_path / (file_name + "_Result.ply"),
                                  predicted_semantic_labels,
                                  predicted_instance_labels, label_names, binary_format);


    tool::io::SaveJSONFile(config, output_folder_path / (file_name + "_Config.json"));


    Logger::Instance().Log("****************************************************************", 0, true, false);
    Logger::Instance().Log(std::format("{} Finish!", input_file_path.string()), 0, true, true);
    Logger::Instance().Log("****************************************************************", 0, true, false);
}


int main() {
    // Load the configuration file
    std::ifstream config_file("../configue.json");
    if (!config_file.is_open()) {
        std::cerr << "Failed to open the configuration file!" << std::endl;
        return EXIT_FAILURE;
    }
    nlohmann::json config;
    try {
        config_file >> config;
    } catch (const nlohmann::json::parse_error &e) {
        std::cerr << "Failed to parse the configuration file: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    if (!configvalidator::ValidateConfig(config)) {
        std::cerr << "Invalid configuration file!" << std::endl;
        return EXIT_FAILURE;
    } else {
        std::cout << "Configuration file loaded successfully!" << std::endl;
    }


    // Initialize the Logger::Instance()
    std::filesystem::path log_path =
            config["Output_Settings"]["Output_Folder_Path"].get<std::filesystem::path>() / "log.txt";
    Logger::Instance().SetLogFile(log_path);
    Logger::Instance().Log("SkelSeg program start!", LogLevel::INFO, true, true);


    // Check the processing mode
    if (config["Input_Settings"]["Batch_Processing"].get<bool>()) {
        Logger::Instance().AddLine(LogLine::DASH);
        Logger::Instance().Log("Batch Processing Mode!", 0, true, true);

        std::filesystem::path batch_folder_path = config["Input_Settings"]["Batch_Folder_Path"].get<std::filesystem::path>();
        std::string file_extension = config["Input_Settings"]["Point_Cloud_File_Extension"].get<std::string>();
        for (const auto &entry: std::filesystem::directory_iterator(batch_folder_path)) {
            if (entry.is_regular_file()) {
                if (entry.path().extension() == file_extension) {
                    Logger::Instance().Log(std::format("Processing file: {}", entry.path().string()), 0, true, false);
                    MainProcess(entry.path(), config);
                }
            }
        }
    } else {
        Logger::Instance().AddLine(LogLine::DASH);
        Logger::Instance().Log("Single Processing Mode!");

        std::filesystem::path file_path = config["Input_Settings"]["Point_Cloud_File_Path"].get<std::filesystem::path>();
        MainProcess(file_path, config);
    }

    return EXIT_SUCCESS;
}


//int main() {
//    std::filesystem::path input_file_path = "/Users/shenqiwei/Documents/Windows/Skeleton_Compare/Other-Skeletonlization/LBC/Ca46-4/Ca46-4_cpts_0.ply";
//    std::filesystem::path skeleton_path = "/Users/shenqiwei/Documents/Windows/Skeleton_Compare/Other-Skeletonlization/WoodSKE/Ca46-4-SIMUSKE.xyz";
//    Eigen::MatrixXd cloud = tool::io::LoadPointCloud(input_file_path);
//    Eigen::MatrixXd skeleton = tool::io::LoadPointCloud(skeleton_path);
//    std::shared_ptr<Eigen::MatrixXd> cloudPtr = std::make_shared<Eigen::MatrixXd>(cloud);
//    std::shared_ptr<Eigen::MatrixXd> skeletonPtr = std::make_shared<Eigen::MatrixXd>(skeleton);
//    double avg_distance = evaluate::DistanceToOriginalPointCloud(cloudPtr, skeletonPtr, "average");
//    double max_distance = evaluate::DistanceToOriginalPointCloud(cloudPtr, skeletonPtr, "max");
//    std::cout << std::format("Avg: {:.6f}", avg_distance) << std::endl;
//    std::cout << std::format("Max: {:.6f}", max_distance) << std::endl;
//}

