#include <filesystem>
#include <nlohmann/json.hpp>
#include <utility/config_validator.h>
#include <utility/evaluator.h>
#include <utility/logger.h>

#include "graph.h"
#include "lop.h"
#include "skeleton.h"
#include "tools.h"



void MainProcess(const std::filesystem::path &input_file_path, nlohmann::json &config) {
	std::filesystem::path output_folder_path = config["Output_Settings"]["Output_Folder_Path"].get<std::filesystem::path>();
	std::string file_name = input_file_path.filename().string();
	file_name = file_name.substr(0, file_name.find('.'));
	Logger::Instance().AddLine(LogLine::DASH);
	Logger::Instance().Log(std::format("{} Start Processing!", file_name));


	// Load the point cloud
	Eigen::MatrixXd input_cloud;
	tool::preprocess::PreparePointCloud(input_cloud, config);


	// Laplacian-based contraction
	Skeleton skeleton(input_cloud, config);
	Eigen::MatrixXd laplacian_skeleton = skeleton.GetLaplacianSkeleton();


	// Down sample the skeleton using the farthest point downsampling
	const auto downsample_number =
			static_cast<size_t>(std::round(config["Skeleton_Building"]["Down_Sample_Ratio"].get<double>() * laplacian_skeleton.rows()));
	Eigen::MatrixXd skeleton_points_cloud;
	std::vector<size_t> _;
	std::tie(skeleton_points_cloud, _) = tool::utility::FarthestPointDownSample(laplacian_skeleton, downsample_number);
	tool::io::SavePointCloudToPLY(skeleton_points_cloud, output_folder_path / "2_FPS-Downsampled.ply");


	// Utilize modified LOP operator to calibrate the skeleton points
	Eigen::MatrixXd final_skeleton_points_cloud = LOPCalibrate(input_cloud, skeleton_points_cloud, config);
	tool::io::SavePointCloudToPLY(final_skeleton_points_cloud, output_folder_path / "3_LOP-Calibrated.ply");


	// Compute and prune MST
	Graph graph(final_skeleton_points_cloud, config);
	Boost_Graph skeleton_initial_graph = graph.GetInitialGraph();
	tool::io::SaveSkeletonGraphToPLY(skeleton_initial_graph, output_folder_path / "4_Initial-Graph.ply");
	Boost_Graph skeleton_mst_graph = graph.GetMST();
	tool::io::SaveSkeletonGraphToPLY(skeleton_mst_graph, output_folder_path / "5_MST-Raw.ply");
	Boost_Graph skeleton_mst_pruned_graph = graph.GetPrunedMST();
	tool::io::SaveSkeletonGraphToPLY(skeleton_mst_pruned_graph, output_folder_path / "6_MST-Pruned.ply");


	// Segment the skeleton
	std::vector<int> skeleton_semantic_labels;
	std::vector<int> skeleton_instance_labels;
	std::tie(skeleton_semantic_labels, skeleton_instance_labels) = graph.SegmentSkeleton();
	tool::io::SaveSkeletonGraphToPLY(skeleton_mst_pruned_graph, output_folder_path / "7_MST-Segmented.ply", skeleton_semantic_labels,
									 skeleton_instance_labels);


	// Detect Stem-Leaf points based on the skeleton segmentation
	std::vector<int> predicted_instance_labels =
			tool::utility::NearestProjectFromBoostVertices(skeleton_mst_pruned_graph, input_cloud, skeleton_instance_labels, "vertex");


	// Write the final PLY file
	std::vector<int> predicted_semantic_labels(input_cloud.rows());
	std::pair<std::string, std::string> label_names = { "pred-semantic", "pred-instance" };
	for (int i = 0; i < predicted_instance_labels.size(); ++i) {
		if (predicted_instance_labels.at(i) == -1) {  // Shoot
			predicted_semantic_labels.at(i) = 0;	  // Semantic label for shoot
		} else {
			predicted_semantic_labels.at(i) = 1;	  // Semantic label for leaf
		}
	}
	tool::io::SavePointCloudToPLY(input_cloud, output_folder_path / (file_name + "_Result.ply"), predicted_semantic_labels, predicted_instance_labels,
								  label_names);


	tool::io::SaveJSONFile(config, output_folder_path / "configure.json");


	Logger::Instance().AddLine(LogLine::DASH);
	Logger::Instance().Log(std::format("{} Finished Processing!", file_name));
}


int main() {
	// Load the configuration file
	std::filesystem::path config_file_path = "../configure.json";
	std::ifstream config_file(config_file_path);
	if (!config_file.is_open()) {
		throw std::runtime_error(std::format("Failed to open the configuration file: {}", config_file_path.string()));
	}
	nlohmann::json config;
	try {
		config_file >> config;
	} catch (const nlohmann::json::parse_error &e) {
		throw std::runtime_error(std::format("Failed to parse the configuration file: {}", e.what()));
	}
	configvalidator::ValidateConfig(config);


	// Initialize the Logger::Instance()
	std::filesystem::path log_path = config["Output_Settings"]["Output_Folder_Path"].get<std::filesystem::path>() / ".log";
	Logger::Instance().SetLogFile(log_path);
	Logger::Instance().PrintTitle();
	Logger::Instance().Log("SkelSeg program start!");


	// Check the processing mode
	if (config["Input_Settings"]["Batch_Processing"].get<bool>()) {
		Logger::Instance().Log("Batch Processing Mode!");

		std::filesystem::path batch_folder_path = config["Input_Settings"]["Batch_Folder_Path"].get<std::filesystem::path>();
		std::string file_extension = config["Input_Settings"]["Point_Cloud_File_Extension"].get<std::string>();
		for (const auto &entry: std::filesystem::directory_iterator(batch_folder_path)) {
			if (entry.is_regular_file()) {
				if (entry.path().extension() == file_extension) {
					MainProcess(entry.path(), config);
				}
			}
		}
	} else {
		Logger::Instance().Log("Single Processing Mode!");

		std::filesystem::path file_path = config["Input_Settings"]["Point_Cloud_File_Path"].get<std::filesystem::path>();
		MainProcess(file_path, config);
	}

	return EXIT_SUCCESS;
}
