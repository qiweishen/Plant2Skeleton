#ifndef CONFIG_VALIDATOR_H
#define CONFIG_VALIDATOR_H


#include <algorithm>
#include <cctype>
#include <filesystem>
#include <format>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>



namespace configvalidator {
	bool ValidateConfig(const nlohmann::json &config);


	std::optional<std::string> ValidateInputSettings(const nlohmann::json &input_settings);


	std::optional<std::string> ValidateLabelsNames(const nlohmann::json &labels_names, std::string_view extension);


	std::optional<std::string> ValidatePLYFormat(const nlohmann::json &ply_format);


	std::optional<std::string> ValidateTXTXYZFormat(const nlohmann::json &txt_xyz_format);


	std::optional<std::string> ValidatePreprocess(const nlohmann::json &preprocess);


	std::optional<std::string> ValidateConstraintLaplacianOperator(const nlohmann::json &constraint_laplacian_operator);


	std::optional<std::string> ValidateAdaptiveContraction(const nlohmann::json &adaptive_contraction);


	std::optional<std::string> ValidateTerminateCondition(const nlohmann::json &terminate_condition);


	std::optional<std::string> ValidateSkeletonBuilding(const nlohmann::json &skeleton_building);


	std::optional<std::string> ValidateOutputSettings(const nlohmann::json &output_settings);


	std::string ToLower(std::string str) {
		std::ranges::transform(str, str.begin(), [](unsigned char c) { return std::tolower(c); });
		return str;
	}


	template<typename ValidatorFunc>
	bool CheckSection(const nlohmann::json &config, const std::string &section_name, ValidatorFunc validator) {
		if (config.contains(section_name) && config[section_name].is_object()) {
			if (auto error = validator(config[section_name])) {
				throw std::runtime_error(error.value());
			}
			return true;
		} else {
			throw std::runtime_error(std::format("Missing or invalid '{}' section.", section_name));
		}
	}


	bool ValidateConfig(const nlohmann::json &config) {
		bool is_valid = true;

		is_valid &= CheckSection(config, "Input_Settings", ValidateInputSettings);
		is_valid &= CheckSection(config, "Preprocess", ValidatePreprocess);
		is_valid &= CheckSection(config, "Constraint_Laplacian_Operator", ValidateConstraintLaplacianOperator);
		is_valid &= CheckSection(config, "Adaptive_Contraction", ValidateAdaptiveContraction);
		is_valid &= CheckSection(config, "Terminate_Condition", ValidateTerminateCondition);
		is_valid &= CheckSection(config, "Skeleton_Building", ValidateSkeletonBuilding);
		is_valid &= CheckSection(config, "Output_Settings", ValidateOutputSettings);

		return is_valid;
	}


	// Validate "Input_Settings" section
	std::optional<std::string> ValidateInputSettings(const nlohmann::json &input_settings) {
		// Check "Batch_Processing"
		if (!input_settings.contains("Batch_Processing") || !input_settings["Batch_Processing"].is_boolean()) {
			return "Missing or invalid 'Batch_Processing' in 'Input_Settings'.";
		}

		bool batch_processing = input_settings["Batch_Processing"].get<bool>();
		if (batch_processing) {
			// Check "Batch_Processing_Folder_Path"
			if (!input_settings.contains("Batch_Processing_Folder_Path") || !input_settings["Batch_Processing_Folder_Path"].is_string()) {
				return "Missing or invalid 'Batch_Processing_Folder_Path' in 'Input_Settings'.";
			}

			std::filesystem::path folder_path = input_settings["Batch_Processing_Folder_Path"].get<std::filesystem::path>();
			try {
				if (!std::filesystem::is_directory(folder_path)) {
					return std::format("Invalid 'Batch_Processing_Folder_Path': '{}' is not a directory.", folder_path.string());
				}
			} catch (const std::filesystem::filesystem_error &e) {
				return std::format("Filesystem error: {}", e.what());
			}

			// Check "Point_Cloud_File_Extension"
			if (!input_settings.contains("Point_Cloud_File_Extension") || !input_settings["Point_Cloud_File_Extension"].is_string()) {
				return "Missing or invalid 'Point_Cloud_File_Extension' in 'Input_Settings'.";
			}
			std::string file_extension = ToLower(input_settings["Point_Cloud_File_Extension"].get<std::string>());
			if (file_extension != ".ply" && file_extension != ".xyz" && file_extension != ".txt") {
				return std::format(
						"Invalid 'Point_Cloud_File_Extension' in 'Input_Settings'. '{}' is not supported. Supported formats: .ply, .xyz, .txt",
						file_extension);
			}
		}

		// Check "Point_Cloud_File_Path"
		if (!batch_processing) {
			if (!input_settings.contains("Point_Cloud_File_Path") || !input_settings["Point_Cloud_File_Path"].is_string()) {
				return "Missing or invalid 'Point_Cloud_File_Path' in 'Input_Settings'.";
			}

			std::filesystem::path file_path = input_settings["Point_Cloud_File_Path"].get<std::string>();
			try {
				if (!std::filesystem::exists(file_path)) {
					return std::format("Invalid 'Point_Cloud_File_Path': '{}' does not exist.", file_path.string());
				}
			} catch (const std::filesystem::filesystem_error &e) {
				return std::format("Filesystem error: {}", e.what());
			}
		}

		// Check "With_Labels"
		if (!input_settings.contains("With_Labels") || !input_settings["With_Labels"].is_boolean()) {
			return "Missing or invalid 'With_Labels' in 'Input_Settings'.";
		}

		if (input_settings["With_Labels"].get<bool>()) {
			std::string file_extension;
			if (batch_processing) {
				file_extension = ToLower(input_settings["Point_Cloud_File_Extension"].get<std::string>());
			} else {
				file_extension = ToLower(input_settings["Point_Cloud_File_Path"].get<std::filesystem::path>().extension());
			}
			// Check "Labels_Names"
			if (input_settings.contains("Labels_Names") && input_settings["Labels_Names"].is_object()) {
				if (auto error = ValidateLabelsNames(input_settings["Labels_Names"], file_extension)) {
					return error;
				}
			} else {
				return "Missing or invalid 'Labels_Names' in 'Input_Settings'.";
			}
		}

		return std::nullopt;
	}


	// Validate "Labels_Names" section
	std::optional<std::string> ValidateLabelsNames(const nlohmann::json &labels_names, std::string_view extension) {
		if (extension == ".ply") {
			// Check "PLY_Format"
			if (labels_names.contains("PLY_Format") && labels_names["PLY_Format"].is_object()) {
				return ValidatePLYFormat(labels_names["PLY_Format"]);
			} else {
				return "Missing or invalid 'PLY_Format' in 'Labels_Names'.";
			}
		} else if (extension == ".xyz" || extension == ".txt") {
			// Check "TXT_XYZ_Format"
			if (labels_names.contains("TXT_XYZ_Format") && labels_names["TXT_XYZ_Format"].is_object()) {
				return ValidateTXTXYZFormat(labels_names["TXT_XYZ_Format"]);
			} else {
				return "Missing or invalid 'TXT_XYZ_Format' in 'Labels_Names'.";
			}
		}
		return std::nullopt;
	}


	// Validate "PLY_Format" section
	std::optional<std::string> ValidatePLYFormat(const nlohmann::json &ply_format) {
		if (!ply_format.contains("Individual_Labels_File") || !ply_format["Individual_Labels_File"].is_boolean()) {
			return "Missing or invalid 'Individual_Labels_File' in 'PLY_Format'.";
		}

		bool individualLabelsFile = ply_format["Individual_Labels_File"].get<bool>();

		if (individualLabelsFile) {
			if (!ply_format.contains("Labels_File_Paths") || !ply_format["Labels_File_Paths"].is_object()) {
				return "Missing or invalid 'Labels_File_Paths' in 'PLY_Format'.";
			} else {
				for (const auto &[key, value]: ply_format["Labels_File_Paths"].items()) {
					auto path = value.get<std::string>();
					try {
						if (!std::filesystem::exists(path)) {
							return std::format("Invalid label file path '{}' in 'Labels_File_Paths'.", path);
						}
					} catch (const std::filesystem::filesystem_error &e) {
						return std::format("Filesystem error: {}", e.what());
					}
				}
			}
		} else {
			if (!ply_format.contains("Semantic_Label_Name") || !ply_format["Semantic_Label_Name"].is_string()) {
				return "Missing or invalid 'Semantic_Label_Name' in 'PLY_Format'.";
			}
			if (!ply_format.contains("Instance_Label_Name") || !ply_format["Instance_Label_Name"].is_string()) {
				return "Missing or invalid 'Instance_Label_Name' in 'PLY_Format'.";
			}
		}

		return std::nullopt;
	}


	// Validate "TXT_XYZ_Format" section
	std::optional<std::string> ValidateTXTXYZFormat(const nlohmann::json &txt_xyz_format) {
		if (!txt_xyz_format.contains("Individual_Labels_File") || !txt_xyz_format["Individual_Labels_File"].is_boolean()) {
			return "Missing or invalid 'Individual_Labels_File' in 'TXT_XYZ_Format'.";
		}

		bool individualLabelsFile = txt_xyz_format["Individual_Labels_File"].get<bool>();

		if (individualLabelsFile) {
			if (!txt_xyz_format.contains("Labels_File_Paths") || !txt_xyz_format["Labels_File_Paths"].is_object()) {
				return "Missing or invalid 'Labels_File_Paths' in 'TXT_XYZ_Format'.";
			} else {
				for (const auto &[key, value]: txt_xyz_format["Labels_File_Paths"].items()) {
					auto path = value.get<std::string>();
					try {
						if (!std::filesystem::exists(path)) {
							return std::format("Invalid label file path '{}' in 'Labels_File_Paths'.", path);
						}
					} catch (const std::filesystem::filesystem_error &e) {
						return std::format("Filesystem error: {}", e.what());
					}
				}
			}
		} else {
			if (!txt_xyz_format.contains("Total_Columns") || !txt_xyz_format["Total_Columns"].is_number_integer()) {
				return "Missing or invalid 'Total_Columns' in 'TXT_XYZ_Format'.";
			}

			int totalColumns = txt_xyz_format["Total_Columns"].get<int>();

			if (!txt_xyz_format.contains("Semantic_Labels_Index") || !txt_xyz_format["Semantic_Labels_Index"].is_number_integer() ||
				txt_xyz_format["Semantic_Labels_Index"].get<int>() > totalColumns) {
				return "Missing or invalid 'Semantic_Labels_Index' in 'TXT_XYZ_Format'.";
			}

			if (!txt_xyz_format.contains("Instance_Labels_Index") || !txt_xyz_format["Instance_Labels_Index"].is_number_integer() ||
				txt_xyz_format["Instance_Labels_Index"].get<int>() > totalColumns) {
				return "Missing or invalid 'Instance_Labels_Index' in 'TXT_XYZ_Format'.";
			}
		}

		return std::nullopt;
	}


	// Validate "Preprocess" section
	std::optional<std::string> ValidatePreprocess(const nlohmann::json &preprocess) {
		if (!preprocess.contains("Down_Sample_Number") || !preprocess["Down_Sample_Number"].is_number_integer()) {
			return "Missing or invalid 'Down_Sample_Number' in 'Preprocess'.";
		}

		if (!preprocess.contains("Normalize_Diagonal_Length") || !preprocess["Normalize_Diagonal_Length"].is_number()) {
			return "Missing or invalid 'Normalize_Diagonal_Length' in 'Preprocess'.";
		}

		return std::nullopt;
	}


	// Validate "Constraint_Laplacian_Operator" section
	std::optional<std::string> ValidateConstraintLaplacianOperator(const nlohmann::json &constraint_laplacian_operator) {
		if (!constraint_laplacian_operator.contains("Use_KNN_Search") || !constraint_laplacian_operator.contains("Use_Radius_Search")) {
			return "Missing 'Use_KNN_Search' or 'Use_Radius_Search' in 'Constraint_Laplacian_Operator'.";
		}

		if (!constraint_laplacian_operator["Use_KNN_Search"].is_boolean() || !constraint_laplacian_operator["Use_Radius_Search"].is_boolean()) {
			return "Invalid 'Use_KNN_Search' or 'Use_Radius_Search' in 'Constraint_Laplacian_Operator'.";
		}

		if (constraint_laplacian_operator["Use_KNN_Search"].get<bool>() == constraint_laplacian_operator["Use_Radius_Search"].get<bool>()) {
			return "Only one of 'Use_KNN_Search' or 'Use_Radius_Search' should be true in 'Constraint_Laplacian_Operator'.";
		}

		if (!constraint_laplacian_operator.contains("Initial_k") || !constraint_laplacian_operator["Initial_k"].is_number_integer()) {
			return "Missing or invalid 'Initial_k' in 'Constraint_Laplacian_Operator'.";
		}

		if (!constraint_laplacian_operator.contains("Delta_k") || !constraint_laplacian_operator["Delta_k"].is_number_integer()) {
			return "Missing or invalid 'Delta_k' in 'Constraint_Laplacian_Operator'.";
		}

		if (!constraint_laplacian_operator.contains("Max_k") || !constraint_laplacian_operator["Max_k"].is_number_integer()) {
			return "Missing or invalid 'Max_k' in 'Constraint_Laplacian_Operator'.";
		}

		if (!constraint_laplacian_operator.contains("Initial_Radius_Search_Ratio") || !constraint_laplacian_operator["Initial_Radius_Search_Ratio"].is_number()) {
			return "Missing or invalid 'Initial_Radius_Search_Ratio' in 'Constraint_Laplacian_Operator'.";
		}

		if (!constraint_laplacian_operator.contains("Delta_Radius_Search_Ratio") || !constraint_laplacian_operator["Delta_Radius_Search_Ratio"].is_number()) {
			return "Missing or invalid 'Delta_Radius_Search_Ratio' in 'Constraint_Laplacian_Operator'.";
		}

		if (!constraint_laplacian_operator.contains("Min_Radius_Search_Ratio") || !constraint_laplacian_operator["Min_Radius_Search_Ratio"].is_number()) {
			return "Missing or invalid 'Min_Radius_Search_Ratio' in 'Constraint_Laplacian_Operator'.";
		}

		return std::nullopt;
	}


	// Validate "Adaptive_Contraction" section
	std::optional<std::string> ValidateAdaptiveContraction(const nlohmann::json &adaptive_contraction) {
		if (!adaptive_contraction.contains("Smooth_Sigma_Threshold") || !adaptive_contraction["Smooth_Sigma_Threshold"].is_number()) {
			return "Missing or invalid 'Smooth_Sigma_Threshold' in 'Adaptive_Contraction'.";
		}

		if (!adaptive_contraction.contains("Sigma_Sphere_Radius_Ratio") || !adaptive_contraction["Sigma_Sphere_Radius_Ratio"].is_number()) {
			return "Missing or invalid 'Sigma_Sphere_Radius_Ratio' in 'Adaptive_Contraction'.";
		}

		if (!adaptive_contraction.contains("Max_Distance_Ratio") || !adaptive_contraction["Max_Distance_Ratio"].is_number()) {
			return "Missing or invalid 'Max_Distance_Ratio' in 'Adaptive_Contraction'.";
		}

		return std::nullopt;
	}


	// Validate "Terminate_Condition" section
	std::optional<std::string> ValidateTerminateCondition(const nlohmann::json &terminate_condition) {
		if (!terminate_condition.contains("Max_Iteration") || !terminate_condition["Max_Iteration"].is_number_integer()) {
			return "Missing or invalid 'Max_Iteration' in 'Terminate_Condition'.";
		}

		if (!terminate_condition.contains("Convergence_Threshold") || !terminate_condition["Convergence_Threshold"].is_number()) {
			return "Missing or invalid 'Convergence_Threshold' in 'Terminate_Condition'.";
		}

		return std::nullopt;
	}


	// Validate "Skeleton_Building" section
	std::optional<std::string> ValidateSkeletonBuilding(const nlohmann::json &skeleton_building) {
		if (!skeleton_building.contains("Down_Sample_Ratio") || !skeleton_building["Down_Sample_Ratio"].is_number()) {
			return "Missing or invalid 'Down_Sample_Ratio' in 'Skeleton_Building'.";
		}

		if (!skeleton_building.contains("LOP_Sphere_Radius_Ratio") || !skeleton_building["LOP_Sphere_Radius_Ratio"].is_number()) {
			return "Missing or invalid 'LOP_Sphere_Radius_Ratio' in 'Skeleton_Building'.";
		}

		if (!skeleton_building.contains("Noise_Branch_Length_Ratio") || !skeleton_building["Noise_Branch_Length_Ratio"].is_number()) {
			return "Missing or invalid 'Noise_Branch_Length_Ratio' in 'Skeleton_Building'.";
		}

		return std::nullopt;
	}


	// Validate "Output_Settings" section
	std::optional<std::string> ValidateOutputSettings(const nlohmann::json &output_settings) {
		if (!output_settings.contains("Output_Folder_Path") || !output_settings["Output_Folder_Path"].is_string()) {
			return "Missing or invalid 'Output_Folder_Path' in 'Output_Settings'.";
		}

		std::filesystem::path output_folder_path = output_settings["Output_Folder_Path"].get<std::filesystem::path>();
		try {
			if (!std::filesystem::exists(output_folder_path)) {
				// Create the output folder
				std::filesystem::create_directories(output_folder_path);
				std::filesystem::create_directories(output_folder_path / ".iterations");
			} else {
				// Check if the output folder contains subdirectories
				bool has_contents = !std::filesystem::is_empty(output_folder_path);
				if (has_contents) {
					std::cout << "Output folder should be empty." << std::endl;
					std::string input;
					constexpr int max_attempts = 3;
					for (int attempt = 0; attempt < max_attempts; ++attempt) {
						std::cout << "Enter 'y' to terminate the program, or 'n' to continue and delete all contents: [y]/n" << std::endl;
						std::getline(std::cin, input);
						if (input == "y" || input == "Y" || input.empty()) {
							std::exit(EXIT_FAILURE);
						} else if (input == "n" || input == "N") {
							// Delete all contents in the output folder
							std::filesystem::remove_all(output_folder_path);
							std::filesystem::create_directories(output_folder_path);
							std::filesystem::create_directories(output_folder_path / ".iterations");
							break;
						} else {
							std::cout << "Invalid input. Please enter 'y' or 'n'." << std::endl;
						}
					}
					if (input != "n" && input != "N") {
						std::cout << "Invalid input. Program terminated." << std::endl;
						std::exit(EXIT_FAILURE);
					}
				}
			}
		} catch (const std::filesystem::filesystem_error &e) {
			return std::format("Filesystem error: {}", e.what());
		}

		if (!output_settings.contains("Output_PLY_File_DataFormat") || !output_settings["Output_PLY_File_DataFormat"].is_string()) {
			return "Missing or invalid 'Output_PLY_File_DataFormat' in 'Output_Settings'.";
		}

		std::string data_format = output_settings["Output_PLY_File_DataFormat"].get<std::string>();
		if (data_format != "Binary" && data_format != "ASCII") {
			return "Invalid 'Output_PLY_File_DataFormat' in 'Output_Settings'. Supported formats: Binary, ASCII.";
		}

		return std::nullopt;
	}
}  // namespace configvalidator



#endif	// CONFIG_VALIDATOR_H
