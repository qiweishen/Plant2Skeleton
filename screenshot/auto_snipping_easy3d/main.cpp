#include <boost/mpl/vector.hpp>
#include <easy3d/util/initializer.h>
#include <filesystem>
#include <iostream>
#include <string>

#include "snipping.h"


int main(int argc, char **argv) {
	std::string skelseg_output_folder = "/workspace/data/output";
	std::string img_output_folder = "/workspace/data/image";
	std::vector<float> camera = { 2.5, 0, 0.08, 0.5, 0.5, 0.5, 0.5 };

	if (!std::filesystem::exists(img_output_folder)) {
		// Create the output folder
		std::filesystem::create_directories(img_output_folder);
	}

	std::vector<std::string> ply_input_folders;
	std::vector<std::string> sample_names;
	for (const auto &entry: std::filesystem::directory_iterator(skelseg_output_folder)) {
		if (entry.is_directory()) {
			ply_input_folders.push_back(entry.path().string());
			sample_names.push_back(entry.path().filename().string());
		}
	}

	snipping::LinesConfig lines_cfg;
	easy3d::vec3 lines_color = easy3d::vec3(255, 170, 127) / 255.0f;
	lines_cfg.uniform_color = easy3d::vec4(lines_color.x, lines_color.y, lines_color.z, 1.0f);

	snipping::CameraConfig camera_cfg;
	camera_cfg.position = easy3d::vec3(camera[0], camera[1], camera[2]);
	easy3d::quat q;
	q[0] = camera[3];
	q[1] = camera[4];
	q[2] = camera[5];
	q[3] = camera[6];
	camera_cfg.orientation = q;

	easy3d::initialize(false);

	// Build the offscreen renderer once and reuse it for every snapshot.
	// Repeatedly constructing/destroying OffScreen leaks driver-side GL resources
	// and eventually breaks VBO allocation (see vertex_array_object.cpp:113).
	const snipping::OutputConfig default_output;
	easy3d::OffScreen os(default_output.width, default_output.height);

	bool ok = false;

	// _result.png
	snipping::PointsConfig result_points_cfg;
	result_points_cfg.point_size = 3.0f;
	result_points_cfg.impostor = easy3d::PointsDrawable::PLAIN;
	result_points_cfg.coloring = easy3d::State::SCALAR_FIELD;
	result_points_cfg.scalar_location = easy3d::State::VERTEX;
	result_points_cfg.scalar_name = "v:pred-instance";
	result_points_cfg.colormap = "colormap";
	// Placeholder
	snipping::PointsConfig skeleton_points_cfg;
	for (int i = 0; i < ply_input_folders.size(); i++) {
		std::filesystem::path original_input_file = std::filesystem::path(ply_input_folders[i]) / (sample_names[i] + "_Result.ply");
		std::filesystem::copy_file(original_input_file, "/workspace/dataset/data/All_Parameters/images/ply" / original_input_file.filename(),
								   std::filesystem::copy_options::overwrite_existing);
		std::filesystem::path skeleton_input_file = "";
		std::filesystem::path output_file = img_output_folder + "/" + sample_names[i] + "_result.png";

		snipping::OutputConfig output;
		output.file_name = output_file;

		ok = snipping::snapshot(os, original_input_file, skeleton_input_file, output, result_points_cfg, skeleton_points_cfg, lines_cfg, camera_cfg);
		if (ok) {
			std::cout << "Snapshot saved to: " << output_file << std::endl;
		} else {
			std::cerr << "Failed to render snapshot." << std::endl;
		}
	}

	// _semantic_result.png
	snipping::PointsConfig semantic_result_cfg;
	semantic_result_cfg.point_size = 3.0f;
	semantic_result_cfg.impostor = easy3d::PointsDrawable::PLAIN;
	semantic_result_cfg.coloring = easy3d::State::SCALAR_FIELD;
	semantic_result_cfg.scalar_location = easy3d::State::VERTEX;
	semantic_result_cfg.scalar_name = "v:pred-semantic";
	semantic_result_cfg.colormap = "french";
	for (int i = 0; i < ply_input_folders.size(); i++) {
		std::filesystem::path original_input_file = std::filesystem::path(ply_input_folders[i]) / (sample_names[i] + "_Result.ply");
		std::filesystem::path skeleton_input_file = "";
		std::filesystem::path output_file = img_output_folder + "/" + sample_names[i] + "_semantic_result.png";

		snipping::OutputConfig output;
		output.file_name = output_file;

		ok = snipping::snapshot(os, original_input_file, skeleton_input_file, output, semantic_result_cfg, skeleton_points_cfg, lines_cfg,
								camera_cfg);
		if (ok) {
			std::cout << "Snapshot saved to: " << output_file << std::endl;
		} else {
			std::cerr << "Failed to render snapshot." << std::endl;
		}
	}

	// _semantic_skel.png
	snipping::PointsConfig original_points_cfg;
	original_points_cfg.visible = false;
	snipping::PointsConfig semantic_skeleton_points_cfg;
	semantic_skeleton_points_cfg.point_size = 8.0f;
	semantic_skeleton_points_cfg.impostor = easy3d::PointsDrawable::SPHERE;
	semantic_skeleton_points_cfg.coloring = easy3d::State::SCALAR_FIELD;
	semantic_skeleton_points_cfg.scalar_location = easy3d::State::VERTEX;
	semantic_skeleton_points_cfg.scalar_name = "v:semantic";
	semantic_skeleton_points_cfg.colormap = "french";
	for (int i = 0; i < ply_input_folders.size(); i++) {
		std::filesystem::path original_input_file = std::filesystem::path(ply_input_folders[i]) / "1_Input.ply";
		std::filesystem::path skeleton_input_file = std::filesystem::path(ply_input_folders[i]) / "7_MST-Segmented.ply";
		std::filesystem::path output_file = img_output_folder + "/" + sample_names[i] + "_semantic_skel.png";

		snipping::OutputConfig output;
		output.file_name = output_file;

		ok = snipping::snapshot(os, original_input_file, skeleton_input_file, output, original_points_cfg, semantic_skeleton_points_cfg, lines_cfg,
								camera_cfg);
		if (ok) {
			std::cout << "Snapshot saved to: " << output_file << std::endl;
		} else {
			std::cerr << "Failed to render snapshot." << std::endl;
		}
	}

	// _instance_skel.png
	snipping::PointsConfig instance_skeleton_points_cfg;
	instance_skeleton_points_cfg.point_size = 8.0f;
	instance_skeleton_points_cfg.impostor = easy3d::PointsDrawable::SPHERE;
	instance_skeleton_points_cfg.coloring = easy3d::State::SCALAR_FIELD;
	instance_skeleton_points_cfg.scalar_location = easy3d::State::VERTEX;
	instance_skeleton_points_cfg.scalar_name = "v:instance";
	instance_skeleton_points_cfg.colormap = "colormap";
	for (int i = 0; i < ply_input_folders.size(); i++) {
		std::filesystem::path original_input_file = std::filesystem::path(ply_input_folders[i]) / "1_Input.ply";
		std::filesystem::path skeleton_input_file = std::filesystem::path(ply_input_folders[i]) / "7_MST-Segmented.ply";
		std::filesystem::path output_file = img_output_folder + "/" + sample_names[i] + "_instance_skel.png";

		snipping::OutputConfig output;
		output.file_name = output_file;

		ok = snipping::snapshot(os, original_input_file, skeleton_input_file, output, original_points_cfg, instance_skeleton_points_cfg, lines_cfg,
								camera_cfg);
		if (ok) {
			std::cout << "Snapshot saved to: " << output_file << std::endl;
		} else {
			std::cerr << "Failed to render snapshot." << std::endl;
		}
	}


	// _skel.png
	snipping::PointsConfig original_result_cfg;
	original_result_cfg.point_size = 3.0f;
	original_result_cfg.impostor = easy3d::PointsDrawable::PLAIN;
	easy3d::vec3 original_color = easy3d::vec3(154, 153, 150) / 255.0f;
	original_result_cfg.uniform_color = easy3d::vec4(original_color.x, original_color.y, original_color.z, 1.0f);
	snipping::PointsConfig uniform_skeleton_points_cfg;
	uniform_skeleton_points_cfg.point_size = 8.0f;
	uniform_skeleton_points_cfg.impostor = easy3d::PointsDrawable::SPHERE;
	easy3d::vec3 uniform_skeleton_color = easy3d::vec3(192, 0, 1) / 255.0f;
	uniform_skeleton_points_cfg.uniform_color = easy3d::vec4(uniform_skeleton_color.x, uniform_skeleton_color.y, uniform_skeleton_color.z, 1.0f);
	for (int i = 0; i < ply_input_folders.size(); i++) {
		std::filesystem::path original_input_file = std::filesystem::path(ply_input_folders[i]) / "1_Input.ply";
		std::filesystem::path skeleton_input_file = std::filesystem::path(ply_input_folders[i]) / "7_MST-Segmented.ply";
		std::filesystem::path output_file = img_output_folder + "/" + sample_names[i] + "_skel.png";

		snipping::OutputConfig output;
		output.file_name = output_file;

		ok = snipping::snapshot(os, original_input_file, skeleton_input_file, output, original_result_cfg, uniform_skeleton_points_cfg, lines_cfg,
								camera_cfg);
		if (ok) {
			std::cout << "Snapshot saved to: " << output_file << std::endl;
		} else {
			std::cerr << "Failed to render snapshot." << std::endl;
		}
	}

	return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
