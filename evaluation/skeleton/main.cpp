#include <iostream>
#include <string>

#include "evaluator.h"


void PrintUsage(const char* program_name) {
	std::cout << "Usage: " << program_name << " <point_cloud.ply> <skeleton.ply>\n"
			  << "\n"
			  << "Evaluate skeleton graph quality against the original point cloud.\n"
			  << "\n"
			  << "Arguments:\n"
			  << "  point_cloud.ply   Input point cloud PLY file\n"
			  << "  skeleton.ply      Skeleton graph PLY file (with vertex and edge elements)\n";
}


int main(int argc, char* argv[]) {
	if (argc != 3) {
		PrintUsage(argv[0]);
		return 1;
	}

	std::filesystem::path cloud_path(argv[1]);
	std::filesystem::path skeleton_path(argv[2]);

	if (!std::filesystem::exists(cloud_path)) {
		std::cerr << "Error: Point cloud file not found: " << cloud_path << "\n";
		return 1;
	}
	if (!std::filesystem::exists(skeleton_path)) {
		std::cerr << "Error: Skeleton file not found: " << skeleton_path << "\n";
		return 1;
	}

	Eigen::MatrixXd cloud = evaluate::utility::LoadPointCloudFromPLY(cloud_path);
	Boost_Graph skeleton = evaluate::utility::LoadGraphFromPLY(skeleton_path);

	std::cout << cloud_path.string() << std::endl;
	std::cout << skeleton_path.string() << std::endl;
	std::cout << "Point cloud: " << cloud.rows() << " points\n";
	std::cout << "Skeleton:    " << boost::num_vertices(skeleton) << " vertices, "
			  << boost::num_edges(skeleton) << " edges\n\n";

	auto metrics = evaluate::skeleton::QuantitativeSkeletonDistance(cloud, skeleton);

	std::cout << "── Forward (cloud → skeleton) ──\n"
			  << "  Mean distance:         " << metrics.forward_mean << "\n"
			  << "  90th percentile:       " << metrics.forward_90th << "\n"
			  << "── Reverse (skeleton → cloud) ──\n"
			  << "  Mean distance:         " << metrics.reverse_mean << "\n"
			  << "── Combined ──\n"
			  << "  Chamfer distance:      " << metrics.chamfer << "\n";

	return 0;
}
