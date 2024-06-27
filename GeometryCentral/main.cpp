#include "Tools.h"
#include "Skeleton.h"

#include <filesystem>


int main() {
    std::vector<std::string> file_names = {"C20-2"}; // "C20-2", "C20-4", "Ca46-1", "Ca46-5", "Ca11-2"
    for (const std::string &file_name: file_names) {
        std::string file_path = "../data/Skeleton/" + file_name + "/" + file_name + "_Downsampled.ply";
//        std::string file_path = "../data/Skeleton/C20-2/Current_one-ring_local/C20-2_cpts_1.ply";
        std::string folder_path = "../data/Skeleton/" + file_name + "/" + "TuftedMeshLaplacian-1_8_fix" + "/";
        if (std::filesystem::exists(folder_path)) {
            std::filesystem::remove_all(folder_path);
        }
        std::filesystem::create_directory(folder_path);

        std::shared_ptr<Eigen::MatrixXd> cloudPtr = std::make_shared<Eigen::MatrixXd>();
        std::shared_ptr<Eigen::MatrixXd> _ = std::make_shared<Eigen::MatrixXd>();
        std::vector<std::pair<std::string, std::string>> __;
        tool::io::LoadPointCloudFromPLY(file_path, cloudPtr, _, __);

//        tool::visualize::DrawTangentPoints("Tangent_8", cloudPtr, 8, 9323);
//        tool::visualize::DrawTangentPoints("Tangent_30", cloudPtr, 30, 9323);

        // Laplacian-based contraction
        Skeleton skeleton(cloudPtr, 8, 0.90, 1.6, folder_path + file_name);
        std::shared_ptr<Eigen::MatrixXd> skeletonPtr = skeleton.GetLaplacianSkeleton();

        // Visualize
//        std::vector<std::pair<std::string, std::shared_ptr<Eigen::MatrixXd>>> cloud_viz = {{"Original", cloudPtr},
//                                                                                           {"Skeleton", skeletonPtr}};
//        if (polyscope::isInitialized()) {
//            polyscope::shutdown(); // Clear previous visualization
//        }
//        tool::visualize::DrawPointClouds("Laplacian", cloud_viz);
    }
    return EXIT_SUCCESS;
}
