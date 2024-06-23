#include "Tools.h"
#include "Skeleton.h"

#include <filesystem>


int main() {
    std::vector<std::string> file_names = {"C20-2"}; // "C20-2", "C20-4", "Ca46-1", "Ca46-5", "Ca11-2"
    for (const std::string &file_name: file_names) {
        std::string file_path = "../data/Skeleton/" + file_name + "/" + file_name + "_Downsampled.ply";
        std::string folder_path = "../data/Skeleton/" + file_name + "/" + "TuftedLaplacian-2" + "/";
        if (std::filesystem::exists(folder_path)) {
            std::filesystem::remove_all(folder_path);
        }
        std::filesystem::create_directory(folder_path);

        std::shared_ptr<Eigen::MatrixXd> cloudPtr = std::make_shared<Eigen::MatrixXd>();
        std::shared_ptr<Eigen::MatrixXd> _ = std::make_shared<Eigen::MatrixXd>();
        std::vector<std::pair<std::string, std::string>> __;
        tool::io::LoadPointCloudFromPLY(file_path, cloudPtr, _, __);

        // Laplacian-based contraction
        Skeleton skeleton(cloudPtr, 8, 0.95, 1.6, folder_path + file_name);
        std::shared_ptr<Eigen::MatrixXd> skeletonPtr = skeleton.GetLaplacianSkeleton();
        std::vector<std::pair<std::string, std::shared_ptr<Eigen::MatrixXd>>> cloud_viz = {{"Original", cloudPtr},
                                                                                           {"Skeleton", skeletonPtr}};
        tool::visualize::DrawPointClouds("Laplacian", cloud_viz);
    }
    return EXIT_SUCCESS;
}
