#include "Tools_pure.h"
#include "Skeleton_pure.h"

#include <filesystem>





int main() {
    std::vector<std::string> file_names = {"C20-2"}; // "C20-2", "C20-4", "Ca46-1", "Ca46-5", "Ca11-2"
    for (const std::string &file_name : file_names) {
        std::string file_path = "../../../main/data/Skeleton/" + file_name + "/" + file_name + "_Downsampled.ply";
        std::string folder_path = "/Users/shenqiwei/Documents/Windows/Skeleton_Compare/Laplacian-viz/";
        std::filesystem::create_directory(folder_path);


//        std::shared_ptr<PointCloud> raw_cloudPtr = std::make_shared<PointCloud>();
//        std::shared_ptr<Eigen::MatrixXd> raw_propertyPtr = std::make_shared<Eigen::MatrixXd>();
//        std::vector<std::pair<std::string, std::string>> raw_property_names = std::vector<std::pair<std::string, std::string>>();
//        tool::io::LoadPointCloudFromPLY(file_path, raw_cloudPtr, raw_propertyPtr, raw_property_names);
//        tool::preprocess::Normalize(raw_cloudPtr, 1.6);


        std::shared_ptr<PointCloud> cloudPtr = std::make_shared<PointCloud>();
        std::shared_ptr<Eigen::MatrixXd> _ = std::make_shared<Eigen::MatrixXd>();
        std::vector<std::pair<std::string, std::string>> __;
        tool::io::LoadPointCloudFromPLY(file_path, cloudPtr, _, __);
        tool::io::SavePointCloudToPLY(folder_path + file_name + "_cpts_0.ply", cloudPtr, _, __);


        // Laplacian-based contraction
        Skeleton skeleton(cloudPtr, 30, folder_path + file_name);
        std::shared_ptr<PointCloud> skeletonPtr = skeleton.GetLaplacianSkeleton();
//        tool::io::SavePointCloudToPLY(folder_path + file_name + "_Refined_Laplacian(PCA_085).ply", skeletonPtr, _, __);
    }


    return EXIT_SUCCESS;
}