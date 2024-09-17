#include "Tools.h"

#include <iostream>
#include <filesystem>


//// Save the point cloud with properties in XYZ file to PLY file
//int main() {
//    std::string folder = "/Users/shenqiwei/Desktop/Plant2Skeleton/data/Dataset/Pheno4D/Tomato07/";
//    std::vector<std::string> files_names;
//    for (const auto &entry: std::filesystem::directory_iterator(folder)) {
//        std::string file_name = entry.path().filename().string();
//        if (file_name.find(".txt") != std::string::npos && file_name.find("T07_0309_a") != std::string::npos) {
//            file_name = file_name.substr(0, file_name.find(".txt"));
//            files_names.emplace_back(file_name);
//        }
//    }
//
////#pragma omp parallel for default(none) shared(folder, files_names)
//    for (int i = 0; i < int(files_names.size()); ++i) {
//        std::shared_ptr<Eigen::MatrixXd> cloudPtr = std::make_shared<Eigen::MatrixXd>();
//        std::shared_ptr<Eigen::MatrixXd> propertyPtr = std::make_shared<Eigen::MatrixXd>();
//        std::vector<std::pair<std::string, std::string>> property_names = {{"int",   "semantic_label"},
//                                                                           {"int",   "instance_label"}};
////        std::vector<std::pair<std::string, std::string>> property_names = {{"uchar", "R"},
////                                                                           {"uchar", "G"},
////                                                                           {"uchar", "B"},
////                                                                           {"int",   "semantic_label"},
////                                                                           {"int",   "instance_label"}};
//
//        std::string line;
//        std::ifstream txt_file(folder + files_names[i] + ".txt");
//        line.clear();
//
////        // Ignore the first line
////        std::getline(txt_file, line);
////        std::vector<std::string> properties_name(5);
////        std::istringstream iss(line);
////        iss >> properties_name[0] >> properties_name[1] >> properties_name[2] >> properties_name[3]
////            >> properties_name[4] >> properties_name[5]
////            >> properties_name[6] >> properties_name[7];
////        assert(properties_name[0] == "x" && properties_name[1] == "y" && properties_name[2] == "z" &&
////               properties_name[3] == "R" && properties_name[4] == "G" && properties_name[5] == "B" &&
////               properties_name[6] == "semantic_label" && properties_name[7] == "instance_label");
//
////        std::vector<std::vector<double>> temp_cloud;
////        std::vector<std::vector<double>> temp_properties;
////        while (std::getline(txt_file, line)) {
////            std::istringstream iss(line);
////            std::vector<double> point(3), properties(5);
////            iss >> point[0] >> point[1] >> point[2] >> properties[0] >> properties[1] >> properties[2] >> properties[3]
////                >> properties[4];
////            temp_cloud.push_back(point);
////            temp_properties.push_back(properties);
////        }
////        txt_file.close();
//
//        std::vector<std::vector<double>> temp_cloud;
//        std::vector<std::vector<double>> temp_properties;
//        while (std::getline(txt_file, line)) {
//            std::istringstream iss(line);
//            std::vector<double> point(3), properties(2), temp(1);
//            iss >> point[0] >> point[1] >> point[2] >> properties[0] >> temp[0];
//            if (properties[0] == 0) {
//                continue;
//            }
//            if (properties[0] == 1) { // stem
//                properties[0] = 0;
//                properties[1] = -1;
//            }
//            if (properties[0] != 1) { // leaf
//                double leaf_label = properties[0];
//                leaf_label = leaf_label - 1;
//                properties[0] = 1;
//                properties[1] = leaf_label;
//            }
//            temp_cloud.push_back(point);
//            temp_properties.push_back(properties);
//        }
//        txt_file.close();
//
//        // Allocate matrix space in one go
//        cloudPtr->resize(int(temp_cloud.size()), 3);
//        propertyPtr->resize(int(temp_properties.size()), 2);
//
//        for (size_t j = 0; j < temp_cloud.size(); ++j) {
//            cloudPtr->row(j) = Eigen::VectorXd::Map(&temp_cloud[j][0], 3);
//            propertyPtr->row(j) = Eigen::VectorXd::Map(&temp_properties[j][0], 2);
//        }
//
//        std::string out_file_name = files_names[i];
//        tool::io::SavePointCloudToPLY("/Users/shenqiwei/Desktop/Plant2Skeleton/data/Dataset/Pheno4D/" + out_file_name + ".ply", cloudPtr,
//                                      propertyPtr,
//                                      property_names);
//
////#pragma omp critical
//        {
//            std::cout << out_file_name << " is done." << std::endl;
//        }
//    }
//
//    return EXIT_SUCCESS;
//}


//// Merge the segmented plant point cloud with the original plant point cloud with pot
//int main() {
//    std::string folder = "/Users/shenqiwei/Documents/Windows/Check/Results/";
//    std::vector<std::string> files_names;
//    for (const auto &entry: std::filesystem::directory_iterator(folder)) {
//        std::string file_name = entry.path().filename().string();
//        if (file_name.find(".txt") != std::string::npos && file_name.find("C70-2") != std::string::npos) {
//            file_name = file_name.substr(0, file_name.find(".txt"));
//            files_names.emplace_back(file_name);
//        }
//    }
//
//    std::string reference_folder = "/Users/shenqiwei/Documents/Windows/BackgroundRemoved_PointClouds/";
//    std::vector<std::string> reference_files_names;
//    for (std::string &file_name: files_names) {
//        for (const auto &entry: std::filesystem::directory_iterator(reference_folder)) {
//            std::string reference_file_name = entry.path().filename().string();
//            if (reference_file_name.find(".txt") != std::string::npos &&
//                reference_file_name.find(file_name.substr(0, file_name.find("_"))) != std::string::npos) {
//                reference_file_name = reference_file_name.substr(0, reference_file_name.find(".txt"));
//                reference_files_names.emplace_back(reference_file_name);
//            } else {
//                continue;
//            }
//        }
//    }
//
//#pragma omp parallel for default(none) shared(reference_folder, reference_files_names, folder, files_names)
//    for (int i = 0; i < int(files_names.size()); ++i) {
//        assert(reference_files_names[i].substr(0, reference_files_names[i].find("_")) ==
//               files_names[i].substr(0, files_names[i].find("_")));
//
//        std::shared_ptr<Eigen::MatrixXd> cloudPtr = std::make_shared<Eigen::MatrixXd>();
//        std::shared_ptr<Eigen::MatrixXd> propertyPtr = std::make_shared<Eigen::MatrixXd>();
//        std::vector<std::pair<std::string, std::string>> property_names = {{"uchar", "R"},
//                                                                           {"uchar", "G"},
//                                                                           {"uchar", "B"},
//                                                                           {"int",   "semantic_label"},
//                                                                           {"int",   "instance_label"}};
//
//
//        std::ifstream reference_txt_file(reference_folder + reference_files_names[i] + ".txt");
//        std::vector<std::vector<double>> temp_cloud;
//        std::vector<std::vector<double>> temp_properties;
//        std::string line;
//        while (std::getline(reference_txt_file, line)) {
//            std::istringstream iss(line);
//            std::vector<double> point(3), properties(5);
//            iss >> point[0] >> point[1] >> point[2] >> properties[0] >> properties[1] >> properties[2];
//            properties[3] = -123;
//            properties[4] = -123;
//            temp_cloud.push_back(point);
//            temp_properties.push_back(properties);
//        }
//        reference_txt_file.close();
//
//        std::ifstream txt_file(folder + files_names[i] + ".txt");
//        line.clear();
//        // Ignore the first line
//        std::getline(txt_file, line);
//        std::vector<std::string> properties_name(5);
//        std::istringstream iss(line);
//        iss >> properties_name[0] >> properties_name[1] >> properties_name[2] >> properties_name[3]
//            >> properties_name[4] >> properties_name[5]
//            >> properties_name[6] >> properties_name[7];
//        assert(properties_name[0] == "x" && properties_name[1] == "y" && properties_name[2] == "z" &&
//               properties_name[3] == "R" && properties_name[4] == "G" && properties_name[5] == "B" &&
//               properties_name[6] == "semantic_label" && properties_name[7] == "instance_label");
//
//        while (std::getline(txt_file, line)) {
//            std::istringstream iss(line);
//            std::vector<double> point(3), properties(5);
//            iss >> point[0] >> point[1] >> point[2] >> properties[0] >> properties[1] >> properties[2] >> properties[3]
//                >> properties[4];
//            temp_cloud.push_back(point);
//            temp_properties.push_back(properties);
//        }
//        txt_file.close();
//
//        // Allocate matrix space in one go
//        cloudPtr->resize(int(temp_cloud.size()), 3);
//        propertyPtr->resize(int(temp_properties.size()), 5);
//
//        for (size_t j = 0; j < temp_cloud.size(); ++j) {
//            cloudPtr->row(j) = Eigen::VectorXd::Map(&temp_cloud[j][0], 3);
//            propertyPtr->row(j) = Eigen::VectorXd::Map(&temp_properties[j][0], 5);
//        }
//
//        std::string out_file_name = reference_files_names[i].substr(0, reference_files_names[i].find("_"));
//        tool::io::SavePointCloudToPLY(reference_folder + "Refer/" + out_file_name + "_Refer.ply", cloudPtr,
//                                      propertyPtr,
//                                      property_names);
//
//#pragma omp critical
//        {
//            std::cout << out_file_name << " is done." << std::endl;
//        }
//    }
//
//    return EXIT_SUCCESS;
//}


// Transfer to binary PLY file to XYZ file without properties
int main() {
    std::string input_folder = "/Users/shenqiwei/Documents/Windows/Skeleton_Compare/Test_Input/";
    std::string outpt_folder = "/Users/shenqiwei/Documents/Windows/Skeleton_Compare/Test_Input/PLY-ASCII/";
    std::vector<std::string> ply_files;
    for (const auto& entry : std::filesystem::directory_iterator(input_folder)) {
        if (entry.is_regular_file() && entry.path().extension() == ".ply") {
            ply_files.push_back(entry.path().string());
        }
    }
    std::vector<std::string> files_names;
    for (const std::string &ply_file: ply_files) {
        std::filesystem::path path_obj(ply_file);
        std::string file_name = path_obj.filename().string();
        if (file_name.find(".ply") != std::string::npos) {
            file_name = file_name.substr(0, file_name.find("_Input.ply"));
            files_names.emplace_back(file_name);
        }
    }

    for (int i = 0; i < int(files_names.size()); ++i) {
        std::shared_ptr<Eigen::MatrixXd> cloudPtr = std::make_shared<Eigen::MatrixXd>();
        std::shared_ptr<Eigen::MatrixXd> propertyPtr = std::make_shared<Eigen::MatrixXd>();
        std::vector<std::pair<std::string, std::string>> properties_name;
        tool::io::LoadPointCloudFromPLY(ply_files[i], cloudPtr, propertyPtr, properties_name);

//        std::ofstream txt_file(outpt_folder + files_names[i] + ".txt");
//        std::string line;
//        for (int i = 0; i < cloudPtr->rows(); ++i) {
//            line = std::to_string(cloudPtr->row(i)[0]) + " " + std::to_string(cloudPtr->row(i)[1]) + " " +
//                   std::to_string(cloudPtr->row(i)[2]) + "\n";
//            txt_file << line;

        std::shared_ptr<Eigen::MatrixXd> _ = std::make_shared<Eigen::MatrixXd>();
        std::vector<std::pair<std::string, std::string>> __;
        tool::io::SavePointCloudToPLY(outpt_folder + files_names[i] + ".ply", cloudPtr, _, __);


        std::cout << files_names[i] << " is done." << std::endl;

    }
}
