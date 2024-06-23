#include "Tools.h"

#include <filesystem>

#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/core/point_cloud.h>





int main() {
    std::string folder = "/Users/shenqiwei/Documents/Check/Results/";
    std::vector<std::string> files_names;
    for (const auto &entry : std::filesystem::directory_iterator(folder)) {
        std::string file_name = entry.path().filename().string();
        if (file_name.find(".txt") != std::string::npos) {
            file_name = file_name.substr(0, file_name.find(".txt"));
            files_names.emplace_back(file_name);
        }
    }

    #pragma omp parallel for default(none) shared(files_names)
    for (std::string &file_name : files_names) {
        std::shared_ptr<Eigen::MatrixXd> cloudPtr = std::make_shared<Eigen::MatrixXd>();
        std::shared_ptr<Eigen::MatrixXd> propertyPtr = std::make_shared<Eigen::MatrixXd>();
        std::vector<std::pair<std::string, std::string>> property_names = {{"uchar", "R"}, {"uchar", "G"},
                                                                           {"uchar", "B"}, {"int", "label"}};

        std::ifstream txt_file("/Users/shenqiwei/Documents/Check/Results/" + file_name + ".txt");
        std::vector<std::vector<double>> temp_cloud;
        std::vector<std::vector<double>> temp_properties;

        std::string line;
        while (std::getline(txt_file, line)) {
            std::istringstream iss(line);
            std::vector<double> point(3), properties(4);
            iss >> point[0] >> point[1] >> point[2] >> properties[0] >> properties[1] >> properties[2] >> properties[3];
            temp_cloud.push_back(point);
            temp_properties.push_back(properties);
        }
        txt_file.close();

        // Allocate matrix space in one go
        cloudPtr->resize(int(temp_cloud.size()), 3);
        propertyPtr->resize(int(temp_properties.size()), 4);

        for (size_t j = 0; j < temp_cloud.size(); ++j) {
            cloudPtr->row(j) = Eigen::VectorXd::Map(&temp_cloud[j][0], 3);
            propertyPtr->row(j) = Eigen::VectorXd::Map(&temp_properties[j][0], 4);
        }

        file_name = file_name.substr(0, file_name.find("_"));
        tool::io::SavePointCloudToPLY("../data/labelled_binary/" + file_name + ".ply", cloudPtr, propertyPtr, property_names);

        #pragma omp critical
        std::cout << file_name << " is done." << std::endl;
    }


    return EXIT_SUCCESS;
}