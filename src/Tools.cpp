#include "Tools.h"

#include <memory>


namespace tool {
    namespace preprocess {
        template<>
        std::tuple<Eigen::Vector3d, double>
        Normalize(std::shared_ptr<Eigen::MatrixXd> &cloudPtr, double diagonal_length) {
            if (cloudPtr->rows() == 0 || cloudPtr->cols() != 3) {
                std::cerr << "Invalid Eigen::MatrixXd dimensions (" << cloudPtr->rows() << ", " << cloudPtr->cols()
                          << ")." << std::endl;
                std::exit(EXIT_FAILURE);
            }
            Eigen::Vector3d center = (cloudPtr->colwise().maxCoeff() + cloudPtr->colwise().minCoeff()) / 2.0;
            // Centre in zero
            for (int i = 0; i < cloudPtr->cols(); ++i) {
                cloudPtr->col(i) -= center;
            }
            // Calculate the scaling factor to make the maximum diagonal length equal to diagonal_length
            Eigen::Vector3d diagonal = cloudPtr->colwise().maxCoeff() - cloudPtr->colwise().minCoeff();
            double max_diagonal = diagonal.maxCoeff();
            double normalization_scaling = diagonal_length / max_diagonal;
            // Apply the scaling
            *cloudPtr *= normalization_scaling;
            return std::make_tuple(center, normalization_scaling);
        }

        template<>
        std::tuple<Eigen::Vector3d, double> Normalize(std::shared_ptr<PointCloud> &cloudPtr, double diagonal_length) {
            if (cloudPtr->points_.empty()) {
                std::cerr << "No point in PointCloud." << std::endl;
                std::exit(EXIT_FAILURE);
            }
            Eigen::Vector3d center = (cloudPtr->GetMaxBound() + cloudPtr->GetMinBound()) / 2;
            // Centre in zero
            for (Eigen::Vector3d &point: cloudPtr->points_) {
                point -= center;
            }
            // Calculate the scaling factor to make the maximum diagonal length equal to diagonal_length
            Eigen::Vector3d diagonal = cloudPtr->GetMaxBound() - cloudPtr->GetMinBound();
            double max_diagonal = diagonal.maxCoeff();
            double normalization_scaling = diagonal_length / max_diagonal;
            // Apply the scaling
            for (Eigen::Vector3d &point: cloudPtr->points_) {
                point *= normalization_scaling;
            }
            return std::make_tuple(center, normalization_scaling);
        }


        template<>
        void UnNormalize(std::shared_ptr<Eigen::MatrixXd> &cloudPtr, const Eigen::Vector3d &center,
                         const double &normalization_scaling) {
            for (int i = 0; i < cloudPtr->rows(); ++i) {
                Eigen::Vector3d point = cloudPtr->row(i);
                point /= normalization_scaling;
                point += center;
                cloudPtr->row(i) = point;
            }
        }

        template<>
        void UnNormalize(std::shared_ptr<PointCloud> &cloudPtr, const Eigen::Vector3d &center,
                         const double &normalization_scaling) {
            for (Eigen::Vector3d &i: cloudPtr->points_) {
                Eigen::Vector3d point = i;
                point /= normalization_scaling;
                point += center;
                i = point;
            }
        }
    }


    namespace io {
        template<>
        void LoadPointCloudFromPLY(const std::string &file_path, std::shared_ptr<Eigen::MatrixXd> &cloudPtr,
                                   std::shared_ptr<Eigen::MatrixXd> &propertyPtr,
                                   std::vector<std::pair<std::string, std::string>> &property_names) {
            property_names.clear();
            std::ifstream plyFile(file_path, std::ios::binary);
            if (!plyFile.is_open()) {
                std::cerr << "Failed to open file for reading." << std::endl;
                std::exit(EXIT_FAILURE);
            }
            std::string line;
            std::vector<std::string> vertex_types;
            int vertex_count = 0;
            bool is_binary = false;
            // Read header
            while (std::getline(plyFile, line)) {
                std::istringstream iss(line);
                std::string token;
                iss >> token;
                if (token == "format") {
                    std::string format;
                    iss >> format;
                    if (format == "binary_little_endian") {
                        is_binary = true;
                    } else if (format == "ascii") {
                        is_binary = false;
                    } else {
                        std::cerr << "Unsupported PLY format: " << format <<
                                  " Only binary little-endian and ascii format is supported." << std::endl;
                        std::exit(EXIT_FAILURE);
                    }
                } else if (token == "element") {
                    iss >> token;
                    if (token == "vertex") {
                        iss >> vertex_count;
                    }
                } else if (token == "property") {
                    std::string type, name;
                    iss >> type >> name;
                    if (name == "x" || name == "y" || name == "z") {
                        vertex_types.emplace_back(type);
                    }
                    if (name != "x" && name != "y" && name != "z") {
                        property_names.emplace_back(type, name);
                    }
                } else if (token == "end_header") {
                    break;
                }
            }
            if (vertex_types.size() != 3 || vertex_types[0] != vertex_types[1] || vertex_types[0] != vertex_types[2]) {
                std::cerr << "Invalid vertex types" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            if (vertex_count == 0) {
                std::cerr << "No vertices found" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            // Allocate memory for matrices
            cloudPtr = std::make_shared<Eigen::MatrixXd>(vertex_count, 3); // xyz
            propertyPtr = std::make_shared<Eigen::MatrixXd>(vertex_count, property_names.size()); // properties
            // Read vertex data
            std::string vertex_type = vertex_types[0];
            if (is_binary) {
                for (int i = 0; i < vertex_count; ++i) {
                    if (vertex_type == "float" || vertex_type == "float32") {
                        float x, y, z;
                        plyFile.read(reinterpret_cast<char *>(&x), sizeof(float));
                        plyFile.read(reinterpret_cast<char *>(&y), sizeof(float));
                        plyFile.read(reinterpret_cast<char *>(&z), sizeof(float));
                        cloudPtr->row(i) << x, y, z;
                    } else if (vertex_type == "double" || vertex_type == "float64") {
                        double x, y, z;
                        plyFile.read(reinterpret_cast<char *>(&x), sizeof(double));
                        plyFile.read(reinterpret_cast<char *>(&y), sizeof(double));
                        plyFile.read(reinterpret_cast<char *>(&z), sizeof(double));
                        cloudPtr->row(i) << x, y, z;
                    } else {
                        std::cerr << "Unsupported vertex type: " << vertex_type << std::endl;
                        std::exit(EXIT_FAILURE);
                    }
                    for (int j = 0; j < property_names.size(); ++j) {
                        std::string type = property_names[j].first;
                        if (type == "float" || type == "float32") {
                            float property;
                            plyFile.read(reinterpret_cast<char *>(&property), sizeof(float));
                            propertyPtr->row(i)(j) = property;
                        } else if (type == "double" || type == "float64") {
                            double property;
                            plyFile.read(reinterpret_cast<char *>(&property), sizeof(double));
                            propertyPtr->row(i)(j) = property;
                        } else if (type == "int" || type == "int32") {
                            int property;
                            plyFile.read(reinterpret_cast<char *>(&property), sizeof(int));
                            propertyPtr->row(i)(j) = property;
                        } else if (type == "uchar" || type == "uint8") {
                            unsigned char property;
                            plyFile.read(reinterpret_cast<char *>(&property), sizeof(unsigned char));
                            propertyPtr->row(i)(j) = property;
                        } else {
                            std::cerr << "Unsupported property type: " << type << std::endl;
                            std::exit(EXIT_FAILURE);
                        }
                    }
                }
            } else {
                for (int i = 0; i < vertex_count; ++i) {
                    double x, y, z; // double is used for better precision
                    plyFile >> x >> y >> z;
                    cloudPtr->row(i) << x, y, z;
                    for (int j = 0; j < property_names.size(); ++j) {
                        if (property_names[j].first == "float" || property_names[j].first == "float32") {
                            float property;
                            plyFile >> property;
                            propertyPtr->row(i)(j) = property;
                        } else if (property_names[j].first == "double" || property_names[j].first == "float64") {
                            double property;
                            plyFile >> property;
                            propertyPtr->row(i)(j) = property;
                        } else if (property_names[j].first == "int" || property_names[j].first == "int32") {
                            int property;
                            plyFile >> property;
                            propertyPtr->row(i)(j) = property;
                        } else if (property_names[j].first == "uchar" || property_names[j].first == "uint8") {
                            int property;
                            plyFile >> property;
                            propertyPtr->row(i)(j) = static_cast<unsigned char>(property);
                        } else {
                            std::cerr << "Unsupported property type: " << property_names[j].first << std::endl;
                            std::exit(EXIT_FAILURE);
                        }
                    }
                }
            }
            plyFile.close();
        }


        template<>
        void LoadPointCloudFromPLY(const std::string &file_path, std::shared_ptr<PointCloud> &cloudPtr,
                                   std::shared_ptr<Eigen::MatrixXd> &propertyPtr,
                                   std::vector<std::pair<std::string, std::string>> &property_names) {
            property_names.clear();
            std::ifstream plyFile(file_path, std::ios::binary);
            if (!plyFile.is_open()) {
                std::cerr << "Failed to open file for reading." << std::endl;
                std::exit(EXIT_FAILURE);
            }
            std::string line;
            std::vector<std::string> vertex_types;
            int vertex_count = 0;
            bool is_binary = false;
            // Read header
            while (std::getline(plyFile, line)) {
                std::istringstream iss(line);
                std::string token;
                iss >> token;
                if (token == "format") {
                    std::string format;
                    iss >> format;
                    if (format == "binary_little_endian") {
                        is_binary = true;
                    } else if (format == "ascii") {
                        is_binary = false;
                    } else {
                        std::cerr << "Unsupported PLY format: " << format <<
                                  " Only binary little-endian and ascii format is supported." << std::endl;
                        std::exit(EXIT_FAILURE);
                    }
                } else if (token == "element") {
                    iss >> token;
                    if (token == "vertex") {
                        iss >> vertex_count;
                    }
                } else if (token == "property") {
                    std::string type, name;
                    iss >> type >> name;
                    if (name == "x" || name == "y" || name == "z") {
                        vertex_types.emplace_back(type);
                    }
                    if (name != "x" && name != "y" && name != "z") {
                        property_names.emplace_back(type, name);
                    }
                } else if (token == "end_header") {
                    break;
                }
            }
            if (vertex_types.size() != 3 || vertex_types[0] != vertex_types[1] || vertex_types[0] != vertex_types[2]) {
                std::cerr << "Invalid vertex types" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            if (vertex_count == 0) {
                std::cerr << "No vertices found" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            // Allocate memory for matrices
            cloudPtr = std::make_shared<PointCloud>();
            cloudPtr->points_.resize(vertex_count); // xyz
            propertyPtr = std::make_shared<Eigen::MatrixXd>(vertex_count, property_names.size()); // properties
            // Read vertex data
            std::string vertex_type = vertex_types[0];
            if (is_binary) {
                for (int i = 0; i < vertex_count; ++i) {
                    if (vertex_type == "float" || vertex_type == "float32") {
                        float x, y, z;
                        plyFile.read(reinterpret_cast<char *>(&x), sizeof(float));
                        plyFile.read(reinterpret_cast<char *>(&y), sizeof(float));
                        plyFile.read(reinterpret_cast<char *>(&z), sizeof(float));
                        cloudPtr->points_[i] = Eigen::Vector3d(x, y, z);
                    } else if (vertex_type == "double" || vertex_type == "float64") {
                        double x, y, z;
                        plyFile.read(reinterpret_cast<char *>(&x), sizeof(double));
                        plyFile.read(reinterpret_cast<char *>(&y), sizeof(double));
                        plyFile.read(reinterpret_cast<char *>(&z), sizeof(double));
                        cloudPtr->points_[i] = Eigen::Vector3d(x, y, z);
                    } else {
                        std::cerr << "Unsupported vertex type: " << vertex_type << std::endl;
                        std::exit(EXIT_FAILURE);
                    }
                    for (int j = 0; j < property_names.size(); ++j) {
                        std::string type = property_names[j].first;
                        if (type == "float" || type == "float32") {
                            float property;
                            plyFile.read(reinterpret_cast<char *>(&property), sizeof(float));
                            propertyPtr->row(i)(j) = property;
                        } else if (type == "double" || type == "float64") {
                            double property;
                            plyFile.read(reinterpret_cast<char *>(&property), sizeof(double));
                            propertyPtr->row(i)(j) = property;
                        } else if (type == "int" || type == "int32") {
                            int property;
                            plyFile.read(reinterpret_cast<char *>(&property), sizeof(int));
                            propertyPtr->row(i)(j) = property;
                        } else if (type == "uchar" || type == "uint8") {
                            unsigned char property;
                            plyFile.read(reinterpret_cast<char *>(&property), sizeof(unsigned char));
                            propertyPtr->row(i)(j) = property;
                        } else {
                            std::cerr << "Unsupported property type: " << type << std::endl;
                            std::exit(EXIT_FAILURE);
                        }
                    }
                }
            } else {
                for (int i = 0; i < vertex_count; ++i) {
                    double x, y, z; // double is used for better precision
                    plyFile >> x >> y >> z;
                    cloudPtr->points_[i] = Eigen::Vector3d(x, y, z);
                    for (int j = 0; j < property_names.size(); ++j) {
                        if (property_names[j].first == "float" || property_names[j].first == "float32") {
                            float property;
                            plyFile >> property;
                            propertyPtr->row(i)(j) = property;
                        } else if (property_names[j].first == "double" || property_names[j].first == "float64") {
                            double property;
                            plyFile >> property;
                            propertyPtr->row(i)(j) = property;
                        } else if (property_names[j].first == "int" || property_names[j].first == "int32") {
                            int property;
                            plyFile >> property;
                            propertyPtr->row(i)(j) = property;
                        } else if (property_names[j].first == "uchar" || property_names[j].first == "uint8") {
                            int property;
                            plyFile >> property;
                            propertyPtr->row(i)(j) = static_cast<unsigned char>(property);
                        } else {
                            std::cerr << "Unsupported property type: " << property_names[j].first << std::endl;
                            std::exit(EXIT_FAILURE);
                        }
                    }
                }
            }
            plyFile.close();
        }


        template<>
        void SavePointCloudToPLY(const std::string &file_path,
                                 const std::shared_ptr<Eigen::MatrixXd> &cloudPtr,
                                 const std::shared_ptr<Eigen::MatrixXd> &propertyPtr,
                                 const std::vector<std::pair<std::string, std::string>> &property_names) {
            std::ofstream plyFile(file_path);
            if (!plyFile.is_open()) {
                std::cerr << "Failed to open file for writing." << std::endl;
                std::exit(EXIT_FAILURE);
            }
            size_t num_points = cloudPtr->rows();
            size_t num_properties = propertyPtr->cols();
            size_t num_property_names = property_names.size();
            assert(num_properties == num_property_names);
            if (num_points == 0) {
                std::cerr << "No points to write." << std::endl;
                std::exit(EXIT_FAILURE);
            }
            plyFile << std::fixed << std::setprecision(16);
            // Write PLY header
            plyFile << "ply\n";
            plyFile << "format ascii 1.0\n";
            plyFile << "comment object: PointCloud\n";
            plyFile << "element vertex " << num_points << "\n";
            plyFile << "property double x\n";
            plyFile << "property double y\n";
            plyFile << "property double z\n";
            if (num_properties == 0 && num_property_names == 0) {
                plyFile << "end_header\n";
                for (int i = 0; i < num_points; ++i) {
                    // Write vertex data
                    plyFile << (*cloudPtr)(i, 0) << " " << (*cloudPtr)(i, 1) << " " << (*cloudPtr)(i, 2) << "\n";
                }
            } else {
                assert(num_points == propertyPtr->rows());
                for (const std::pair<std::string, std::string> &property_name: property_names) {
                    std::string type = property_name.first;
                    std::string name = property_name.second;
                    plyFile << "property " << type << " " << name << "\n";
                }
                plyFile << "end_header\n";
                for (int i = 0; i < num_points; ++i) {
                    // Write vertex data
                    plyFile << (*cloudPtr)(i, 0) << " " << (*cloudPtr)(i, 1) << " " << (*cloudPtr)(i, 2);
                    for (int j = 0; j < propertyPtr->cols(); ++j) {
                        // Write property data
                        if (property_names[j].first == "int" || property_names[j].first == "uchar") {
                            plyFile << " " << (int) (*propertyPtr)(i, j);
                        } else {
                            plyFile << " " << (*propertyPtr)(i, j);
                        }
                    }
                    plyFile << "\n";
                }
            }
        }

        template<>
        void SavePointCloudToPLY(const std::string &file_path,
                                 const std::shared_ptr<PointCloud> &cloudPtr,
                                 const std::shared_ptr<Eigen::MatrixXd> &propertyPtr,
                                 const std::vector<std::pair<std::string, std::string>> &property_names) {
            std::ofstream plyFile(file_path);
            if (!plyFile.is_open()) {
                std::cerr << "Failed to open file for writing." << std::endl;
                std::exit(EXIT_FAILURE);
            }
            size_t num_points = cloudPtr->points_.size();
            size_t num_properties = propertyPtr->cols();
            size_t num_property_names = property_names.size();
            assert(num_properties == num_property_names);
            if (num_points == 0) {
                std::cerr << "No points to write." << std::endl;
                std::exit(EXIT_FAILURE);
            }
            plyFile << std::fixed << std::setprecision(16);
            // Write PLY header
            plyFile << "ply\n";
            plyFile << "format ascii 1.0\n";
            plyFile << "comment object: PointCloud\n";
            plyFile << "element vertex " << num_points << "\n";
            plyFile << "property double x\n";
            plyFile << "property double y\n";
            plyFile << "property double z\n";
            if (num_properties == 0 && num_property_names == 0) {
                plyFile << "end_header\n";
                for (const Eigen::Vector3d &point: cloudPtr->points_) {
                    // Write vertex data
                    plyFile << point[0] << " " << point[1] << " " << point[2] << "\n";
                }
            } else {
                assert(num_points == propertyPtr->rows());
                for (const std::pair<std::string, std::string> &property_name: property_names) {
                    std::string type = property_name.first;
                    std::string name = property_name.second;
                    plyFile << "property " << type << " " << name << "\n";
                }
                plyFile << "end_header\n";
                for (int i = 0; i < num_points; ++i) {
                    // Write vertex data
                    plyFile << cloudPtr->points_[i][0] << " " << cloudPtr->points_[i][1] << " "
                            << cloudPtr->points_[i][2];
                    for (int j = 0; j < propertyPtr->cols(); ++j) {
                        // Write property data
                        if (property_names[j].first == "int" || property_names[j].first == "uchar") {
                            plyFile << " " << (int) (*propertyPtr)(i, j);
                        } else {
                            plyFile << " " << (*propertyPtr)(i, j);
                        }
                    }
                    plyFile << "\n";
                }
            }
        }


        void SaveGraphToPLY(const std::string &file_path, const std::shared_ptr<Boost_Graph> &graphPtr) {
            std::ofstream plyFile(file_path);
            if (!plyFile.is_open()) {
                std::cerr << "Failed to open file for writing." << std::endl;
                std::exit(EXIT_FAILURE);
            }
            plyFile << std::fixed << std::setprecision(16);
            // Write PLY header
            plyFile << "ply\n";
            plyFile << "format ascii 1.0\n";
            plyFile << "comment object: Boost_Graph\n";
            plyFile << "element vertex " << num_vertices(*graphPtr) << "\n";
            plyFile << "property double x\n";
            plyFile << "property double y\n";
            plyFile << "property double z\n";
            plyFile << "element edge " << num_edges(*graphPtr) << "\n";
            plyFile << "property int vertex1\n";
            plyFile << "property int vertex2\n";
            plyFile << "end_header\n";
            // Write vertex data
            auto vertices = boost::vertices(*graphPtr);
            for (auto vi = vertices.first; vi != vertices.second; ++vi) {
                const Eigen::Vector3d &point = (*graphPtr)[*vi];
                plyFile << point[0] << " " << point[1] << " " << point[2] << "\n";
            }
            // Write edge data
            auto edges = boost::edges(*graphPtr);
            for (auto ei = edges.first; ei != edges.second; ++ei) {
                auto source = boost::source(*ei, *graphPtr);
                auto target = boost::target(*ei, *graphPtr);
                plyFile << source << " " << target << "\n";
            }
            plyFile.close();
        }
    }


    namespace utility {
        struct hash_eigen {
            std::size_t operator()(const Eigen::Vector3i &idx) const {
                std::size_t seed = 0;
                for (int i = 0; i < 3; ++i) {
                    seed ^= std::hash<int>()(idx[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                }
                return seed;
            }
        };

        class AccumulatedPoint {
            Eigen::Vector3d sum;
            int count;
        public:
            AccumulatedPoint() : sum(Eigen::Vector3d::Zero()), count(0) {}

            void AddPoint(const Eigen::Vector3d &point) {
                sum += point;
                ++count;
            }

            Eigen::Vector3d GetAveragePoint() const {
                if (count > 0) return sum / double(count);
                return Eigen::Vector3d::Zero(); // return zero vector if no points were added
            }
        };

        template<>
        // This part is adapted from Open3D,
        // https://github.com/isl-org/Open3D/blob/main/cpp/open3d/geometry/PointCloud.cpp#L354-L400
        std::shared_ptr<Eigen::MatrixXd>
        VoxelDownSample(std::shared_ptr<Eigen::MatrixXd> &cloudPtr, double voxel_size) {
            if (voxel_size <= 0.0) {
                std::cerr << "voxel_size <= 0.0" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            Eigen::Vector3d voxel_size3 = Eigen::Vector3d::Constant(voxel_size);
            Eigen::Vector3d voxel_max_bound = cloudPtr->colwise().maxCoeff().array() - voxel_size / 2.0;
            Eigen::Vector3d voxel_min_bound = cloudPtr->colwise().minCoeff().array() - voxel_size / 2.0;
            if (voxel_size * std::numeric_limits<int>::max() < (voxel_max_bound - voxel_min_bound).maxCoeff()) {
                std::cerr << "voxel_size is too small." << std::endl;
                std::exit(EXIT_FAILURE);
            }
            std::unordered_map<Eigen::Vector3i, AccumulatedPoint, hash_eigen> voxelindex_to_accpoint;
            Eigen::Vector3d ref_coord;
            Eigen::Vector3i voxel_index;
            for (int i = 0; i < cloudPtr->rows(); i++) {
                Eigen::Vector3d point = cloudPtr->row(i);
                ref_coord = (point - voxel_min_bound) / voxel_size;
                voxel_index << int(floor(ref_coord(0))), int(floor(ref_coord(1))), int(floor(ref_coord(2)));
                voxelindex_to_accpoint[voxel_index].AddPoint(point);
            }
            std::shared_ptr<Eigen::MatrixXd> output = std::make_shared<Eigen::MatrixXd>(voxelindex_to_accpoint.size(),
                                                                                        3);
            int idx = 0;
            for (const auto &accpoint: voxelindex_to_accpoint) {
                output->row(idx++) = accpoint.second.GetAveragePoint();
            }
            return output;
        }

        template<>
        std::shared_ptr<PointCloud> VoxelDownSample(std::shared_ptr<PointCloud> &cloudPtr, double voxel_size) {
            return cloudPtr->VoxelDownSample(voxel_size);
        }


        std::vector<int>
        FindIndices(std::shared_ptr<PointCloud> &cloudPtr, std::shared_ptr<PointCloud> &sampled_cloudPtr) {
            KDTreeFlann kdtree;
            kdtree.SetGeometry(*cloudPtr);
            std::vector<int> indices;
            for (const Eigen::Vector3d &point: sampled_cloudPtr->points_) {
                std::vector<int> index;
                std::vector<double> _;
                kdtree.SearchKNN(point, 1, index, _);
                indices.push_back(index[0]);
            }
            return indices;
        }


        template<>
        std::shared_ptr<PointCloud>
        SelectByIndices(const std::shared_ptr<PointCloud> &cloudPtr, const std::vector<int> &indices) {
            auto selected_cloudPtr = std::make_shared<PointCloud>();
            for (int index: indices) {
                selected_cloudPtr->points_.emplace_back(cloudPtr->points_[index]);
            }
            return selected_cloudPtr;
        }

        template<>
        std::shared_ptr<PointCloud>
        SelectByIndices(const std::shared_ptr<PointCloud> &cloudPtr, const std::vector<size_t> &indices) {
            return cloudPtr->SelectByIndex(indices);
        }


        std::vector<std::vector<int>> RetrievePoints(std::vector<std::vector<Boost_Vertex>> &classes,
                                                     std::shared_ptr<Boost_Graph> &graphPtr,
                                                     std::shared_ptr<PointCloud> &cloudPtr) {
            std::cout << "--------------------------------------------------" << std::endl;
            std::cout << "Start retrieve points" << std::endl;
            std::shared_ptr<PointCloud> skeleton_cloudPtr = std::make_shared<PointCloud>();
            for (int i = 0; i < graphPtr->m_vertices.size(); ++i) {
                skeleton_cloudPtr->points_.emplace_back((*graphPtr)[i]);
            }
            std::shared_ptr<KDTreeFlann> kdtree = std::make_shared<KDTreeFlann>();
            kdtree->SetGeometry(*skeleton_cloudPtr);

            std::vector<int> nearest_skeleton_index;
            nearest_skeleton_index.reserve(cloudPtr->points_.size());
            for (const Eigen::Vector3d &point: cloudPtr->points_) {
                std::vector<int> indices;
                std::vector<double> _;
                kdtree->SearchKNN(point, 1, indices, _);
                nearest_skeleton_index.emplace_back(indices[0]);
            }
            // Prepare a mapping from skeleton index to class index
            std::unordered_map<size_t, int> skeleton_to_class;
            for (int j = 0; j < classes.size(); ++j) {
                for (size_t idx: classes[j]) {
                    skeleton_to_class[idx] = j;
                }
            }
            // Assign points to their corresponding classes based on nearest skeleton index
            std::vector<std::vector<int>> result(classes.size());
            for (int i = 0; i < nearest_skeleton_index.size(); ++i) {
                int skeleton_idx = nearest_skeleton_index[i];
                auto class_itr = skeleton_to_class.find(skeleton_idx);
                if (class_itr != skeleton_to_class.end()) {
                    result[class_itr->second].emplace_back(i);
                }
            }
            return result;
        }
    }


    namespace debug {
        void LoadPointCloudFromXYZ(const std::string &file_path, std::shared_ptr<PointCloud> &cloudPtr) {
            if (file_path.empty()) {
                std::cerr << "Empty file path." << std::endl;
                std::exit(EXIT_FAILURE);
            }
            std::ifstream xyzFile(file_path);
            if (!xyzFile.is_open()) {
                std::cerr << "Failed to open file for writing." << std::endl;
                std::exit(EXIT_FAILURE);
            }
            std::vector<std::vector<double>> points;
            std::string line;
            while (std::getline(xyzFile, line)) {
                std::istringstream iss(line);
                std::vector<double> point(3);
                iss >> point[0] >> point[1] >> point[2];
                points.push_back(point);
            }
            xyzFile.close();
            cloudPtr = std::make_shared<PointCloud>();
            cloudPtr->points_.resize(points.size());
            for (int i = 0; i < points.size(); ++i) {
                cloudPtr->points_[i] = Eigen::VectorXd::Map(&points[i][0], 3);
            }
        }


        void SaveSphereToPLY(const Eigen::Vector3d &center, double radius, const std::string &filepath) {
            std::vector<Eigen::Vector3d> vertices;
            std::vector<std::vector<int>> faces;
            int sectorCount = 36;
            int stackCount = 18;
            double x, y, z, xy;                            // vertex position
            double sectorStep = 2 * M_PI / sectorCount;
            double stackStep = M_PI / stackCount;
            double sectorAngle, stackAngle;
            // Generate vertices
            for (int i = 0; i <= stackCount; ++i) {
                stackAngle = M_PI / 2 - i * stackStep;     // starting from pi/2 to -pi/2
                xy = radius * cos(stackAngle);             // r * cos(u)
                z = radius * sin(stackAngle);              // r * sin(u)
                for (int j = 0; j <= sectorCount; ++j) {
                    sectorAngle = j * sectorStep;          // starting from 0 to 2pi
                    x = xy * cos(sectorAngle);             // r * cos(u) * cos(v)
                    y = xy * sin(sectorAngle);             // r * cos(u) * sin(v)
                    vertices.emplace_back(center + Eigen::Vector3d(x, y, z));
                }
            }
            // Generate faces
            int k1, k2;
            for (int i = 0; i < stackCount; ++i) {
                k1 = i * (sectorCount + 1);
                k2 = k1 + sectorCount + 1;
                for (int j = 0; j < sectorCount; ++j, ++k1, ++k2) {
                    if (i != 0) {
                        faces.push_back({k1, k2, k1 + 1});
                    }
                    if (i != (stackCount - 1)) {
                        faces.push_back({k1 + 1, k2, k2 + 1});
                    }
                }
            }
            // Write to PLY
            std::ofstream plyFile(filepath);
            if (!plyFile.is_open()) {
                std::cerr << "Failed to open file for writing: " << filepath << std::endl;
                std::exit(EXIT_FAILURE);
            }
            plyFile << std::fixed << std::setprecision(16);
            // PLY header
            plyFile << "ply\n";
            plyFile << "format ascii 1.0\n";
            plyFile << "comment object: Sphere\n";
            plyFile << "element vertex " << vertices.size() << "\n";
            plyFile << "property double x\n";
            plyFile << "property double y\n";
            plyFile << "property double z\n";
            plyFile << "element face " << faces.size() << "\n";
            plyFile << "property list uchar int vertex_index\n";
            plyFile << "end_header\n";
            // Vertices
            for (const Eigen::Vector3d &vertex: vertices) {
                plyFile << vertex.x() << " " << vertex.y() << " " << vertex.z() << "\n";
            }
            // Faces
            for (const std::vector<int> &face: faces) {
                plyFile << face.size() << " ";
                for (size_t i = 0; i < face.size(); ++i) {
                    plyFile << face[i];
                    if (i < face.size() - 1) plyFile << " ";
                }
                plyFile << "\n";
            }
            plyFile.close();
        }
    }
}
