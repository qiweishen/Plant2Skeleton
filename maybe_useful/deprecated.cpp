//        void LoadPointCloudFromPLY(const std::string &file_path, std::shared_ptr<Eigen::MatrixXd> &cloudPtr,
//                                   std::shared_ptr<Eigen::MatrixXd> &propertyPtr,
//                                   std::vector<std::pair<std::string, std::string>> &property_names) {
//            property_names.clear();
//            std::ifstream plyFile(file_path, std::ios::binary);
//            if (!plyFile.is_open()) {
//                std::cerr << "Failed to open file for reading." << std::endl;
//                std::exit(EXIT_FAILURE);
//            }
//            std::string line;
//            std::vector<std::string> vertex_types;
//            int vertex_count = 0;
//            bool is_binary = false;
//            // Read header
//            while (std::getline(plyFile, line)) {
//                std::istringstream iss(line);
//                std::string token;
//                iss >> token;
//                if (token == "format") {
//                    std::string format;
//                    iss >> format;
//                    if (format == "binary_little_endian") {
//                        is_binary = true;
//                    } else if (format == "ascii") {
//                        is_binary = false;
//                    } else {
//                        std::cerr << "Unsupported Binary format: " << format <<
//                                  " Only binary little-endian and ascii format is supported." << std::endl;
//                        std::exit(EXIT_FAILURE);
//                    }
//                } else if (token == "element") {
//                    iss >> token;
//                    if (token == "vertex") {
//                        iss >> vertex_count;
//                    }
//                } else if (token == "property") {
//                    std::string type, name;
//                    iss >> type >> name;
//                    if (name == "x" || name == "y" || name == "z") {
//                        vertex_types.emplace_back(type);
//                    }
//                    if (name != "x" && name != "y" && name != "z") {
//                        property_names.emplace_back(type, name);
//                    }
//                } else if (token == "end_header") {
//                    break;
//                }
//            }
//            if (vertex_types.size() != 3 || vertex_types[0] != vertex_types[1] || vertex_types[0] != vertex_types[2]) {
//                std::cerr << "Invalid vertex types" << std::endl;
//                std::exit(EXIT_FAILURE);
//            }
//            if (vertex_count == 0) {
//                std::cerr << "No vertices found" << std::endl;
//                std::exit(EXIT_FAILURE);
//            }
//            // Allocate memory for matrices
//            cloudPtr = std::make_shared<Eigen::MatrixXd>(vertex_count, 3); // xyz
//            propertyPtr = std::make_shared<Eigen::MatrixXd>(vertex_count, property_names.size()); // properties
//            // Read vertex data
//            std::string vertex_type = vertex_types[0];
//            if (is_binary) {
//                for (int i = 0; i < vertex_count; ++i) {
//                    if (vertex_type == "float" || vertex_type == "float32") {
//                        float x, y, z;
//                        plyFile.read(reinterpret_cast<char *>(&x), sizeof(float));
//                        plyFile.read(reinterpret_cast<char *>(&y), sizeof(float));
//                        plyFile.read(reinterpret_cast<char *>(&z), sizeof(float));
//                        cloudPtr->row(i) << x, y, z;
//                    } else if (vertex_type == "double" || vertex_type == "float64") {
//                        double x, y, z;
//                        plyFile.read(reinterpret_cast<char *>(&x), sizeof(double));
//                        plyFile.read(reinterpret_cast<char *>(&y), sizeof(double));
//                        plyFile.read(reinterpret_cast<char *>(&z), sizeof(double));
//                        cloudPtr->row(i) << x, y, z;
//                    } else {
//                        std::cerr << "Unsupported vertex type: " << vertex_type << std::endl;
//                        std::exit(EXIT_FAILURE);
//                    }
//                    for (int j = 0; j < property_names.size(); ++j) {
//                        std::string type = property_names[j].first;
//                        if (type == "float" || type == "float32") {
//                            float property;
//                            plyFile.read(reinterpret_cast<char *>(&property), sizeof(float));
//                            propertyPtr->row(i)(j) = property;
//                        } else if (type == "double" || type == "float64") {
//                            double property;
//                            plyFile.read(reinterpret_cast<char *>(&property), sizeof(double));
//                            propertyPtr->row(i)(j) = property;
//                        } else if (type == "int" || type == "int32") {
//                            int property;
//                            plyFile.read(reinterpret_cast<char *>(&property), sizeof(int));
//                            propertyPtr->row(i)(j) = property;
//                        } else if (type == "uchar" || type == "uint8") {
//                            unsigned char property;
//                            plyFile.read(reinterpret_cast<char *>(&property), sizeof(unsigned char));
//                            propertyPtr->row(i)(j) = property;
//                        } else {
//                            std::cerr << "Unsupported property type: " << type << std::endl;
//                            std::exit(EXIT_FAILURE);
//                        }
//                    }
//                }
//            } else {
//                for (int i = 0; i < vertex_count; ++i) {
//                    double x, y, z; // double is used for better precision
//                    plyFile >> x >> y >> z;
//                    cloudPtr->row(i) << x, y, z;
//                    for (int j = 0; j < property_names.size(); ++j) {
//                        if (property_names[j].first == "float" || property_names[j].first == "float32") {
//                            float property;
//                            plyFile >> property;
//                            propertyPtr->row(i)(j) = property;
//                        } else if (property_names[j].first == "double" || property_names[j].first == "float64") {
//                            double property;
//                            plyFile >> property;
//                            propertyPtr->row(i)(j) = property;
//                        } else if (property_names[j].first == "int" || property_names[j].first == "int32") {
//                            int property;
//                            plyFile >> property;
//                            propertyPtr->row(i)(j) = property;
//                        } else if (property_names[j].first == "uchar" || property_names[j].first == "uint8") {
//                            int property;
//                            plyFile >> property;
//                            propertyPtr->row(i)(j) = static_cast<unsigned char>(property);
//                        } else {
//                            std::cerr << "Unsupported property type: " << property_names[j].first << std::endl;
//                            std::exit(EXIT_FAILURE);
//                        }
//                    }
//                }
//            }
//            plyFile.close();
//        }
//
//
//        void SavePointCloudToPLY(const std::string &file_path,
//                                 const std::shared_ptr<Eigen::MatrixXd> &cloudPtr,
//                                 const std::shared_ptr<Eigen::MatrixXd> &propertyPtr,
//                                 const std::vector<std::pair<std::string, std::string>> &property_names) {
//            std::ofstream plyFile(file_path);
//            if (!plyFile.is_open()) {
//                std::cerr << "Failed to open file for writing." << std::endl;
//                std::exit(EXIT_FAILURE);
//            }
//            size_t num_points = cloudPtr->rows();
//            size_t num_properties = propertyPtr->cols();
//            size_t num_property_names = property_names.size();
//            assert(num_properties == num_property_names);
//            if (num_points == 0) {
//                std::cerr << "No points to write." << std::endl;
//                std::exit(EXIT_FAILURE);
//            }
//            plyFile << std::fixed << std::setprecision(16);
//            // Write Binary header
//            plyFile << "ply\n";
//            plyFile << "format ascii 1.0\n";
//            plyFile << "comment object: o3d_PointCloud\n";
//            plyFile << "element vertex " << num_points << "\n";
//            plyFile << "property double x\n";
//            plyFile << "property double y\n";
//            plyFile << "property double z\n";
//            if (num_properties == 0 && num_property_names == 0) {
//                plyFile << "end_header\n";
//                for (int i = 0; i < num_points; ++i) {
//                    // Write vertex data
//                    plyFile << (*cloudPtr)(i, 0) << " " << (*cloudPtr)(i, 1) << " " << (*cloudPtr)(i, 2) << "\n";
//                }
//            } else {
//                assert(num_points == propertyPtr->rows());
//                for (const std::pair<std::string, std::string> &property_name: property_names) {
//                    std::string type = property_name.first;
//                    std::string name = property_name.second;
//                    plyFile << "property " << type << " " << name << "\n";
//                }
//                plyFile << "end_header\n";
//                for (int i = 0; i < num_points; ++i) {
//                    // Write vertex data
//                    plyFile << (*cloudPtr)(i, 0) << " " << (*cloudPtr)(i, 1) << " " << (*cloudPtr)(i, 2);
//                    for (int j = 0; j < propertyPtr->cols(); ++j) {
//                        // Write property data
//                        if (property_names[j].first == "int" || property_names[j].first == "uchar") {
//                            plyFile << " " << (int) (*propertyPtr)(i, j);
//                        } else {
//                            plyFile << " " << (*propertyPtr)(i, j);
//                        }
//                    }
//                    plyFile << "\n";
//                }
//            }
//        }
//
//
//
//
//
//        void SaveGraphToPLY(const std::string &file_path, const std::shared_ptr<Boost_Graph> &graphPtr) {
//            std::ofstream plyFile(file_path);
//            if (!plyFile.is_open()) {
//                std::cerr << "Failed to open file for writing." << std::endl;
//                std::exit(EXIT_FAILURE);
//            }
//            plyFile << std::fixed << std::setprecision(16);
//            // Write Binary header
//            plyFile << "ply\n";
//            plyFile << "format ascii 1.0\n";
//            plyFile << "comment object: Boost_Graph\n";
//            plyFile << "element vertex " << num_vertices(*graphPtr) << "\n";
//            plyFile << "property double x\n";
//            plyFile << "property double y\n";
//            plyFile << "property double z\n";
//            plyFile << "element edge " << num_edges(*graphPtr) << "\n";
//            plyFile << "property int vertex1\n";
//            plyFile << "property int vertex2\n";
//            plyFile << "end_header\n";
//            // Write vertex data
//            auto vertices = boost::vertices(*graphPtr);
//            for (auto vi = vertices.first; vi != vertices.second; ++vi) {
//                const Eigen::Vector3d &point = (*graphPtr)[*vi];
//                plyFile << point[0] << " " << point[1] << " " << point[2] << "\n";
//            }
//            // Write edge data
//            auto edges = boost::edges(*graphPtr);
//            for (auto ei = edges.first; ei != edges.second; ++ei) {
//                auto source = boost::source(*ei, *graphPtr);
//                auto target = boost::target(*ei, *graphPtr);
//                plyFile << source << " " << target << "\n";
//            }
//            plyFile.close();
//        }
// --------------------------------------------------------------------------
// Debug - Visualization
// --------------------------------------------------------------------------
//        // For visualization
//        std::shared_ptr<Eigen::MatrixXd> tip_points = std::make_shared<Eigen::MatrixXd>(tip_indices_.size(), 3);
//        for (int i = 0; i < tip_indices_.size(); ++i) {
//            tip_points->row(i) = cloudPtr->row(tip_indices_[i]);
//        }
//        tool::visualize::DrawPointClouds("Cloud", {{"Vertex", cloudPtr}});
//        polyscope::getPointCloud("Vertex")->addScalarQuantity("geodesic distance", distance);
//        tool::visualize::DrawPointClouds("Tip Points", {{"Tip", tip_points}});
//        std::shared_ptr<Eigen::MatrixXd> source_points = std::make_shared<Eigen::MatrixXd>(1, 3);
//        source_points->row(0) = cloudPtr->row(min_index);
//        tool::visualize::DrawPointClouds("Source Points", {{"Source", source_points}});
// --------------------------------------------------------------------------
// Debug - Visualization
// --------------------------------------------------------------------------
//struct JunctionFoundException : std::exception {
//    const char *what() const throw() {
//        return "Junction found";
//    }
//};
//
//struct bfs_visitor : public boost::default_bfs_visitor {
//    Boost_Vertex end_vertex;
//    bool &found;
//
//    bfs_visitor(Boost_Vertex v, bool &found) : end_vertex(v), found(found) {}
//
//    template<typename Vertex, typename Graph>
//    void discover_vertex(Vertex u, const Graph &g) {
//        if (u == end_vertex) {
//            found = true;
//            throw u; // Throw to stop the BFS when the junction is reached
//        }
//    }
//};