#include "Tools.h"


namespace tool {
	namespace preprocess {
		std::tuple<Eigen::Vector3d, double> Normalize(std::shared_ptr<Eigen::MatrixXd> &cloudPtr, double diagonal_length) {
			if (cloudPtr->rows() == 0 || cloudPtr->cols() != 3) {
				Logger::Instance().Log(std::format("Invalid Eigen::MatrixXd dimensions ({}, {})", cloudPtr->rows(), cloudPtr->cols()),
									   LogLevel::ERROR);
			}
			Eigen::Vector3d center = (cloudPtr->colwise().maxCoeff() + cloudPtr->colwise().minCoeff()) / 2.0;
			// Centre in zero
			for (int i = 0; i < cloudPtr->rows(); ++i) {
				Eigen::Vector3d point = cloudPtr->row(i);
				point -= center;
				cloudPtr->row(i) = point;
			}
			// Calculate the scaling factor to make the maximum diagonal length equal to diagonal_length
			Eigen::Vector3d diagonal = cloudPtr->colwise().maxCoeff() - cloudPtr->colwise().minCoeff();
			double max_diagonal = diagonal.maxCoeff();
			double normalization_scaling = diagonal_length / max_diagonal;
			// Apply the scaling
			*cloudPtr *= normalization_scaling;
			return std::make_tuple(center, normalization_scaling);
		}


		void UnNormalize(std::shared_ptr<Eigen::MatrixXd> &cloudPtr, const Eigen::Vector3d &center, const double &normalization_scaling) {
			for (int i = 0; i < cloudPtr->rows(); ++i) {
				Eigen::Vector3d point = cloudPtr->row(i);
				point /= normalization_scaling;
				point += center;
				cloudPtr->row(i) = point;
			}
		}


		void PreparePointCloud(std::shared_ptr<Eigen::MatrixXd> &final_cloudPtr, nlohmann::json &config) {
			auto *easy3d_cloud = new easy3d::PointCloud();
			if (!config["Input_Settings"]["With_Labels"].get<bool>()) {	 // No labels
					std::vector<easy3d::vec3> cloud_verts = io::LoadPointCloud(config);
					easy3d_cloud->points() = cloud_verts;
			} else {  // With labels
				io::FormatPointCloud(config);
				std::filesystem::path ply_path = config["Input_Settings"]["Input_Point_Cloud_File_Path"].get<std::filesystem::path>();
				Timer timer;
				easy3d_cloud = easy3d::PointCloudIO::load(ply_path);
				double elapsed = timer.elapsed<Timer::TimeUnit::Seconds>();
				Logger::Instance().Log(std::format("Original Point Cloud (#vertex: {}) has been loaded by Easy3D! Elapsed time: {:.6f}s", easy3d_cloud->n_vertices(), elapsed), LogLevel::INFO, true, false);
			}
			auto num_points = config["Preprocess"]["Down_Sample_Number"].get<unsigned int>();
			std::vector<int> semantic_labels;
			std::vector<int> instance_labels;
			Logger::Instance().AddLine(LogLine::DASH);
			Logger::Instance().Log(std::format("Start uniformly downsample the original Point Cloud to {}", num_points), LogLevel::INFO);
			Timer timer;
			// Down sample the point cloud
			if (easy3d_cloud->n_vertices() < num_points) {
				Logger::Instance().Log(std::format("The original Point Cloud point size (#vertex {}) is less than the downsample number (#num {})", easy3d_cloud->n_vertices(), num_points), LogLevel::ERROR);
			} else if (easy3d_cloud->n_vertices() > num_points) {
				std::unique_ptr<easy3d::KdTreeSearch_ETH> kdtree = std::make_unique<easy3d::KdTreeSearch_ETH>(easy3d_cloud);
				double initial_epsilon = easy3d::PointCloudSimplification::average_space(easy3d_cloud, kdtree.get(), 6, false, 10000) * 10.0;
				std::vector<easy3d::PointCloud::Vertex> indices_to_delete = easy3d::PointCloudSimplification::uniform_simplification(easy3d_cloud, static_cast<float>(initial_epsilon), kdtree.get());
				// Adjust epsilon until the number of points after deletion is within [num_points, num_points + 512]
				while (!(num_points <= easy3d_cloud->n_vertices() - static_cast<int>(indices_to_delete.size()) && num_points + 512 >= easy3d_cloud->n_vertices() - static_cast<int>(indices_to_delete.size()))) {
					if (num_points < easy3d_cloud->n_vertices() - static_cast<int>(indices_to_delete.size())) {
						initial_epsilon *= 1.1;
					} else {
						initial_epsilon *= 0.9;
					}
					indices_to_delete = easy3d::PointCloudSimplification::uniform_simplification(easy3d_cloud, static_cast<float>(initial_epsilon), kdtree.get());
				}
				// Delete vertices from the point cloud
				for (const auto &vertex: indices_to_delete) {
					easy3d_cloud->delete_vertex(easy3d::PointCloud::Vertex(vertex.idx()));
				}
				easy3d_cloud->collect_garbage();
				// Perform a second simplification to reach the exact number of num_points
				indices_to_delete = easy3d::PointCloudSimplification::uniform_simplification(easy3d_cloud, num_points);
				// Delete vertices from the point cloud again
				for (const auto &vertex: indices_to_delete) {
					easy3d_cloud->delete_vertex(easy3d::PointCloud::Vertex(vertex.idx()));
				}
				easy3d_cloud->collect_garbage();
				assert(easy3d_cloud->n_vertices() == num_points);
				if (config["Input_Settings"]["With_Labels"].get<bool>()) {  // Collect semantic and instance labels
					semantic_labels.resize(easy3d_cloud->n_vertices());
					instance_labels.resize(easy3d_cloud->n_vertices());
					auto semantic_label_name = config["Input_Settings"]["Labels_Names"]["PLY_Format"]["Semantic_Label_Name"].get<std::string>();
					auto instance_label_name = config["Input_Settings"]["Labels_Names"]["PLY_Format"]["Instance_Label_Name"].get<std::string>();
					final_cloudPtr->resize(easy3d_cloud->n_vertices(), 3);
					easy3d::PointCloud::VertexProperty<easy3d::vec3> points = easy3d_cloud->get_vertex_property<easy3d::vec3>("v:point");
					easy3d::PointCloud::VertexProperty<int> semantic = easy3d_cloud->get_vertex_property<int>("v:" + semantic_label_name);
					easy3d::PointCloud::VertexProperty<int> instance = easy3d_cloud->get_vertex_property<int>("v:" + instance_label_name);
					for (const easy3d::PointCloud::Vertex &vert: easy3d_cloud->vertices()) {
						final_cloudPtr->row(vert.idx()) = Eigen::Vector3d(points[vert].x, points[vert].y, points[vert].z);
						semantic_labels[vert.idx()] = semantic[vert];
						instance_labels[vert.idx()] = instance[vert];
					}
				} else {
					final_cloudPtr->resize(easy3d_cloud->n_vertices(), 3);
					easy3d::PointCloud::VertexProperty<easy3d::vec3> points = easy3d_cloud->get_vertex_property<easy3d::vec3>("v:point");
					for (const easy3d::PointCloud::Vertex &vert: easy3d_cloud->vertices()) {
						final_cloudPtr->row(vert.idx()) = Eigen::Vector3d(points[vert].x, points[vert].y, points[vert].z);
					}
				}
				// Normalize the point cloud
				Eigen::Vector3d center;
				double normalization_scaling;
				std::tie(center, normalization_scaling) = preprocess::Normalize(final_cloudPtr, config["Preprocess"]["Normalize_Diagonal_Length"].get<double>());
				// Update the normalization parameters in the configuration
				config["Preprocess"]["Normalize_Center"] = nlohmann::json::array({ center.x(), center.y(), center.z() });
				config["Preprocess"]["Normalize_Scaling"] = normalization_scaling;
			} else if (easy3d_cloud->n_vertices() == num_points) {
				// No need to down sample
				semantic_labels.resize(easy3d_cloud->n_vertices());
				instance_labels.resize(easy3d_cloud->n_vertices());
				auto semantic_label_name = config["Input_Settings"]["Labels_Names"]["PLY_Format"]["Semantic_Label_Name"].get<std::string>();
				auto instance_label_name = config["Input_Settings"]["Labels_Names"]["PLY_Format"]["Instance_Label_Name"].get<std::string>();
				final_cloudPtr->resize(easy3d_cloud->n_vertices(), 3);
				easy3d::PointCloud::VertexProperty<easy3d::vec3> points = easy3d_cloud->get_vertex_property<easy3d::vec3>("v:point");
				easy3d::PointCloud::VertexProperty<int> semantic = easy3d_cloud->get_vertex_property<int>("v:" + semantic_label_name);
				easy3d::PointCloud::VertexProperty<int> instance = easy3d_cloud->get_vertex_property<int>("v:" + instance_label_name);
				for (const easy3d::PointCloud::Vertex &vert: easy3d_cloud->vertices()) {
					final_cloudPtr->row(vert.idx()) = Eigen::Vector3d(points[vert].x, points[vert].y, points[vert].z);
					semantic_labels[vert.idx()] = semantic[vert];
					instance_labels[vert.idx()] = instance[vert];
				}
				// Normalize the point cloud
				Eigen::Vector3d center;
				double normalization_scaling;
				std::pair(center, normalization_scaling) =
						preprocess::Normalize(final_cloudPtr, config["Preprocess"]["Normalize_Diagonal_Length"].get<double>());
				config["Preprocess"]["Normalize_Center"] = { center(0), center(1), center(2) };
				config["Preprocess"]["Normalize_Scaling"] = normalization_scaling;
			}
			// Save the input point cloud
			if (config["Input_Settings"]["With_Labels"].get<bool>()) {
				std::pair<std::string, std::string> label_names = { config["Input_Settings"]["Labels_Names"]["PLY_Format"]["Semantic_Label_Name"].get<std::string>(), config["Input_Settings"]["Labels_Names"]["PLY_Format"]["Instance_Label_Name"].get<std::string>() };
				io::SavePointCloudToPLY(final_cloudPtr, config["Output_Settings"]["Output_Folder_Path"].get<std::filesystem::path>() / "0_Input.ply", semantic_labels, instance_labels, label_names);
			} else {
				io::SavePointCloudToPLY(final_cloudPtr, config["Output_Settings"]["Output_Folder_Path"].get<std::filesystem::path>() / "0_Input.ply");
			}
			double elapsed = timer.elapsed<Timer::TimeUnit::Seconds>();
			Logger::Instance().Log(std::format("Original Point Cloud has been uniformly downsampled to (#vertex {})! Elapsed time: {:.6f}s", num_points, elapsed), LogLevel::INFO, true, false);
		}
	}  // namespace preprocess


	namespace io {
		std::vector<easy3d::vec3> LoadPointCloud(nlohmann::json &config) {
			Logger::Instance().AddLine(LogLine::DASH);
			Logger::Instance().Log("Start Load Original Point Cloud");
			Timer timer;
			std::filesystem::path file_path = config["Input_Settings"]["Point_Cloud_File_Path"].get<std::filesystem::path>();
			std::filesystem::path output_folder_path = config["Output_Settings"]["Output_Folder_Path"].get<std::filesystem::path>();
			std::vector<easy3d::vec3> cloud_verts;
			if (file_path.extension() == ".ply") {
				cloud_verts = internal::LoadPointCloudFromPLY(file_path);
			} else if (file_path.extension() == ".xyz" || file_path.extension() == ".txt") {
				cloud_verts = internal::LoadPointCloudFromXYZTXT(file_path);
			} else {
				Logger::Instance().Log(std::format("Unsupported file format: {}", file_path.extension()), LogLevel::ERROR);
			}
			std::shared_ptr<Eigen::MatrixXd> cloudPtr = utility::Vector2Matrix(cloud_verts);
			SavePointCloudToPLY(cloudPtr, output_folder_path / "Original.ply", true);
			config["Input_Settings"]["Input_Point_Cloud_File_Path"] = output_folder_path / "Original.ply";
			double elapsed = timer.elapsed<Timer::TimeUnit::Seconds>();
			Logger::Instance().Log(std::format("Original Point Cloud (#vertex: {}) has been loaded has been made a backup PLY file! Elapsed time: {:.6f}s", cloud_verts.size(), elapsed), LogLevel::INFO, true, false);
			return cloud_verts;
		}


		void FormatPointCloud(const std::filesystem::path &file_path, nlohmann::json &config) {
			Logger::Instance().AddLine(LogLine::DASH);
			Logger::Instance().Log("Start Load Original Point Cloud and its labels");
			Timer timer;
			if (file_path.extension() == ".ply") {
				internal::FormatPointCloudFromPLY(config);
			} else if (file_path.extension() == ".xyz" || file_path.extension() == ".txt") {
				internal::FormatPointCloudFromXYZTXT(config);
			} else {
				Logger::Instance().Log(std::format("Unsupported file format: {}.", file_path.extension()), LogLevel::ERROR);
			}
			double elapsed = timer.elapsed<Timer::TimeUnit::Seconds>();
			Logger::Instance().Log(std::format("Original Point Cloud and its labels has been saved in one PLY file! Elapsed time: {:.6f}s.", elapsed), LogLevel::INFO, true, false);
		}


		namespace internal {
			std::vector<easy3d::vec3> LoadPointCloudFromPLY(const std::filesystem::path &file_path) {
				miniply::PLYReader reader(file_path.string().c_str());
				if (!reader.valid()) {
					Logger::Instance().Log(std::format("Failed to open the file ({}) for reading", file_path), LogLevel::ERROR);
				}
				bool got_verts = false;
				std::vector<easy3d::vec3> cloud_verts;
				while (reader.has_element() && !got_verts) {
					if (reader.element_is(miniply::kPLYVertexElement)) {
						uint32_t prop_idxs[3];
						if (!reader.load_element()) {
							Logger::Instance().Log("Failed to load vertex element", LogLevel::ERROR);
						}
						if (!reader.find_pos(prop_idxs)) {
							Logger::Instance().Log("Failed to find position properties", LogLevel::ERROR);
						}
						uint32_t numVerts = reader.num_rows();
						std::vector<double> positions(numVerts * 3);
						reader.extract_properties(prop_idxs, 3, miniply::PLYPropertyType::Double, positions.data());
						// Map the positions vector to an Eigen matrix
						cloud_verts.reserve(numVerts);
						for (uint32_t i = 0; i < numVerts; ++i) {
							easy3d::vec3 point;
							point.x() = positions[i * 3 + 0];
							point.y() = positions[i * 3 + 1];
							point.z() = positions[i * 3 + 2];
							cloud_verts.push_back(point);
						}
						got_verts = true;
					}
					reader.next_element();
				}
				if (!got_verts) {
					Logger::Instance().Log("No vertex data found", LogLevel::ERROR);
				}
				return cloud_verts;
			}

			void FormatPointCloudFromPLY(const std::filesystem::path &file_path, nlohmann::json &config) {
				std::filesystem::path output_folder_path = config["Output_Settings"]["Output_Folder_Path"].get<std::filesystem::path>();
				if (config["Input_Settings"]["Labels_Names"]["PLY_Format"]["Individual_Labels_File"].get<bool>()) {
					std::vector<easy3d::vec3> cloud_verts = LoadPointCloudFromPLY(file_path);
					std::filesystem::path semantic_label_file_path = config["Input_Settings"]["Labels_Names"]["PLY_Format"]["Labels_File_Paths"]["Semantic_Label_File_Path"].get<std::filesystem::path>();
					std::filesystem::path instance_label_file_path = config["Input_Settings"]["Labels_Names"]["PLY_Format"]["Labels_File_Paths"]["Instance_Label_File_Path"].get<std::filesystem::path>();
					std::vector<int> semantic_labels = LoadDataFromTXTXYZ<int>(semantic_label_file_path, 1);
					std::vector<int> instance_labels = LoadDataFromTXTXYZ<int>(instance_label_file_path, 1);
					if (semantic_labels.size() != cloud_verts.size() || instance_labels.size() != cloud_verts.size()) {
						Logger::Instance().Log("The number of labels does not match the number of vertices. Check the input label files and point cloud file.", LogLevel::ERROR);
					}
					std::shared_ptr<Eigen::MatrixXd> cloudPtr = utility::Vector2Matrix(cloud_verts);
					SavePointCloudToPLY(cloudPtr, output_folder_path / "Original.ply", semantic_labels, instance_labels, { "semantic", "instance" }, true);
					config["Input_Settings"]["Labels_Names"]["PLY_Format"]["Semantic_Label_Name"] = "semantic";
					config["Input_Settings"]["Labels_Names"]["PLY_Format"]["Instance_Label_Name"] = "instance";
					config["Input_Settings"]["Input_Point_Cloud_File_Path"] = output_folder_path / "Original.ply";
				} else {
					std::filesystem::copy_file(file_path, output_folder_path / "Original.ply", std::filesystem::copy_options::overwrite_existing);
					config["Input_Settings"]["Input_Point_Cloud_File_Path"] = output_folder_path / "Original.ply";
				}
			}


			std::vector<easy3d::vec3> LoadPointCloudFromXYZTXT(const std::filesystem::path &file_path) {
				std::vector<easy3d::vec3> cloud_verts = LoadDataFromTXTXYZ<easy3d::vec3>(file_path, 3);
				return cloud_verts;
			}


			void FormatPointCloudFromXYZTXT(nlohmann::json &config) {
				if (config["Input_Settings"]["Labels_Names"]["PLY_Format"]["Individual_Labels_File"].get<bool>()) {
					std::filesystem::path file_path = config["Input_Settings"]["Point_Cloud_File_Path"].get<std::filesystem::path>();
					std::filesystem::path output_folder_path = config["Output_Settings"]["Output_Folder_Path"].get<std::filesystem::path>();
					std::vector<Eigen::Vector3d> cloud_verts = LoadDataFromTXTXYZ<Eigen::Vector3d>(file_path, 3);
					std::filesystem::path semantic_label_file_path = config["Input_Settings"]["Labels_Names"]["PLY_Format"]["Labels_File_Paths"]["Semantic_Label_File_Path"].get<std::filesystem::path>();
					std::vector<int> semantic_labels = LoadDataFromTXTXYZ<int>(semantic_label_file_path, 1);
					std::filesystem::path instance_label_file_path = config["Input_Settings"]["Labels_Names"]["PLY_Format"]["Labels_File_Paths"]["Instance_Label_File_Path"].get<std::filesystem::path>();
					std::vector<int> instance_labels = LoadDataFromTXTXYZ<int>(instance_label_file_path, 1);
					if (semantic_labels.size() != cloud_verts.size() || instance_labels.size() != cloud_verts.size()) {
						Logger::Instance().Log(std::format("The number of labels (#semantic {}, #instance {}) does not match the number of vertices (#vertex {}). Check the input label files and point cloud file", semantic_labels.size(), instance_labels.size(), cloud_verts.size()), LogLevel::ERROR);
					}
					std::shared_ptr<Eigen::MatrixXd> cloudPtr = utility::Vector2Matrix(cloud_verts);
					SavePointCloudToPLY(cloudPtr, output_folder_path / "Original.ply", semantic_labels, instance_labels, { "semantic", "instance" }, true);
					config["Input_Settings"]["Point_Cloud_File_Path"] = output_folder_path / "Original.ply";
				} else {
					std::filesystem::path file_path = config["Input_Settings"]["Point_Cloud_File_Path"].get<std::filesystem::path>();
					std::filesystem::path output_folder_path = config["Output_Settings"]["Output_Folder_Path"].get<std::filesystem::path>();
					int total_columns = config["Input_Settings"]["TXT_XYZ_Format"]["Total_Columns"].get<int>();
					std::vector<Eigen::VectorXd> data = LoadDataFromTXTXYZ<Eigen::VectorXd>(file_path, total_columns);
					int semantic_label_index = config["Input_Settings"]["TXT_XYZ_Format"]["Semantic_Labels_Index"].get<int>();
					int instance_label_index = config["Input_Settings"]["TXT_XYZ_Format"]["Instance_Labels_Index"].get<int>();
					std::shared_ptr<Eigen::MatrixXd> cloudPtr = std::make_shared<Eigen::MatrixXd>(data.size(), 3);
					std::vector<int> semantic_labels;
					semantic_labels.reserve(data.size());
					std::vector<int> instance_labels;
					instance_labels.reserve(data.size());
					for (int i = 0; i < data.size(); ++i) {
						const Eigen::VectorXd row = data[i];
						(*cloudPtr)(i, 0) = row[0];
						(*cloudPtr)(i, 1) = row[1];
						(*cloudPtr)(i, 2) = row[2];
						semantic_labels.emplace_back(static_cast<int>(row[semantic_label_index]));
						instance_labels.emplace_back(static_cast<int>(row[instance_label_index]));
					}
					SavePointCloudToPLY(cloudPtr, output_folder_path / "Original.ply", semantic_labels, instance_labels, { "semantic", "instance" }, true);
					config["Input_Settings"]["Labels_Names"]["PLY_Format"]["Semantic_Label_Name"] = "semantic";
					config["Input_Settings"]["Labels_Names"]["PLY_Format"]["Instance_Label_Name"] = "instance";
					config["Input_Settings"]["Point_Cloud_File_Path"] = output_folder_path / "Original.ply";
				}
			}


			template<typename T>
			std::vector<T> LoadDataFromTXTXYZ(const std::filesystem::path &file_path, size_t dim) {
				if (dim == 0) {
					Logger::Instance().Log("Dimension must be greater than 0", LogLevel::ERROR);
				}
				std::ifstream file(file_path, std::ios::in | std::ios::binary);
				if (!file) {
					Logger::Instance().Log(std::format("Failed to open file ({}) for reading", file_path), LogLevel::ERROR);
				}
				constexpr size_t buffer_size = 16 * 1024 * 1024;  // 16 MB buffer
				std::vector<char> buffer(buffer_size);
				file.read(buffer.data(), static_cast<long>(buffer.size()));
				std::streamsize bytesRead = file.gcount();
				file.close();
				if constexpr (std::is_same_v<T, int> || std::is_same_v<T, double>) {
					if (dim != 1) {
						Logger::Instance().Log(std::format("Invalid dimension value (Error: {})", dim), LogLevel::ERROR);
					}
					return parse1DData<T>(buffer.data(), bytesRead, file_path);
				} else {
					return parseNDData<T>(buffer.data(), bytesRead, dim, file_path);
				}
			}

			template<typename T>
			std::vector<T> parse1DData(const char *data, std::streamsize bytes_read, const std::filesystem::path &file_path) {
				std::vector<T> result;
				result.reserve(1000000);
				std::istringstream iss(std::string(data, bytes_read));
				T value;
				while (iss >> value) {
					result.push_back(value);
				}
				if (iss.bad()) {
					Logger::Instance().Log(std::format("Error reading data from file ({})", file_path), LogLevel::ERROR);
				}
				return result;
			}

			template<typename T>
			std::vector<T> parseNDData(const char *data, std::streamsize bytes_read, size_t dim, const std::filesystem::path &file_path) {
				std::vector<T> result;
				result.reserve(1000000);
				std::istringstream iss(std::string(data, bytes_read));
				std::vector<double> temp_numbers;
				temp_numbers.reserve(dim);
				double value;
				if (std::is_same_v<T, easy3d::vec3> || std::is_same_v<T, Eigen::Vector3d>) {
					if (dim != 3) {
						Logger::Instance().Log(std::format("Invalid dimension value for easy3d::vec3 or Eigen::Vector3d (Error: {})", dim), LogLevel::ERROR);
					}
					while (iss >> value) {
						temp_numbers.push_back(value);
						if (temp_numbers.size() == dim) {
							T vec;
							for (size_t i = 0; i < dim; ++i) {
								vec(i) = temp_numbers[i];
							}
							result.push_back(vec);
							temp_numbers.clear();
						} else if (temp_numbers.size() > dim) {
							Logger::Instance().Log(std::format("Invalid data format in file ({})", file_path), LogLevel::ERROR);
						}
					}
				} else {
					while (iss >> value) {
						temp_numbers.push_back(value);
						if (temp_numbers.size() == dim) {
							T vec(dim);
							for (size_t i = 0; i < dim; ++i) {
								vec(i) = temp_numbers[i];
							}
							result.push_back(vec);
							temp_numbers.clear();
						} else if (temp_numbers.size() > dim) {
							Logger::Instance().Log(std::format("Invalid data format in file ({})", file_path), LogLevel::ERROR);
						}
					}
				}
				if (!temp_numbers.empty()) {
					Logger::Instance().Log(std::format("Incomplete vector data found at the end of file ({})", file_path), LogLevel::ERROR);
				}
				if (iss.bad()) {
					Logger::Instance().Log(std::format("Error reading data from file ({})", file_path), LogLevel::ERROR);
				}
				return result;
			}


			namespace under_construction {
				// TODO: Support .obj file?
				Eigen::MatrixXd LoadPointCloudFromOBJ(const std::filesystem::path &file_path) {
					tinyobj::ObjReaderConfig reader_config;
					reader_config.mtl_search_path = "";	 // Ignore the MTL file
					tinyobj::ObjReader reader;
					if (!reader.ParseFromFile(file_path, reader_config)) {
						if (!reader.Error().empty()) {
							std::cerr << "TinyObjReader: " << reader.Error() << std::endl;
						}
						exit(1);
					}
					if (!reader.Warning().empty()) {
						std::cout << "TinyObjReader: " << reader.Warning() << std::endl;
					}
					auto &attrib = reader.GetAttrib();
					int num_points = static_cast<int>(attrib.vertices.size() / 3);
					Eigen::MatrixXd cloud(num_points, 3);
#pragma omp parallel for
					for (int i = 0; i < num_points; ++i) {
						cloud(i, 0) = attrib.vertices[3 * i + 0];
						cloud(i, 1) = attrib.vertices[3 * i + 1];
						cloud(i, 2) = attrib.vertices[3 * i + 2];
					}
					return cloud;
				}
			}  // namespace under_construction
		}  // namespace internal


		void SavePointCloudToPLY(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, const std::filesystem::path &file_path, bool binary) {
			auto num_pts = static_cast<int32_t>(cloudPtr->rows());
			msh_ply_desc_t descriptors[1];
			// Vertices (x, y, z)
			descriptors[0] = {
				.element_name = 'vertex',
				.property_names = (const char *[]){ "x", "y", "z" },
				.num_properties = 3,
				.data_type = MSH_PLY_DOUBLE,
				.data = cloudPtr->data(),
				.data_count = &num_pts
			};
			// Create a new PLY file for writing (binary or ASCII)
			msh_ply_t *ply_file = msh_ply_open(file_path.string().c_str(), binary ? "wb" : "w");
			if (!ply_file) {
				Logger::Instance().Log(std::format("Failed to open PLY file ({})", file_path), LogLevel::ERROR);
			}
			// Add descriptors to the PLY file
			msh_ply_add_descriptor(ply_file, &descriptors[0]);	// Vertices
			// Write the data to the PLY file
			if (msh_ply_write(ply_file) != 0) {
				msh_ply_close(ply_file);
				Logger::Instance().Log(std::format("Failed to write PLY file ({})", file_path), LogLevel::ERROR);
			}
			// Close the file
			msh_ply_close(ply_file);
		}

		void SavePointCloudToPLY(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr,
								 const std::filesystem::path &file_path,
								 std::vector<double> &property,
								 const std::string &property_name,
								 bool binary) {
			if (cloudPtr->rows() == 0 || cloudPtr->rows() != property.size()) {
				Logger::Instance().Log(std::format("Point cloud (#vertex {}) or property (#size {}) has invalid sizes.", cloudPtr->rows(), property.size()), LogLevel::ERROR);
			}
			auto num_pts = static_cast<int32_t>(cloudPtr->rows());
			msh_ply_desc_t descriptors[2];
			// Vertices (x, y, z)
			descriptors[0] = {
				.element_name = 'vertex',
				.property_names = (const char *[]){ "x", "y", "z" },
				.num_properties = 3,
				.data_type = MSH_PLY_DOUBLE,
				.data = cloudPtr->data(),
				.data_count = &num_pts
			};
			// Property
			descriptors[1] = {
				.element_name = 'vertex',
				.property_names = (const char *[]){ property_name.c_str() },
				.num_properties = 1,
				.data_type = MSH_PLY_DOUBLE,
				.data = property.data(),
				.data_count = &num_pts
			};
			// Create a new PLY file for writing (binary or ASCII)
			msh_ply_t *ply_file = msh_ply_open(file_path.string().c_str(), binary ? "wb" : "w");
			if (!ply_file) {
				Logger::Instance().Log(std::format("Failed to open PLY file ({})", file_path), LogLevel::ERROR);
			}
			// Add descriptors to the PLY file
			msh_ply_add_descriptor(ply_file, &descriptors[0]);	// Vertices
			msh_ply_add_descriptor(ply_file, &descriptors[1]);	// Label one
			msh_ply_add_descriptor(ply_file, &descriptors[2]);	// Label two
			// Write the data to the PLY file
			if (msh_ply_write(ply_file) != 0) {
				msh_ply_close(ply_file);
				Logger::Instance().Log(std::format("Failed to write PLY file ({})", file_path), LogLevel::ERROR);
			}
			// Close the file
			msh_ply_close(ply_file);
		}

		void SavePointCloudToPLY(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr,
								 const std::filesystem::path &file_path,
								 std::vector<int> &label_one,
								 std::vector<int> &label_two,
								 const std::pair<std::string, std::string> &label_names,
								 bool binary) {
			if (cloudPtr->rows() == 0 || cloudPtr->rows() != label_one.size() || cloudPtr->rows() != label_two.size()) {
				Logger::Instance().Log(std::format("Point cloud (#vertex {}), label one (#size {}) or label two (#size {}) has invalid sizes.", cloudPtr->rows(), label_one.size(), label_two.size()), LogLevel::ERROR);
			}
			auto num_pts = static_cast<int32_t>(cloudPtr->rows());
			msh_ply_desc_t descriptors[3];
			// Vertices (x, y, z)
			descriptors[0] = {
				.element_name = 'vertex',
				.property_names = (const char *[]){ "x", "y", "z" },
				.num_properties = 3,
				.data_type = MSH_PLY_DOUBLE,
				.data = cloudPtr->data(),
				.data_count = &num_pts
			};
			// First label set
			descriptors[1] = {
				.element_name = 'vertex',
				.property_names = (const char *[]){ label_names.first.c_str() },
				.num_properties = 1,
				.data_type = MSH_PLY_INT32,
				.data = label_one.data(),
				.data_count = &num_pts
			};
			// Second label set
			descriptors[2] = {
				.element_name = 'vertex',
				.property_names = (const char *[]){ label_names.second.c_str() },
				.num_properties = 1,
				.data_type = MSH_PLY_INT32,
				.data = label_two.data(),
				.data_count = &num_pts
			};
			// Create a new PLY file for writing (binary or ASCII)
			msh_ply_t *ply_file = msh_ply_open(file_path.string().c_str(), binary ? "wb" : "w");
			if (!ply_file) {
				Logger::Instance().Log(std::format("Failed to open PLY file ({})", file_path), LogLevel::ERROR);
			}
			// Add descriptors to the PLY file
			msh_ply_add_descriptor(ply_file, &descriptors[0]);	// Vertices
			msh_ply_add_descriptor(ply_file, &descriptors[1]);	// Label one
			msh_ply_add_descriptor(ply_file, &descriptors[2]);	// Label two
			// Write the data to the PLY file
			if (msh_ply_write(ply_file) != 0) {
				msh_ply_close(ply_file);
				Logger::Instance().Log(std::format("Failed to write PLY file ({})", file_path), LogLevel::ERROR);
			}
			// Close the file
			msh_ply_close(ply_file);
		}


		void SaveSkeletonGraphToPLY(const std::shared_ptr<Boost_Graph> &graphPtr, const std::filesystem::path &file_path, bool binary) {
			std::vector<Eigen::Vector3d> cloud_verts;
			cloud_verts.resize(boost::num_vertices(*graphPtr));
			for (int i = 0; i < boost::num_vertices(*graphPtr); ++i) {
				cloud_verts.at(i) = { (*graphPtr)[i].x(), (*graphPtr)[i].y(), (*graphPtr)[i].z() };
			}


			std::vector<int> edges_vertex1;
			std::vector<int> edges_vertex2;
			for (auto ei = boost::edges(*graphPtr).first; ei != boost::edges(*graphPtr).second; ++ei) {
				auto source = boost::source(*ei, *graphPtr);
				auto target = boost::target(*ei, *graphPtr);
				edges_vertex1.push_back(static_cast<int>(source));
				edges_vertex2.push_back(static_cast<int>(target));
			}
			assert(edges_vertex1.size() == edges_vertex2.size());
			plyOut.addElement("edge", edges_vertex1.size());
			plyOut.getElement("edge").addProperty<int>("vertex1", edges_vertex1);
			plyOut.getElement("edge").addProperty<int>("vertex2", edges_vertex2);
			if (binary) {
				try {
					plyOut.write(file_path, happly::DataFormat::Binary);
				} catch (const std::exception &e) {
					MyLogger.Log(e.what(), 1, true, true);
					std::exit(EXIT_FAILURE);
				}
			} else {
				try {
					plyOut.write(file_path, happly::DataFormat::ASCII);
				} catch (const std::exception &e) {
					MyLogger.Log(e.what(), 1, true, true);
					std::exit(EXIT_FAILURE);
				}
			}
		}

		void SaveSkeletonGraphToPLY(const std::shared_ptr<Boost_Graph> &graphPtr,
									const std::filesystem::path &file_path,
									const std::vector<int> &vertex_labels,
									bool binary) {
			happly::PLYData plyOut;
			std::vector<std::array<double, 3>> vertices;
			vertices.resize(boost::num_vertices(*graphPtr));
			for (int i = 0; i < boost::num_vertices(*graphPtr); ++i) {
				vertices.at(i) = { (*graphPtr)[i].x(), (*graphPtr)[i].y(), (*graphPtr)[i].z() };
			}
			plyOut.addVertexPositions(vertices);
			plyOut.getElement("vertex").addProperty<int>("label", vertex_labels);
			std::vector<int> edges_vertex1;
			std::vector<int> edges_vertex2;
			for (auto ei = boost::edges(*graphPtr).first; ei != boost::edges(*graphPtr).second; ++ei) {
				auto source = boost::source(*ei, *graphPtr);
				auto target = boost::target(*ei, *graphPtr);
				edges_vertex1.push_back(static_cast<int>(source));
				edges_vertex2.push_back(static_cast<int>(target));
			}
			assert(edges_vertex1.size() == edges_vertex2.size());
			plyOut.addElement("edge", edges_vertex1.size());
			plyOut.getElement("edge").addProperty<int>("vertex1", edges_vertex1);
			plyOut.getElement("edge").addProperty<int>("vertex2", edges_vertex2);
			if (binary) {
				try {
					plyOut.write(file_path, happly::DataFormat::Binary);
				} catch (const std::exception &e) {
					MyLogger.Log(e.what(), 1, true, true);
					std::exit(EXIT_FAILURE);
				}
			} else {
				try {
					plyOut.write(file_path, happly::DataFormat::ASCII);
				} catch (const std::exception &e) {
					MyLogger.Log(e.what(), 1, true, true);
					std::exit(EXIT_FAILURE);
				}
			}
		}


		void SaveJSONFile(const nlohmann::json &config, const std::filesystem::path &file_path) {
			std::ofstream outFile(file_path);
			if (outFile.is_open()) {
				outFile << config.dump(4);
				outFile.close();
			} else {
				MyLogger.Log(std::format("Unable to open file for writing: {}.", file_path.string()), 1, true, true);
				std::exit(EXIT_FAILURE);
			}
		}
	}  // namespace io



	namespace utility {
		template<typename T>
		std::shared_ptr<Eigen::MatrixXd> Vector2Matrix(const std::vector<T> &cloud_verts) {
			std::shared_ptr<Eigen::MatrixXd> cloudPtr = std::make_shared<Eigen::MatrixXd>(cloud_verts.size(), 3);
			Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>> mapped_data(reinterpret_cast<const double*>(cloud_verts.data()), static_cast<int>(cloud_verts.size()), 3);
			*cloudPtr = mapped_data;
			return cloudPtr;
		}


		std::vector<geometrycentral::Vector3> Matrix2GCVector(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr) {
			std::vector<geometrycentral::Vector3> points;
			for (int i = 0; i < cloudPtr->rows(); ++i) {
				geometrycentral::Vector3 pos{ (*cloudPtr)(i, 0), (*cloudPtr)(i, 1), (*cloudPtr)(i, 2) };
				points.emplace_back(pos);
			}
			return points;
		}


		std::vector<std::vector<size_t>> KNNSearch(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, const size_t k) {
			std::vector<geometrycentral::Vector3> points = Matrix2GCVector(cloudPtr);
			geometrycentral::NearestNeighborFinder finder(points);
			std::vector<std::vector<size_t>> result;
			for (size_t i = 0; i < points.size(); i++) {
				result.emplace_back(finder.kNearestNeighbors(i, k));
			}
			return result;
		}


		std::vector<std::vector<size_t>> RadiusSearch(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, const double radius) {
			std::vector<geometrycentral::Vector3> points = Matrix2GCVector(cloudPtr);
			geometrycentral::NearestNeighborFinder finder(points);
			std::vector<std::vector<size_t>> result;
			for (const geometrycentral::Vector3 &point: points) {
				std::vector<size_t> temp = finder.radiusSearch(point, radius);
				result.emplace_back(temp);
			}
			return result;
		}


		Eigen::MatrixXd ComputeCovariance(const Eigen::MatrixXd &cloud) {
			Eigen::Vector3d mean = cloud.colwise().mean();
			Eigen::MatrixXd centered = cloud.rowwise() - mean.transpose();
			Eigen::MatrixXd covariance = (centered.adjoint() * centered) / double(cloud.rows() - 1);
			return covariance;
		}


		std::vector<int> FindUpperOutlierBySTD(const std::vector<double> &data) {
			int n = static_cast<int>(data.size());
			if (n == 0)
				return {};	// Handle empty data case
			// Calculate the mean
			double sum = std::accumulate(data.begin(), data.end(), 0.0);
			double mean = sum / n;
			// Calculate the standard deviation
			double sq_sum = std::accumulate(
					data.begin(), data.end(), 0.0, [mean](double acc, double value) { return acc + (value - mean) * (value - mean); });
			double std = std::sqrt(sq_sum / n);
			// Detect outliers
			std::vector<int> outliers_indices;
			for (int i = 0; i < n; ++i) {
				if (data[i] - mean > 3 * std) {
					outliers_indices.push_back(i);
				}
			}
			return outliers_indices;
		}


		// This function is adapted from Open3D,
		// https://github.com/isl-org/Open3D/blob/main/cpp/open3d/geometry/PointCloud.cpp
		std::shared_ptr<Eigen::MatrixXd> VoxelDownSample(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, double voxel_size) {
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

				[[nodiscard]] Eigen::Vector3d GetAveragePoint() const {
					if (count > 0)
						return sum / double(count);
					return Eigen::Vector3d::Zero();	 // return zero vector if no points were added
				}
			};
			if (voxel_size <= 0.0) {
				MyLogger.Log("voxel_size <= 0.0", 1, true, true);
				std::exit(EXIT_FAILURE);
			}
			Eigen::Vector3d voxel_size3 = Eigen::Vector3d::Constant(voxel_size);
			Eigen::Vector3d voxel_max_bound = cloudPtr->colwise().maxCoeff().array() - voxel_size / 2.0;
			Eigen::Vector3d voxel_min_bound = cloudPtr->colwise().minCoeff().array() - voxel_size / 2.0;
			if (voxel_size * std::numeric_limits<int>::max() < (voxel_max_bound - voxel_min_bound).maxCoeff()) {
				MyLogger.Log("voxel_size is too small.", 1, true, true);
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
			std::shared_ptr<Eigen::MatrixXd> output = std::make_shared<Eigen::MatrixXd>(voxelindex_to_accpoint.size(), 3);
			int idx = 0;
			for (const auto &accpoint: voxelindex_to_accpoint) {
				output->row(idx++) = accpoint.second.GetAveragePoint();
			}
			return output;
		}


		// This function is adapted from Open3D,
		// https://github.com/isl-org/Open3D/blob/main/cpp/open3d/geometry/PointCloud.cpp
		std::vector<size_t> FarthestPointDownSample(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, int num_samples) {
			if (num_samples <= 0) {
				MyLogger.Log("Number of samples should be larger than 0.", 1, true, true);
				std::exit(EXIT_FAILURE);
			} else if (num_samples >= cloudPtr->rows()) {
				// Modified condition to cover all cases where num_samples >= num_points
				if (num_samples > cloudPtr->rows()) {
					MyLogger.Log(std::format("Number of samples ({}) should be smaller than the total point ({}).", num_samples, cloudPtr->rows()),
								 1,
								 true,
								 true);
					std::exit(EXIT_FAILURE);
				}
				std::vector<size_t> indices(cloudPtr->rows());
				std::iota(indices.begin(), indices.end(), 0);
				return indices;
			}
			std::vector<size_t> selected_indices;
			selected_indices.reserve(num_samples);
			const size_t num_points = cloudPtr->rows();
			std::vector<double> distances(num_points, std::numeric_limits<double>::infinity());
			// You can use a random starting point to avoid degenerate cases
			size_t farthest_index = 42 % num_points;  // We use 42 as the seed for reproducibility
			for (size_t i = 0; i < num_samples; ++i) {
				selected_indices.push_back(farthest_index);
				const Eigen::Vector3d &selected = cloudPtr->row(int(farthest_index));
#pragma omp parallel for
				for (size_t j = 0; j < num_points; ++j) {
					const Eigen::Vector3d &pt = cloudPtr->row(j);
					double dist = (pt - selected).squaredNorm();
#pragma omp critical
					{ distances[j] = std::min(distances[j], dist); }
				}
				// Use std::max_element to find the farthest point
				farthest_index = std::distance(distances.begin(), std::max_element(distances.begin(), distances.end()));
			}
			return selected_indices;
		}


		Eigen::MatrixXd SelectByIndices(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, const std::vector<size_t> &indices, const int &dim) {
			Eigen::MatrixXd selected_cloud(indices.size(), dim);
			for (int i = 0; i < indices.size(); ++i) {
				selected_cloud.row(i) = cloudPtr->row(int(indices[i]));
			}
			return selected_cloud;
		}


		std::vector<std::vector<int>> NearestProjectFromBoostVerts(std::vector<std::vector<Boost_Vertex>> &classes,
																   std::shared_ptr<Boost_Graph> &graphPtr,
																   std::shared_ptr<Eigen::MatrixXd> &cloudPtr) {
			auto start = std::chrono::high_resolution_clock::now();
			MyLogger.Log("--------------------------------------------------", 0, true, false);
			MyLogger.Log("Start Stem-Leaf detection", 0, true, true);

			std::shared_ptr<Eigen::MatrixXd> skeleton_cloudPtr = std::make_shared<Eigen::MatrixXd>(graphPtr->m_vertices.size(), 3);
			for (int i = 0; i < graphPtr->m_vertices.size(); ++i) {
				skeleton_cloudPtr->row(i) = (*graphPtr)[i];
			}
			std::vector<geometrycentral::Vector3> skeleton_points = tool::utility::Matrix2GCVector(skeleton_cloudPtr);
			geometrycentral::NearestNeighborFinder skeleton_finder(skeleton_points);
			std::vector<int> nearest_skeleton_index;
			nearest_skeleton_index.reserve(cloudPtr->rows());
			for (int i = 0; i < cloudPtr->rows(); ++i) {
				std::vector<size_t> indices;
				geometrycentral::Vector3 query_point = { (*cloudPtr)(i, 0), (*cloudPtr)(i, 1), (*cloudPtr)(i, 2) };
				indices = skeleton_finder.kNearest(query_point, 1);
				nearest_skeleton_index.emplace_back(indices[0]);
			}
			// Prepare a mapping from skeleton index to class index
			std::unordered_map<size_t, int> skeleton_to_class;
			for (int i = 0; i < classes.size(); ++i) {
				for (size_t idx: classes[i]) {
					skeleton_to_class[idx] = i;
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

			auto end = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> elapsed = end - start;
			MyLogger.Log(std::format("Stem-Leaf detection have been completed! Elapsed time: {:.6f}s", elapsed.count()), 0, true, false);
			return result;
		}
	}  // namespace utility



	namespace debug {
		void SaveSphereToPLY(const Eigen::Vector3d &center, double radius, const std::filesystem::path &file_path) {
			std::vector<Eigen::Vector3d> vertices;
			std::vector<std::vector<int>> faces;
			int sectorCount = 36;
			int stackCount = 18;
			double x, y, z, xy;	 // vertex position
			double sectorStep = 2 * M_PI / sectorCount;
			double stackStep = M_PI / stackCount;
			double sectorAngle, stackAngle;
			// Generate vertices
			for (int i = 0; i <= stackCount; ++i) {
				stackAngle = M_PI / 2 - i * stackStep;	// starting from pi/2 to -pi/2
				xy = radius * cos(stackAngle);	// r * cos(u)
				z = radius * sin(stackAngle);  // r * sin(u)
				for (int j = 0; j <= sectorCount; ++j) {
					sectorAngle = j * sectorStep;  // starting from 0 to 2pi
					x = xy * cos(sectorAngle);	// r * cos(u) * cos(v)
					y = xy * sin(sectorAngle);	// r * cos(u) * sin(v)
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
						faces.push_back({ k1, k2, k1 + 1 });
					}
					if (i != (stackCount - 1)) {
						faces.push_back({ k1 + 1, k2, k2 + 1 });
					}
				}
			}
			// Write to Binary
			std::ofstream plyFile(file_path);
			if (!plyFile.is_open()) {
				MyLogger.Log(std::format("Failed to open file ({}) for writing", file_path), 1, true, true);
				std::exit(EXIT_FAILURE);
			}
			plyFile << std::fixed << std::setprecision(16);
			// Binary header
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
					if (i < face.size() - 1)
						plyFile << " ";
				}
				plyFile << "\n";
			}
			plyFile.close();
		}


		namespace visualize {
			void DrawPointClouds(const std::string &group_title,
								 const std::vector<std::pair<std::string, std::shared_ptr<Eigen::MatrixXd>>> &cloudsPairs) {
				if (polyscope::isInitialized()) {
					try {
						polyscope::getGroup(group_title);
					} catch (std::runtime_error &e) {
						polyscope::createGroup(group_title);
					}
					polyscope::Group *group = polyscope::getGroup(group_title);
					for (int i = 0; i < cloudsPairs.size(); ++i) {
						const std::string &cloudName = cloudsPairs[i].first;
						const std::shared_ptr<Eigen::MatrixXd> &cloudMatrix = cloudsPairs[i].second;
						std::vector<geometrycentral::Vector3> points = utility::Matrix2GCVector(cloudMatrix);
						polyscope::PointCloud *psCloud = polyscope::registerPointCloud(cloudName, points);
						if (cloudsPairs.size() > 1 && i == 0) {
							psCloud->setPointRadius(0.001);
						} else {
							psCloud->setPointRadius(0.0015);
						}
						psCloud->setPointRenderMode(polyscope::PointRenderMode::Sphere);
						// Add point cloud to the group
						psCloud->addToGroup(*group);
					}
					// Set some options
					group->setEnabled(true);
					group->setHideDescendantsFromStructureLists(true);
					group->setShowChildDetails(true);
				} else {
					// Initialize
					polyscope::init();

					// Some options
					polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;
					polyscope::options::groundPlaneHeightFactor = 0.1;
					polyscope::options::shadowDarkness = 0.3;
					// A few camera options
					polyscope::view::setNavigateStyle(polyscope::NavigateStyle::Turntable);
					polyscope::view::setUpDir(polyscope::UpDir::ZUp);
					polyscope::view::setFrontDir(polyscope::FrontDir::XFront);
					// Create a group
					polyscope::Group *group = polyscope::createGroup(group_title);
					// Register the point clouds
					for (int i = 0; i < cloudsPairs.size(); ++i) {
						const std::string &cloudName = cloudsPairs[i].first;
						const std::shared_ptr<Eigen::MatrixXd> &cloudMatrix = cloudsPairs[i].second;
						std::vector<geometrycentral::Vector3> points = utility::Matrix2GCVector(cloudMatrix);
						polyscope::PointCloud *psCloud = polyscope::registerPointCloud(cloudName, points);
						if (cloudsPairs.size() > 1 && i == 0) {
							psCloud->setPointRadius(0.001);
						} else {
							psCloud->setPointRadius(0.0015);
						}
						psCloud->setPointRenderMode(polyscope::PointRenderMode::Sphere);
						// Add point cloud to the group
						psCloud->addToGroup(*group);
					}
					// Set some options
					group->setEnabled(true);
					group->setHideDescendantsFromStructureLists(true);
					group->setShowChildDetails(true);
				}
				// Show the GUI
				polyscope::show();
			}


			void DrawUnionLocalTriangles(const std::string &title,
										 const std::shared_ptr<geometrycentral::pointcloud::PointCloud> &gc_cloudPtr,
										 const std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> &gc_geom) {
				// Initialize
				polyscope::init();
				// Some options
				polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;
				polyscope::options::groundPlaneHeightFactor = 0.1;
				polyscope::options::shadowDarkness = 0.3;
				// A few camera options
				polyscope::view::setNavigateStyle(polyscope::NavigateStyle::Turntable);
				polyscope::view::setUpDir(polyscope::UpDir::ZUp);
				polyscope::view::setFrontDir(polyscope::FrontDir::XFront);
				// Generate the local triangles
				geometrycentral::pointcloud::PointData<std::vector<std::array<geometrycentral::pointcloud::Point, 3>>> localTriPoint =
						geometrycentral::pointcloud::buildLocalTriangulations(*gc_cloudPtr, *gc_geom, true);
				// Make a union mesh
				std::vector<std::vector<size_t>> allTris = handleToFlatInds(*gc_cloudPtr, localTriPoint);
				std::vector<geometrycentral::Vector3> posRaw(gc_cloudPtr->nPoints());
				for (size_t iP = 0; iP < posRaw.size(); iP++) {
					posRaw[iP] = gc_geom->positions[iP];
				}
				// Register the mesh
				polyscope::registerSurfaceMesh(title, posRaw, allTris);
				// Show the GUI
				polyscope::show();
			}


			void DrawTuftedMesh(const std::string &title, const std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> &gc_geom) {
				// Initialize
				polyscope::init();
				// Some options
				polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;
				polyscope::options::groundPlaneHeightFactor = 0.1;
				polyscope::options::shadowDarkness = 0.3;
				// A few camera options
				polyscope::view::setNavigateStyle(polyscope::NavigateStyle::Turntable);
				polyscope::view::setUpDir(polyscope::UpDir::ZUp);
				polyscope::view::setFrontDir(polyscope::FrontDir::XFront);
				// Generate the Tufted mesh
				if (!gc_geom->tuftedMesh) {
					gc_geom->requireTuftedTriangulation();
				}
				// Register the mesh
				polyscope::registerSurfaceMesh(title, gc_geom->positions, gc_geom->tuftedMesh->getFaceVertexList());
				// Show the GUI
				polyscope::show();
			}


			void DrawTangentPoints(const std::string &title, const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, const int k, const int center_index) {
				std::shared_ptr<geometrycentral::pointcloud::PointCloud> gc_cloudPtr =
						std::make_shared<geometrycentral::pointcloud::PointCloud>(cloudPtr->rows());
				std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> gc_geom =
						std::make_shared<geometrycentral::pointcloud::PointPositionGeometry>(*gc_cloudPtr);
				std::vector<geometrycentral::Vector3> vertices_positions = tool::utility::Matrix2GCVector(cloudPtr);
				for (int i = 0; i < cloudPtr->rows(); ++i) {
					gc_geom->positions[i] = vertices_positions[i];
				}
				gc_geom->kNeighborSize = k;


				// Prepare data
				gc_geom->requireNeighbors();
				gc_geom->requireTangentBasis();
				gc_geom->requireTangentCoordinates();
				std::vector<geometrycentral::pointcloud::Point> neigh = gc_geom->neighbors->neighbors[center_index];
				std::vector<geometrycentral::Vector3> neigh_points;
				neigh_points.reserve(neigh.size());
				for (const geometrycentral::pointcloud::Point &point: neigh) {
					neigh_points.emplace_back(gc_geom->positions[point.getIndex()]);
				}


				std::vector<geometrycentral::Vector2> tangentCoords = gc_geom->tangentCoordinates[center_index];
				std::vector<geometrycentral::Vector3> origin;
				origin.emplace_back(geometrycentral::Vector3{ 0.0, 0.0, 0.0 });


				geometrycentral::pointcloud::PointData<std::vector<std::array<geometrycentral::pointcloud::Point, 3>>> localTriPoint =
						geometrycentral::pointcloud::buildLocalTriangulations(*gc_cloudPtr, *gc_geom, true);
				std::vector<std::array<geometrycentral::pointcloud::Point, 3>> localTri = localTriPoint[center_index];
				std::vector<std::vector<size_t>> tris;
				tris.reserve(localTri.size());
				for (const std::array<geometrycentral::pointcloud::Point, 3> &tri: localTri) {
					std::vector<size_t> temp;
					temp.reserve(tri.size());
					for (const geometrycentral::pointcloud::Point &point: tri) {
						temp.emplace_back(point.getIndex());
					}
					tris.emplace_back(temp);
				}
				std::vector<geometrycentral::Vector3> new_points;
				geometrycentral::Vector3 center = gc_geom->positions[center_index];
				geometrycentral::Vector3 normal = gc_geom->normals[center_index];
				geometrycentral::Vector3 basisX = gc_geom->tangentBasis[center_index][0];
				geometrycentral::Vector3 basisY = gc_geom->tangentBasis[center_index][1];
				for (size_t iN = 0; iN < gc_geom->positions.size(); iN++) {
					geometrycentral::Vector3 vec = gc_geom->positions[iN] - center;
					vec = vec.removeComponent(normal);
					geometrycentral::Vector3 coord{ dot(basisX, vec), dot(basisY, vec), 0.0 };
					new_points.push_back(coord);
				}
				std::vector<geometrycentral::Vector3> new_neigh_points;
				new_neigh_points.reserve(neigh.size());
				for (const geometrycentral::pointcloud::Point &point: neigh) {
					new_neigh_points.emplace_back(new_points[point.getIndex()]);
				}

				gc_geom->requireTuftedTriangulation();
				std::vector<std::vector<size_t>> tufted_tri;
				for (const std::vector<size_t> &tri: gc_geom->tuftedMesh->getFaceVertexList()) {
					for (size_t vertex: tri) {
						if (vertex == center_index) {
							tufted_tri.push_back(tri);
						}
					}
				}

				// Initialize
				polyscope::init();
				// Some options
				polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;
				polyscope::options::groundPlaneHeightFactor = -0.760f;
				polyscope::options::shadowDarkness = 0.5;
				// A few camera options
				polyscope::view::setNavigateStyle(polyscope::NavigateStyle::Turntable);
				polyscope::view::setUpDir(polyscope::UpDir::ZUp);
				polyscope::view::setFrontDir(polyscope::FrontDir::XFront);
				polyscope::Group *group = polyscope::createGroup(title);
				// Register

				//            polyscope::PointCloud *psCloud_1 = polyscope::registerPointCloud("pc_1", vertices_positions);
				//            psCloud_1->setPointRadius(0.0005);
				//            psCloud_1->setPointColor(glm::vec3(145.0/255.0, 146.0/255.0, 145.0/255.0));
				//            psCloud_1->addToGroup(*group);
				//            polyscope::PointCloud *psCloud_2 = polyscope::registerPointCloud("pc_2",
				//            std::vector<geometrycentral::Vector3>{vertices_positions[center_index]}); psCloud_2->setPointRadius(0.0010);
				//            psCloud_2->setPointColor(glm::vec3(255.0/255.0, 0.0/255.0, 0.0/255.0));
				//            psCloud_2->addToGroup(*group);
				//            polyscope::PointCloud *psCloud_3 = polyscope::registerPointCloud("pc_3", neigh_points);
				//            psCloud_3->setPointRadius(0.0010);
				//            psCloud_3->setPointColor(glm::vec3(49.0/255.0, 49.0/255.0, 129.0/255.0));
				//            psCloud_3->addToGroup(*group);

				//            polyscope::PointCloud *psCloud_1 = polyscope::registerPointCloud2D("pc_1", tangentCoords);
				//            psCloud_1->setPointRadius(0.001);
				//            psCloud_1->setPointColor(glm::vec3(49.0/255.0, 49.0/255.0, 129.0/255.0));
				//            psCloud_1->addToGroup(*group);
				//            polyscope::PointCloud *psCloud_2 = polyscope::registerPointCloud2D("pc_2", origin);
				//            psCloud_2->setPointRadius(0.001);
				//            psCloud_2->setPointColor(glm::vec3(255.0/255.0, 0.0/255.0, 0.0/255.0));
				//            psCloud_2->addToGroup(*group);
				//
				//          // In 3D space, show triangles
				polyscope::SurfaceMesh *psMesh_1 = polyscope::registerSurfaceMesh("mesh_1", new_points, tris);
				psMesh_1->setEdgeWidth(2.0);
				psMesh_1->setTransparency(0.8);
				psMesh_1->addToGroup(*group);
				polyscope::PointCloud *psCloud_1 = polyscope::registerPointCloud("pc_1", new_neigh_points);
				psCloud_1->setPointRadius(0.00015);
				psCloud_1->setPointColor(glm::vec3(49.0 / 255.0, 49.0 / 255.0, 129.0 / 255.0));
				psCloud_1->addToGroup(*group);
				polyscope::PointCloud *psCloud_2 = polyscope::registerPointCloud("pc_2", origin);
				psCloud_2->setPointRadius(0.00025);
				psCloud_2->setPointColor(glm::vec3(255.0 / 255.0, 0.0 / 255.0, 0.0 / 255.0));
				psCloud_2->addToGroup(*group);
				//
				//            // In 3D space, show one-ring neighborhood and one-ring triangles
				//            polyscope::SurfaceMesh *psMesh_1 = polyscope::registerSurfaceMesh("mesh_1", gc_geom->positions, tufted_tri);
				//            psMesh_1->setEdgeWidth(2.0);
				//            psMesh_1->setTransparency(0.85);
				//            psMesh_1->addToGroup(*group);
				//            polyscope::PointCloud *psCloud_1 = polyscope::registerPointCloud("pc_1", neigh_points);
				//            psCloud_1->setPointRadius(0.0005);
				//            psCloud_1->setPointColor(glm::vec3(49.0 / 255.0, 49.0 / 255.0, 129.0 / 255.0));
				//            psCloud_1->addToGroup(*group);
				//            polyscope::PointCloud *psCloud_2 = polyscope::registerPointCloud("pc_2",
				//                                                                             std::vector<geometrycentral::Vector3>
				//                                                                                     {vertices_positions[center_index]});
				//            psCloud_2->setPointRadius(0.0005);
				//            psCloud_2->setPointColor(glm::vec3(255.0 / 255.0, 0.0 / 255.0, 0.0 / 255.0));
				//            psCloud_2->addToGroup(*group);

				// Show the GUI
				std::string myString = polyscope::view::getViewAsJson();
				polyscope::show();
			}


			void DrawTwoTangentPoints(const std::string &title,
									  const std::shared_ptr<Eigen::MatrixXd> &origianl_cloudPtr,
									  const std::shared_ptr<Eigen::MatrixXd> &contracted_cloudPtr,
									  const int k,
									  const int center_index) {
				std::shared_ptr<geometrycentral::pointcloud::PointCloud> gc_origianl_cloudPtr =
						std::make_shared<geometrycentral::pointcloud::PointCloud>(origianl_cloudPtr->rows());
				std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> gc_origianl_geom =
						std::make_shared<geometrycentral::pointcloud::PointPositionGeometry>(*gc_origianl_cloudPtr);
				std::vector<geometrycentral::Vector3> vertices_positions = tool::utility::Matrix2GCVector(origianl_cloudPtr);
				for (int i = 0; i < origianl_cloudPtr->rows(); ++i) {
					gc_origianl_geom->positions[i] = vertices_positions[i];
				}
				gc_origianl_geom->kNeighborSize = k;

				std::shared_ptr<geometrycentral::pointcloud::PointCloud> gc_contracted_cloudPtr =
						std::make_shared<geometrycentral::pointcloud::PointCloud>(contracted_cloudPtr->rows());
				std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> gc_contracted_geom =
						std::make_shared<geometrycentral::pointcloud::PointPositionGeometry>(*gc_contracted_cloudPtr);
				std::vector<geometrycentral::Vector3> contracted_vertices_positions = tool::utility::Matrix2GCVector(contracted_cloudPtr);
				for (int i = 0; i < contracted_cloudPtr->rows(); ++i) {
					gc_contracted_geom->positions[i] = contracted_vertices_positions[i];
				}
				gc_contracted_geom->kNeighborSize = k;


				// Prepare data
				gc_origianl_geom->requireNeighbors();
				gc_origianl_geom->requireTangentBasis();
				gc_origianl_geom->requireTangentCoordinates();
				std::vector<geometrycentral::pointcloud::Point> neigh = gc_origianl_geom->neighbors->neighbors[center_index];


				// Prepare data
				gc_contracted_geom->requireNeighbors();
				gc_contracted_geom->requireTangentBasis();
				gc_contracted_geom->requireTangentCoordinates();
				std::vector<geometrycentral::Vector2> tangentCoords = gc_contracted_geom->tangentCoordinates[center_index];
				std::vector<geometrycentral::Vector3> origin;
				origin.emplace_back(geometrycentral::Vector3{ 0.0, 0.0, 0.0 });


				geometrycentral::pointcloud::PointData<std::vector<std::array<geometrycentral::pointcloud::Point, 3>>> localTriPoint =
						geometrycentral::pointcloud::buildLocalTriangulations(*gc_origianl_cloudPtr, *gc_origianl_geom, true);
				std::vector<std::array<geometrycentral::pointcloud::Point, 3>> localTri = localTriPoint[center_index];
				std::vector<std::vector<size_t>> tris;
				tris.reserve(localTri.size());
				for (const std::array<geometrycentral::pointcloud::Point, 3> &tri: localTri) {
					std::vector<size_t> temp;
					temp.reserve(tri.size());
					for (const geometrycentral::pointcloud::Point &point: tri) {
						temp.emplace_back(point.getIndex());
					}
					tris.emplace_back(temp);
				}
				std::vector<geometrycentral::Vector3> new_points;
				geometrycentral::Vector3 center = gc_contracted_geom->positions[center_index];
				geometrycentral::Vector3 normal = gc_contracted_geom->normals[center_index];
				geometrycentral::Vector3 basisX = gc_contracted_geom->tangentBasis[center_index][0];
				geometrycentral::Vector3 basisY = gc_contracted_geom->tangentBasis[center_index][1];
				for (size_t iN = 0; iN < gc_contracted_geom->positions.size(); iN++) {
					geometrycentral::Vector3 vec = gc_contracted_geom->positions[iN] - center;
					vec = vec.removeComponent(normal);
					geometrycentral::Vector3 coord{ dot(basisX, vec), dot(basisY, vec), 0.0 };
					new_points.push_back(coord);
				}
				std::vector<geometrycentral::Vector3> new_neigh_points;
				new_neigh_points.reserve(neigh.size());
				for (const geometrycentral::pointcloud::Point &point: neigh) {
					new_neigh_points.emplace_back(new_points[point.getIndex()]);
				}

				// Initialize
				polyscope::init();
				// Some options
				polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;
				polyscope::options::groundPlaneHeightFactor = -0.760f;
				polyscope::options::shadowDarkness = 0.5;
				// A few camera options
				polyscope::view::setNavigateStyle(polyscope::NavigateStyle::Turntable);
				polyscope::view::setUpDir(polyscope::UpDir::ZUp);
				polyscope::view::setFrontDir(polyscope::FrontDir::XFront);
				polyscope::Group *group = polyscope::createGroup(title);

				// Register
				polyscope::SurfaceMesh *psMesh_1 = polyscope::registerSurfaceMesh("mesh_1", new_points, tris);
				psMesh_1->setEdgeWidth(2.0);
				psMesh_1->setTransparency(0.8);
				psMesh_1->addToGroup(*group);
				polyscope::PointCloud *psCloud_1 = polyscope::registerPointCloud("pc_1", new_neigh_points);
				psCloud_1->setPointRadius(0.00015);
				psCloud_1->setPointColor(glm::vec3(49.0 / 255.0, 49.0 / 255.0, 129.0 / 255.0));
				psCloud_1->addToGroup(*group);
				polyscope::PointCloud *psCloud_2 = polyscope::registerPointCloud("pc_2", origin);
				psCloud_2->setPointRadius(0.00025);
				psCloud_2->setPointColor(glm::vec3(255.0 / 255.0, 0.0 / 255.0, 0.0 / 255.0));
				psCloud_2->addToGroup(*group);

				// Show the GUI
				std::string myString = polyscope::view::getViewAsJson();
				polyscope::show();
			}
		}  // namespace visualize
	}  // namespace debug
}  // namespace tool
