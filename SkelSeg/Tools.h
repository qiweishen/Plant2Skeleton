#ifndef TOOLS_H
#define TOOLS_H


#include "Graph.h"
#include "Skeleton.h"
#include "miniply.h"
#include "utility/timer/timer.h"
#define MSH_PLY_ENCODER_ONLY
#include <easy3d/algo/point_cloud_simplification.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/kdtree/kdtree_search.h>
#include <easy3d/kdtree/kdtree_search_eth.h>
#include <geometrycentral/pointcloud/local_triangulation.h>
#include <geometrycentral/utilities/knn.h>
#include <geometrycentral/utilities/vector2.h>
#include <geometrycentral/utilities/vector3.h>
#include <memory>
#include <polyscope/point_cloud.h>
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>

#include "deps/msh_ply/msh_ply.h"
#include "deps/tinyobjloader/tiny_obj_loader.h"



namespace tool {
	namespace preprocess {
		/**
		 * @brief Normalize the point cloud to a unit sphere.
		 * @param cloudPtr The smart pointer to the point cloud.
		 * @param diagonal_length The maximum diagonal length of the normalized point cloud (the diameter of the sphere).
		 * @return A tuple, the first element is the center of the original point cloud, the second element is the normalization scaling.
		 */
		std::tuple<Eigen::Vector3d, double> Normalize(std::shared_ptr<Eigen::MatrixXd> &cloudPtr, double diagonal_length);


		/**
		 * @brief Un-normalize the point cloud.
		 * @param cloudPtr The smart pointer to the point cloud.
		 * @param center The center of the original point cloud.
		 * @param normalization_scaling The normalization scaling, can be obtained from Normalize() function.
		 */
		void UnNormalize(std::shared_ptr<Eigen::MatrixXd> &cloudPtr, const Eigen::Vector3d &center, const double &normalization_scaling);


		/**
		 * @brief Normalize the point cloud to a unit sphere.
		 * @param final_cloudPtr The smart pointer to the final prepared point cloud.
		 * @param file_path The path of the original point cloud file (support Binary, XYZ, and TXT).
		 * @param config The configuration file.
		 */
		void PreparePointCloud(std::shared_ptr<Eigen::MatrixXd> &final_cloudPtr, const std::filesystem::path &file_path, nlohmann::json &config);
	}  // namespace preprocess



	// For PLY file I/O performance, we use the "miniply" library for reading, and the "msh_ply" library for writing.
	namespace io {
		/**
		 * @brief Load point cloud from Binary/ASCII PLY, XYZ or TXT file, which **only contains vetex information**. And save a copy PLY file in the
		 * Output folder.
		 * @param config The configuration file, used to specify the path to the Binary/ASCII PLY, XYZ or TXT file and the Output folder path.
		 * @return Only returns the points in std::vector<easy3d::vec3>.
		 */
		std::vector<easy3d::vec3> LoadPointCloud(nlohmann::json &config);


		/**
		 * @brief Format Binary/ASCII PLY, XYZ or TXT point cloud file to a signle PLY file with properties, and save in the Output folder.
		 * @param config The configuration file, used to specify the path to the Binary/ASCII PLY, XYZ or TXT file and the properties to be loaded and
		 * the Output folder path.
		 */
		void FormatPointCloud(nlohmann::json &config);


		namespace internal {
			/**
			 * @brief Load point cloud from Binary/ASCII PLY file, which **only contains vetex information**.
			 * @param file_path The path to the Binary/ASCII PLY file, must end with ".ply" and the Output folder
			 * path.
			 * @return Only returns the points in std::vector<easy3d::vec3>.
			 */
			std::vector<easy3d::vec3> LoadPointCloudFromPLY(const std::filesystem::path &file_path);


			/**
			 * @brief Format Binary/ASCII PLY point cloud file to a signle PLY file with properties, and save in the Output folder.
			 * @param config The configuration file, used to specify the path to the Binary/ASCII PLY file, must end with ".ply" and the properties to
			 * be loaded and the Output folder path.
			 */
			void FormatPointCloudFromPLY(nlohmann::json &config);


			/**
			 * @brief Load point cloud from XYZ or TXT file, which **only contains vetex information**.
			 * @param file_path The path of the XYZ or TXT file, must end with ".xyz" or ".txt" and the Output
			 * folder path.
			 * @return Only returns the points in std::vector<easy3d::vec3>.
			 */
			std::vector<easy3d::vec3> LoadPointCloudFromXYZTXT(const std::filesystem::path &file_path);


			/**
			 * @brief Format XYZ or TXT point cloud file to a signle PLY file with properties, and save in the Output folder.
			 * @param config The configuration file, used to specify the path of the XYZ or TXT file, must end with ".xyz" or ".txt" and the
			 * properties to be loaded and the Output folder path.
			 */
			void FormatPointCloudFromXYZTXT(nlohmann::json &config);


			/**
			 * @brief
			 * @param file_path
			 * @param dim
			 * @return
			 */
			template<typename T>
			std::vector<T> LoadDataFromTXTXYZ(const std::filesystem::path &file_path, size_t dim);
			template std::vector<int> LoadDataFromTXTXYZ<int>(const std::filesystem::path &file_path, size_t dim);
			template std::vector<double> LoadDataFromTXTXYZ<double>(const std::filesystem::path &file_path, size_t dim);
			template std::vector<easy3d::vec3> LoadDataFromTXTXYZ<easy3d::vec3>(const std::filesystem::path &file_path, size_t dim);
			template std::vector<Eigen::Vector3d> LoadDataFromTXTXYZ<Eigen::Vector3d>(const std::filesystem::path &file_path, size_t dim);
			template std::vector<Eigen::VectorXi> LoadDataFromTXTXYZ<Eigen::VectorXi>(const std::filesystem::path &file_path, size_t dim);
			template std::vector<Eigen::VectorXd> LoadDataFromTXTXYZ<Eigen::VectorXd>(const std::filesystem::path &file_path, size_t dim);
			template<typename T>
			std::vector<T> parse1DData(const char *data, std::streamsize bytes_read, const std::filesystem::path &file_path);
			template<typename T>
			std::vector<T> parseNDData(const char *data, std::streamsize bytes_read, size_t dim, const std::filesystem::path &file_path);


			namespace under_construction {
				/**
				 * @brief Load point cloud from OBJ file.
				 * @param file_path The path of the OBJ file.
				 * @return The point cloud.
				 */
				Eigen::MatrixXd LoadPointCloudFromOBJ(const std::filesystem::path &file_path);
			}  // namespace under_construction
		}  // namespace internal


		/**
		 * @brief Save point cloud to Binary file.
		 * @param file_path The path of the Binary file, must end with ".ply".
		 * @param cloudPtr The smart pointer to the point cloud.
		 * @param binary Whether to save the Binary file in binary format, true for binary, false for ASCII. Default is binary format.
		 */
		void SavePointCloudToPLY(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, const std::filesystem::path &file_path, bool binary = true);

		/**
		 * @brief Save point cloud to Binary file.
		 * @param file_path The path of the Binary file, must end with ".ply".
		 * @param cloudPtr The smart pointer to the point cloud.
		 * @param property The property of the points.
		 * @param property_name The name of the property.
		 * @param binary Whether to save the Binary file in binary format, true for binary, false for ASCII. Default is binary format.
		 */
		void SavePointCloudToPLY(std::shared_ptr<Eigen::MatrixXd> &cloudPtr,
								 const std::filesystem::path &file_path,
								 const std::vector<double> &property,
								 const std::string &property_name,
								 bool binary = true);

		/**
		 * @brief Save point cloud to Binary file.
		 * @param file_path The path of the Binary file, must end with ".ply".
		 * @param cloudPtr The smart pointer to the point cloud.
		 * @param label_one The first labels of the points.
		 * @param label_two The second labels of the points.
		 * @param label_names The names of the first and second labels.
		 * @param binary Whether to save the Binary file in binary format, true for binary, false for ASCII. Default is binary format.
		 */
		void SavePointCloudToPLY(std::shared_ptr<Eigen::MatrixXd> &cloudPtr,
								 const std::filesystem::path &file_path,
								 const std::vector<int> &label_one,
								 const std::vector<int> &label_two,
								 const std::pair<std::string, std::string> &label_names,
								 bool binary = true);


		/**
		 * @brief Save the skeleton graph to Binary file.
		 * @param file_path The path of the Binary file, must end with ".ply".
		 * @param graphPtr The smart pointer to the graph object (boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Eigen::Vector3d,
		 * Boost_EdgeWeightProperty>).
		 * @param binary Whether to save the Binary file in binary format, true for binary, false for ASCII. Default is binary format.
		 */
		void SaveSkeletonGraphToPLY(const std::shared_ptr<Boost_Graph> &graphPtr, const std::filesystem::path &file_path, bool binary = true);

		/**
		 * @brief Save the skeleton graph to Binary file.
		 * @param file_path The path of the Binary file, must end with ".ply".
		 * @param graphPtr The smart pointer to the graph object (boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Eigen::Vector3d,
		 * Boost_EdgeWeightProperty>).
		 * @param vertex_labels The labels of the vertices.
		 * @param binary Whether to save the Binary file in binary format, true for binary, false for ASCII. Default is binary format.
		 */
		void SaveSkeletonGraphToPLY(const std::shared_ptr<Boost_Graph> &graphPtr,
									const std::filesystem::path &file_path,
									const std::vector<int> &vertex_labels,
									bool binary = true);


		/**
		 * @brief Save the configuration which used for current results to JSON file.
		 * @param config The configuration.
		 * @param file_path The path of the JSON file.
		 */
		void SaveJSONFile(const nlohmann::json &config, const std::filesystem::path &file_path);
	}  // namespace io



	namespace utility {
		/**
		 * @brief Transfer std::vector<easy3d::vec3>/std::vector<Eigen::Vector3d> point cloud to std::shared_ptr<Eigen::MatrixXd> point cloud.
		 * @param cloud_verts The smart pointer to the <Eigen::MatrixXd> point cloud.
		 * @return cloudPtr The smart pointer to the <Eigen::MatrixXd> point cloud.
		 */
		template<typename T>
		std::shared_ptr<Eigen::MatrixXd> Vector2Matrix(const std::vector<T> &cloud_verts);


		/**
		 * @brief Transfer <Eigen::MatrixXd> point cloud to geometry-central accepted format.
		 * @param cloudPtr The smart pointer to the <Eigen::MatrixXd> point cloud.
		 */
		std::vector<geometrycentral::Vector3> Matrix2GCVector(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr);


		/**
		 * @brief Generate the k-nearest-neighbors for each point in the &cloudPtr, the query point is _not_ included.
		 * @param cloudPtr The smart pointer to the <Eigen::MatrixXd> point cloud.
		 * @param k The number of neighbors to search.
		 */
		std::vector<std::vector<size_t>> KNNSearch(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, size_t k);


		/**
		 * @brief Generate all the neighbors within ball, the center point is included.
		 * @param cloudPtr The smart pointer to the point cloud.
		 * @param radius The radius of the ball.
		 */
		std::vector<std::vector<size_t>> RadiusSearch(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, const double radius);


		/**
		 * @brief Compute the covariance matrix of the point cloud.
		 * @param cloud The point cloud.
		 */
		Eigen::MatrixXd ComputeCovariance(const Eigen::MatrixXd &cloud);


		/**
		 * @brief Find the upper outliers by the standard deviation method.
		 * @param data The data to be processed.
		 */
		std::vector<int> FindUpperOutlierBySTD(const std::vector<double> &data);


		/**
		 * @brief Down-sample the point cloud by voxel grid.
		 * @param cloudPtr The smart pointer to the point cloud.
		 * @param voxel_size The size of the voxel.
		 * @return The smart pointer to the down-sampled point cloud.
		 */
		std::shared_ptr<Eigen::MatrixXd> VoxelDownSample(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, double voxel_size);


		/**
		 * @brief Down-sample the point cloud by farthest point sampling.
		 * @param cloudPtr The smart pointer to the point cloud.
		 * @param num_samples The number of samples to be selected.
		 * @return The selected indices.
		 */
		std::vector<size_t> FarthestPointDownSample(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, int num_samples);


		/**
		 * @brief Generate a point cloud from the indices.
		 * @param cloudPtr The smart pointer to the point cloud.
		 * @param indices The indices of the points to be selected.
		 * @return The point cloud containing the selected points.
		 */
		Eigen::MatrixXd SelectByIndices(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, const std::vector<size_t> &indices, const int &dim);


		/**
		 * @brief Assign classes to the points in the original point cloud according to their the nearest points in the captured skeleton graph.
		 * @param classes The classes of the points in the skeleton graph.
		 * @param graphPtr The smart pointer to the skeleton graph.
		 * @param cloudPtr The smart pointer to the original point cloud.
		 */
		std::vector<std::vector<int>> NearestProjectFromBoostVerts(std::vector<std::vector<Boost_Vertex>> &classes,
																   std::shared_ptr<Boost_Graph> &graphPtr,
																   std::shared_ptr<Eigen::MatrixXd> &cloudPtr);
	}  // namespace utility



	namespace debug {
		/**
		 * @brief Save a sphere to Binary file, for debugging.
		 */
		void SaveSphereToPLY(const Eigen::Vector3d &center, double radius, const std::string &filepath);


		namespace visualize {
			/**
			 * @brief Draw the point clouds, for debugging.
			 */
			void DrawPointClouds(const std::string &group_title,
								 const std::vector<std::pair<std::string, std::shared_ptr<Eigen::MatrixXd>>> &cloudsPairs);


			/**
			 * @brief Draw the mesh, for debugging.
			 */
			void DrawUnionLocalTriangles(const std::string &title,
										 const std::shared_ptr<geometrycentral::pointcloud::PointCloud> &gc_cloudPtr,
										 const std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> &gc_geom);


			/**
			 * @brief Draw the mesh, for debugging.
			 */
			void DrawTuftedMesh(const std::string &title, const std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> &gc_geom);


			/**
			 * @brief Draw the point clouds, for debugging.
			 */
			void DrawTangentPoints(const std::string &title, const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, int k, int center_index);


			void DrawTwoTangentPoints(const std::string &title,
									  const std::shared_ptr<Eigen::MatrixXd> &origianl_cloudPtr,
									  const std::shared_ptr<Eigen::MatrixXd> &contracted_cloudPtr,
									  const int k,
									  const int center_index);
		}  // namespace visualize
	}  // namespace debug
}  // namespace tool



#endif	// TOOLS_H
