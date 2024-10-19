#ifndef TOOLS_H
#define TOOLS_H


#include <KDTree.hpp>
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
#include <plywoot.hpp>
#include <utility/timer.h>

#include "graph.h"



inline bool isPlyFormatBinary;	// Whether to save the PLY file in binary format, true for binary, false for ASCII. Default is binary format.

namespace tool {
	namespace preprocess {
		/**
		 * @brief Normalize the point cloud to a unit sphere.
		 * @param final_cloud The point cloud.
		 * @param config The configuration file.
		 */
		void PreparePointCloud(Eigen::MatrixXd &final_cloud, nlohmann::json &config);


		namespace internal {
			/**
			 * @brief Uniformly downsample the point cloud (easy3d::PointCloud) to the given point number.
			 * @param cloud: The point cloud object (easy3d::PointCloud).
			 * @param num_samples: The expected point number, which must be less than or equal to the original point number.
			 */
			void UniformDownSample(easy3d::PointCloud &cloud, unsigned int num_samples);
		}  // namespace internal
	}  // namespace preprocess



	namespace io {
		/**
		 * @brief Load point cloud from Binary/ASCII PLY, XYZ or TXT file, which **only contains vertex information**. And save a copy PLY file in the
		 * Output folder.
		 * @param config The configuration file, used to specify the path to the Binary/ASCII PLY, XYZ or TXT file and the Output folder path.
		 * @return Only returns the points in std::vector<easy3d::vec3>.
		 */
		std::vector<easy3d::vec3> LoadPointCloud(nlohmann::json &config);


		/**
		 * @brief Format Binary/ASCII PLY, XYZ or TXT point cloud file to a single PLY file with properties, and save in the Output folder.
		 * @param config The configuration file, used to specify the path to the Binary/ASCII PLY, XYZ or TXT file and the properties to be loaded and
		 * the Output folder path.
		 */
		void FormatPointCloud(nlohmann::json &config);


		/**
		 * @brief Save point cloud to PLY file, which **only contains vertex information**.
		 * @param file_path The path to the PLY file, must end with ".ply".
		 * @param cloud The point cloud object (Eigen::MatrixXd).
		 */
		void SavePointCloudToPLY(const Eigen::MatrixXd &cloud, const std::filesystem::path &file_path);

		/**
		 * @brief Save point cloud to PLY file, which can contain one property (double type) of the points.
		 * @param file_path The path to the PLY file, must end with ".ply".
		 * @param cloud The point cloud object (Eigen::MatrixXd).
		 * @param property The property of the points (std::vector<double>).
		 * @param property_name The name of the property.
		 */
		void SavePointCloudToPLY(const Eigen::MatrixXd &cloud, const std::filesystem::path &file_path, const std::vector<double> &property,
								 const std::string &property_name);

		/**
		 * @brief Save point cloud to PLY file, which can contain two label properties (int type) of the points.
		 * @param file_path The path to the PLY file, must end with ".ply".
		 * @param cloud The point cloud object (Eigen::MatrixXd).
		 * @param label_one The first label of the points (std::vector<int>).
		 * @param label_two The second label of the points (std::vector<int>).
		 * @param label_names The names of the first and second labels (std::pair<std::string, std::string>).
		 */
		void SavePointCloudToPLY(const Eigen::MatrixXd &cloud, const std::filesystem::path &file_path, const std::vector<int> &label_one,
								 const std::vector<int> &label_two, const std::pair<std::string, std::string> &label_names);


		/**
		 * @brief Save the skeleton graph to PLY file, which **only contains vertex and edge information**.
		 * @param graph The graph object (boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Eigen::Vector3d,
		 * Boost_EdgeWeightProperty>).
		 * @param file_path The path to the PLY file, must end with ".ply".
		 */
		void SaveSkeletonGraphToPLY(const Boost_Graph &graph, const std::filesystem::path &file_path);

		/**
		 * @brief Save the skeleton graph to Binary file, which will derive the semantic labels of the vertices and the semantic/instance labels of
		 * the edges from the input `vertex_instance_labels`.
		 * @param graph The graph object (boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Eigen::Vector3d,
		 * Boost_EdgeWeightProperty>).
		 * @param file_path The path to the PLY file, must end with ".ply".
		 * @param vertex_semantic_labels The `semantic labels` of the skeleton vertices (std::vector<int>).
		 * @param vertex_instance_labels The `instance labels` of the skeleton vertices (std::vector<int>).
		 */
		void SaveSkeletonGraphToPLY(const Boost_Graph &graph, const std::filesystem::path &file_path, const std::vector<int> &vertex_semantic_labels,
									const std::vector<int> &vertex_instance_labels);


		/**
		 * @brief Save the configuration which used for current results to JSON file.
		 * @param config The configuration.
		 * @param file_path The path to the archive JSON file.
		 */
		void SaveJSONFile(const nlohmann::json &config, const std::filesystem::path &file_path);


		namespace internal {
			/**
			 * @brief Load point cloud from Binary/ASCII PLY file, which **only contains vertex information**.
			 * @param file_path The path to the Binary/ASCII PLY file, must end with ".ply" and the Output folder path.
			 * @return Only returns the points in std::vector<easy3d::vec3>.
			 */
			std::vector<easy3d::vec3> LoadPointCloudFromPLY(const std::filesystem::path &file_path);


			/**
			 * @brief Format Binary/ASCII PLY point cloud file to a single PLY file with properties, and save in the Output folder.
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
			 * @brief Format XYZ or TXT point cloud file to a single PLY file with properties, and save in the Output folder.
			 * @param config The configuration file, used to specify the path of the XYZ or TXT file, must end with ".xyz" or ".txt" and the
			 * properties to be loaded and the Output folder path.
			 */
			void FormatPointCloudFromXYZTXT(nlohmann::json &config);


			/**
			 * @brief Load data from Binary/ASCII XYZ/TXT file.
			 * @param file_path
			 * @param dim
			 * @param delimiter
			 * @return
			 */
			template<typename T>
			std::vector<T> LoadDataFromTXTXYZ(const std::filesystem::path &file_path, size_t dim, char delimiter = ' ');
			template<typename T>
			std::vector<T> ParseNDData(const std::string &data, size_t dim, char delimiter);


			struct DataPoint {
				Eigen::Vector3d pos;
				int label1;
				int label2;
				DataPoint() : pos(Eigen::Vector3d::Zero()), label1(-2), label2(-2) {}
				DataPoint(Eigen::Vector3d vec, int l1, int l2) : pos(std::move(vec)), label1(l1), label2(l2) {}
			};
			struct DataEdge {
				Eigen::Vector2i idx;
				int label1;
				int label2;
				DataEdge() : idx(Eigen::Vector2i::Zero()), label1(-2), label2(-2) {}
				DataEdge(Eigen::Vector2i vec, int l1, int l2) : idx(std::move(vec)), label1(l1), label2(l2) {}
			};
		}  // namespace internal
	}  // namespace io



	namespace utility {
		/**
		 * @brief Normalize the point cloud to a unit sphere.
		 * @param cloud The point cloud.
		 * @param diagonal_length The maximum diagonal length of the normalized point cloud (the diameter of the sphere).
		 * @return A tuple, the first element is the center of the original point cloud, the second element is the normalization scaling.
		 */
		std::tuple<Eigen::Vector3d, double> Normalize(Eigen::MatrixXd &cloud, double diagonal_length);


		/**
		 * @brief Un-normalize the point cloud.
		 * @param cloud The point cloud.
		 * @param center The center of the original point cloud.
		 * @param normalization_scaling The normalization scaling, can be obtained from Normalize() function.
		 */
		void UnNormalize(Eigen::MatrixXd &cloud, const Eigen::Vector3d &center, const double &normalization_scaling);


		/**
		 * @brief Transfer `std::vector<easy3d::vec3>` / `std::vector<Eigen::Vector3d>` / `Eigen::VectorXd` point cloud to `Eigen::MatrixXd` point
		 * cloud.
		 * @param vec The `std::vector<easy3d::vec3>` / `std::vector<Eigen::Vector3d>` object.
		 * @return The `Eigen::MatrixXd` object.
		 */
		template<typename T>
		Eigen::MatrixXd Vector2Matrix(const std::vector<T> &vec);
		Eigen::MatrixXd Vector2Matrix(const std::vector<easy3d::vec3> &vec);


		/**
		 * @brief Transfer `Eigen::MatrixXd` point cloud to `std::vector<easy3d::vec3>` / `std::vector<Eigen::Vector3d>` /
		 * `std::vector<geometrycentral::Vector3>` point cloud.
		 * @param mat The `Eigen::MatrixXd` object.
		 * @return The `std::vector<easy3d::vec3>` / `std::vector<Eigen::Vector3d>` / `std::vector<geometrycentral::Vector3>` object.
		 */
		template<typename T>
		std::vector<T> Matrix2Vector(const Eigen::MatrixXd &mat);


		/**
		 * @brief Generate the k-nearest-neighbors for each point in the &cloudPtr, the query point is _not_ included.
		 * @param cloud The point cloud.
		 * @param k The number of neighbors to search.
		 * @return
		 */
		std::vector<std::vector<size_t>> KNNSearch(const Eigen::MatrixXd &cloud, size_t k);
		std::vector<std::vector<size_t>> SKNNSearch(const Eigen::MatrixXd &cloud, const size_t k);


		/**
		 * @brief Generate all the neighbors within ball, the center point is _included_.
		 * @param cloud The point cloud.
		 * @param radius The radius of the ball.
		 */
		std::vector<std::vector<size_t>> RadiusSearch(const Eigen::MatrixXd &cloud, double radius);
		std::vector<std::vector<size_t>> SRadiusSearch(const Eigen::MatrixXd &cloud, const double radius);


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
		 * @brief Downsample the point cloud (Eigen::MatrixXd) to the given point number by farthest point sampling.
		 * @param cloud The point cloud object (Eigen::MatrixXd).
		 * @param num_samples The expected point number, which must be less than or equal to the original point number.
		 * @return The sampled point cloud object and the indices of the sampled points.
		 */
		std::tuple<Eigen::MatrixXd, std::vector<size_t>> FarthestPointDownSample(const Eigen::MatrixXd &cloud, size_t num_samples);


		/**
		 * @brief Generate a point cloud from the indices.
		 * @param cloud The point cloud.
		 * @param indices The indices of the points to be selected.
		 * @param dim The dimension of the point cloud.
		 * @return The point cloud containing the selected points.
		 */
		Eigen::MatrixXd SelectByIndices(const Eigen::MatrixXd &cloud, const std::vector<size_t> &indices, const int &dim);


		/**
		 * @brief Assign classes to the points in the original point cloud according to their nearest points in the captured skeleton graph.
		 * @param graph The graph object (boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Eigen::Vector3d,
		 * Boost_EdgeWeightProperty>).
		 * @param cloud The point cloud.
		 * @param reference_labels The labels of the reference points/edges in the skeleton graph.
		 * @param reference The reference type, can be one of:
		 *   - `"vertex"`
		 *   - `"edge"`
		 */
		std::vector<int> NearestProjectFromBoostVertices(const Boost_Graph &graph, const Eigen::MatrixXd &cloud,
														 const std::vector<int> &reference_labels, const std::string &reference);
	}  // namespace utility



	namespace debug {
		/**
		 * @brief Compute the average number of non-zero values per row in the sparse matrix.
		 * @param mat The sparse matrix (Eigen::SparseMatrix<double>).
		 * @return The average number of non-zero values per row.
		 */
		double AverageNoneZerosPerRow(const Eigen::SparseMatrix<double> &mat);


		/**
		 * @brief Save a sphere to a ASCII PLY file, for debugging reference.
		 * @param center The center of the sphere.
		 * @param radius The radius of the sphere.
		 * @param filepath The path to the ASCII PLY file, must end with ".ply".
		 */
		void SaveSphereToPLY(const Eigen::Vector3d &center, double radius, const std::filesystem::path &filepath);


		//	 	namespace visualize {
		//	 		/**
		//	 		 * @brief Draw the point clouds, for debugging.
		//	 		 */
		//	 		void DrawPointClouds(const std::string &group_title,
		//	 							 const std::vector<std::pair<std::string, std::shared_ptr<Eigen::MatrixXd>>> &cloudsPairs);
		//
		//
		//	 		/**
		//	 		 * @brief Draw the mesh, for debugging.
		//	 		 */
		//	 		void DrawUnionLocalTriangles(const std::string &title,
		//	 									 const std::shared_ptr<geometrycentral::pointcloud::PointCloud> &gc_cloudPtr,
		//	 									 const std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> &gc_geom);
		//
		//
		//	 		/**
		//	 		 * @brief Draw the mesh, for debugging.
		//	 		 */
		//	 		void DrawTuftedMesh(const std::string &title, const std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> &gc_geom);
		//
		//
		//	 		/**
		//	 		 * @brief Draw the point clouds, for debugging.
		//	 		 */
		//	 		void DrawTangentPoints(const std::string &title, const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, int k, int center_index);
		//
		//
		//	 		void DrawTwoTangentPoints(const std::string &title,
		//	 								  const std::shared_ptr<Eigen::MatrixXd> &origianl_cloudPtr,
		//	 								  const std::shared_ptr<Eigen::MatrixXd> &contracted_cloudPtr,
		//	 								  int k,
		//	 								  int center_index);
		//	 	}  // namespace visualize
	}  // namespace debug
}  // namespace tool



#endif	// TOOLS_H
