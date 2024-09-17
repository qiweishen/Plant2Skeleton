#ifndef CPPTEST_TOOLS_H
#define CPPTEST_TOOLS_H


#include "Skeleton.h"
#include "Graph.h"

#include "geometrycentral/utilities/vector2.h"
#include "geometrycentral/utilities/vector3.h"
#include "geometrycentral/utilities/knn.h"
#include "geometrycentral/pointcloud/local_triangulation.h"

#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"

#include <memory>


namespace tool {
    namespace preprocess {
        /**
         * @brief Normalize the point cloud to a unit sphere.
         * @param cloudPtr The smart pointer to the point cloud.
         * @param diagonal_length The maximum diagonal length of the normalized point cloud (the diameter of the sphere).
         * @return A tuple, the first element is the center of the point cloud, the second element is the normalization scaling.
         */
        std::tuple<Eigen::Vector3d, double>
        Normalize(std::shared_ptr<Eigen::MatrixXd> &cloudPtr, double diagonal_length);


        /**
         * @brief Un-normalize the point cloud.
         * @param cloudPtr The smart pointer to the point cloud.
         * @param center The center of the original point cloud.
         * @param normalization_scaling The normalization scaling, can be obtained from Normalize() function.
         */
        void UnNormalize(std::shared_ptr<Eigen::MatrixXd> &cloudPtr, const Eigen::Vector3d &center,
                         const double &normalization_scaling);
    }


    // Wil consider to be replaced by hapPLY (https://github.com/nmwsharp/happly)
    namespace io {
        /**
         * @brief Load point cloud from PLY file.
         * @param file_path The path of the PLY file, must end with ".ply".
         * @param cloudPtr The smart pointer to the point cloud.
         * @param propertyPtr The pointer to the property matrix, the shape is (n, m), n is the number of points,
         * m is the number of properties.
         * @param property_names A vector of pairs. Stores the type and name of each property,
         * in the pair, the first element is the type, the second element is the name.
         */
        void LoadPointCloudFromPLY(const std::string &file_path, std::shared_ptr<Eigen::MatrixXd> &cloudPtr,
                                   std::shared_ptr<Eigen::MatrixXd> &propertyPtr,
                                   std::vector<std::pair<std::string, std::string>> &property_names);


        /**
         * @brief Load point cloud from XYZ file, currently only support XYZ file with 3D points, no properties.
         * @param file_path The path of the XYZ file, must end with ".xyz".
         * @param cloudPtr The smart pointer to the point cloud.
         */
        void LoadPointCloudFromXYZ(const std::string &file_path, std::shared_ptr<Eigen::MatrixXd> &cloudPtr);


        /**
         * @brief Save point cloud to PLY file.
         * @param file_path The path of the PLY file, must end with ".ply".
         * @param cloudPtr The smart pointer to the point cloud.
         * @param propertyPtr The pointer to the property matrix, the shape is (n, m), n is the number of points,
         * m is the number of properties.
         * @param property_names The type and name of each property,
         * in the pair, the first element is the type, the second element is the name.
         */
        void SavePointCloudToPLY(const std::string &file_path, const std::shared_ptr<Eigen::MatrixXd> &cloudPtr,
                                 const std::shared_ptr<Eigen::MatrixXd> &propertyPtr,
                                 const std::vector<std::pair<std::string, std::string>> &property_names);


        /**
         * @brief Save graph to PLY file.
         * @param file_path The path of the XYZ file, must end with ".xyz".
         * @param graphPtr The smart pointer to the graph object
         * (boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Eigen::Vector3d, Boost_EdgeWeightProperty>).
         */
        void SaveGraphToPLY(const std::string &file_path, const std::shared_ptr<Boost_Graph> &graphPtr);
    }


    namespace utility {
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
        std::vector<std::vector<size_t>> KNNSearch(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, const size_t k);


        /**
         * @brief Generate all the neighbors within ball, the center point is included.
         * @param cloudPtr The smart pointer to the point cloud.
         * @param radius The radius of the ball.
         */
        std::vector<std::vector<size_t>>
        RadiusSearch(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, const double radius);


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
        std::shared_ptr<Eigen::MatrixXd>
        VoxelDownSample(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, double voxel_size);


        /**
         * @brief Down-sample the point cloud randomly.
         * @param cloudPtr The smart pointer to the point cloud.
         * @param voxel_size The size of the voxel.
         * @return The selected indices.
         */
        std::vector<size_t> RandomDownSample(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, int num_samples);


        /**
         * @brief Down-sample the point cloud by farthest point sampling.
         * @param cloudPtr The smart pointer to the point cloud.
         * @param voxel_size The size of the voxel.
         * @return The selected indices.
         */
        std::vector<size_t> FarthestPointDownSample(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, int num_samples);


        /**
         * @brief Generate a point cloud from the indices.
         * @param cloudPtr The smart pointer to the point cloud.
         * @param indices The indices of the points to be selected.
         * @return The point cloud containing the selected points.
         */
        Eigen::MatrixXd
        SelectByIndices(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, const std::vector<size_t> &indices,
                        const int &dim);


        /**
         * @brief Assign classes to the points in the original point cloud according to their the nearest points in the captured skeleton graph.
         * @param classes The classes of the points in the skeleton graph.
         * @param graphPtr The smart pointer to the skeleton graph.
         * @param cloudPtr The smart pointer to the original point cloud.
         */
        std::vector<std::vector<int>> NearestProjectFromBoostVerts(std::vector<std::vector<Boost_Vertex>> &classes,
                                                                   std::shared_ptr<Boost_Graph> &graphPtr,
                                                                   std::shared_ptr<Eigen::MatrixXd> &cloudPtr);
    }


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
        void DrawTuftedMesh(const std::string &title,
                            const std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> &gc_geom);


        /**
         * @brief Draw the point clouds, for debugging.
         */
        void DrawTangentPoints(const std::string &title, const std::shared_ptr<Eigen::MatrixXd> &cloudPtr,
                               int k, int center_index);


        void DrawTwoTangentPoints(const std::string &title, const std::shared_ptr<Eigen::MatrixXd> &origianl_cloudPtr,
                                  const std::shared_ptr<Eigen::MatrixXd> &contracted_cloudPtr,
                                  const int k, const int center_index);
    }


    namespace debug {
        /**
         * @brief Save a sphere to PLY file, for debugging.
         */
        void SaveSphereToPLY(const Eigen::Vector3d &center, double radius, const std::string &filepath);
    }
}


#endif //CPPTEST_TOOLS_H
