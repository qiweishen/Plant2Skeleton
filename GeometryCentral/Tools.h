#ifndef CPPTEST_TOOLS_H
#define CPPTEST_TOOLS_H


#include "Skeleton.h"
#include "Graph.h"

#include "geometrycentral/utilities/vector3.h"
#include "geometrycentral/utilities/knn.h"
#include "geometrycentral/pointcloud/local_triangulation.h"

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
         TODO: Add description, and load property matrix.
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
         TODO: Add description
         */
        std::vector<geometrycentral::Vector3> Matrix2GCVector(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr);


        /**
         * @brief Generate the k-nearest-neighbors for the points, the center point is _not_ included.
         * @param cloudPtr The smart pointer to the point cloud.
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
         TODO: Add description
         */
        Eigen::MatrixXd ComputeCovariance(const Eigen::MatrixXd &cloud);


        /**
         * @brief Down-sample the point cloud.
         * @tparam T The type of the point cloud, can be Eigen::MatrixXd or open3d::geometry::o3d_PointCloud.
         * @param cloudPtr The smart pointer to the point cloud.
         * @param voxel_size The size of the voxel.
         * @return The smart pointer to the down-sampled point cloud.
         */
        template<typename T>
        std::shared_ptr<T> VoxelDownSample(std::shared_ptr<T> &cloudPtr, double voxel_size);


//        /**
//         * @brief Find the indices of the points in the sampled point cloud in the original point cloud.
//         * @param cloudPtr The smart pointer to the original point cloud.
//         * @param sampled_cloudPtr The smart pointer to the sampled point cloud.
//         * @return A vector of indices.
//         */
//        std::vector<int> FindIndices(std::shared_ptr<o3d_PointCloud> &cloudPtr, std::shared_ptr<o3d_PointCloud> &sampled_cloudPtr);


        /**
         * @brief Generate a point cloud from the indices.
         * @param cloudPtr The smart pointer to the point cloud.
         * @param indices The indices of the points to be selected.
         * @return The point cloud containing the selected points.
         */
        Eigen::MatrixXd
        SelectByIndices(const std::shared_ptr<Eigen::MatrixXd> &cloudPtr, const std::vector<size_t> &indices);
    }


    namespace segment {
        /**
         TODO: Add description
         */
        std::vector<std::vector<int>> SegmentBranches(std::vector<std::vector<Boost_Vertex>> &classes,
                                                      std::shared_ptr<Boost_Graph> &graphPtr,
                                                      std::shared_ptr<Eigen::MatrixXd> &cloudPtr);
    }


    namespace visualize {
        /**
         TODO: Add description
         */
        void DrawPointClouds(const std::string &group_title,
                             const std::vector<std::pair<std::string, std::shared_ptr<Eigen::MatrixXd>>> &cloudsPairs);


        /**
         TODO: Add description
         */
        void DrawUnionLocalTriangles(const std::string &title,
                                     const std::shared_ptr<geometrycentral::pointcloud::PointCloud> &gc_cloudPtr,
                                     const std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> &gc_geom);


        /**
         TODO: Add description
         */
        void DrawTuftedMesh(const std::string &title,
                            const std::shared_ptr<geometrycentral::pointcloud::PointPositionGeometry> &gc_geom);
    }


    namespace debug {
        /**
         TODO: Add description
         */
        void SaveSphereToPLY(const Eigen::Vector3d &center, double radius, const std::string &filepath);
    }
}


#endif //CPPTEST_TOOLS_H
