#ifndef CPPTEST_TOOLS_H
#define CPPTEST_TOOLS_H


#include "Skeleton_pure.h"

#include <cstdint>





namespace tool {
    namespace preprocess {
        /**
         * @brief Normalize the point cloud to a unit sphere.
         * @tparam T The type of the point cloud, can be Eigen::MatrixXd or open3d::geometry::PointCloud.
         * @param cloudPtr The smart pointer to the point cloud.
         * @param diagonal_length The maximum diagonal length of the normalized point cloud (the diameter of the sphere).
         * @return A tuple, the first element is the center of the point cloud, the second element is the normalization scaling.
         */
        template<typename T>
        std::tuple<Eigen::Vector3d, double> Normalize(std::shared_ptr<T>& cloudPtr, double diagonal_length);


        /**
         * @brief Un-normalize the point cloud.
         * @tparam T The type of the point cloud, can be Eigen::MatrixXd or open3d::geometry::PointCloud.
         * @param cloudPtr The smart pointer to the point cloud.
         * @param center The center of the original point cloud.
         * @param normalization_scaling The normalization scaling, can be obtained from Normalize() function.
         */
        template<typename T>
        void UnNormalize(std::shared_ptr<T>& cloudPtr, const Eigen::Vector3d& center, const double& normalization_scaling);
    }



    namespace io {
        /**
         * @brief Load point cloud from PLY file.
         * @tparam T The type of the point cloud, can be Eigen::MatrixXd or open3d::geometry::PointCloud.
         * @param file_path The path of the PLY file, must end with ".ply".
         * @param cloudPtr The smart pointer to the point cloud.
         * @param propertyPtr The pointer to the property matrix, the shape is (n, m), n is the number of points,
         * m is the number of properties.
         * @param property_names A vector of pairs. Stores the type and name of each property,
         * in the pair, the first element is the type, the second element is the name.
         */
        template<typename T>
        void LoadPointCloudFromPLY(const std::string& file_path, std::shared_ptr<T>& cloudPtr,
                                   std::shared_ptr<Eigen::MatrixXd>& propertyPtr,
                                   std::vector<std::pair<std::string, std::string>>& property_names);


        /**
         * @brief Save point cloud to PLY file.
         * @tparam T The type of the point cloud, can be Eigen::MatrixXd or open3d::geometry::PointCloud.
         * @param file_path The path of the PLY file, must end with ".ply".
         * @param cloudPtr The smart pointer to the point cloud.
         * @param propertyPtr The pointer to the property matrix, the shape is (n, m), n is the number of points,
         * m is the number of properties.
         * @param property_names The type and name of each property,
         * in the pair, the first element is the type, the second element is the name.
         */
        template<typename T>
        void SavePointCloudToPLY(const std::string& file_path, const std::shared_ptr<T>& cloudPtr,
                                 const std::shared_ptr<Eigen::MatrixXd>& propertyPtr,
                                 const std::vector<std::pair<std::string, std::string>>& property_names);
    }



    namespace utility {
        /**
         * @brief Down-sample the point cloud.
         * @tparam T The type of the point cloud, can be Eigen::MatrixXd or open3d::geometry::PointCloud.
         * @param cloudPtr The smart pointer to the point cloud.
         * @param voxel_size The size of the voxel.
         * @return The smart pointer to the down-sampled point cloud.
         */
        template<typename T>
        std::shared_ptr<T> VoxelDownSample(std::shared_ptr<T> &cloudPtr, double voxel_size);


        /**
         * @brief Find the indices of the points in the sampled point cloud in the original point cloud.
         * @param cloudPtr The smart pointer to the original point cloud.
         * @param sampled_cloudPtr The smart pointer to the sampled point cloud.
         * @return A vector of indices.
         */
        std::vector<int> FindIndices(std::shared_ptr<PointCloud> &cloudPtr, std::shared_ptr<PointCloud> &sampled_cloudPtr);


        /**
         * @brief Select points from the point cloud by indices.
         * @tparam T The type of the indices, can be int or size_t.
         * @param cloudPtr The smart pointer to the point cloud.
         * @param indices The indices of the points to be selected.
         * @return The smart pointer to the selected point cloud.
         */
        template<typename T>
        std::shared_ptr<PointCloud> SelectByIndices(const std::shared_ptr<PointCloud> &cloudPtr, const std::vector<T> &indices);
    }



    namespace debug {
        void LoadPointCloudFromXYZ(const std::string &file_path, std::shared_ptr<PointCloud> &cloudPtr);
        void SaveSphereToPLY(const Eigen::Vector3d& center, double radius, const std::string& filepath);
    }
}





#endif //CPPTEST_TOOLS_H
