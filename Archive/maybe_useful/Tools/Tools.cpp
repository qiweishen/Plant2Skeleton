#include "Tools.h"


py::array_t<int> get_indices_from_coordinates(const py::array_t<double>& cloud_in, const py::array_t<double>& cloud_ref) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_refPtr(new pcl::PointCloud<pcl::PointXYZ>);
    numpy2cloud(cloud_in, cloud_inPtr);
    numpy2cloud(cloud_ref, cloud_refPtr);

    pcl::Indices indices;
    pcl::getApproximateIndices<pcl::PointXYZ>(cloud_inPtr, cloud_refPtr, indices);

    return vector2numpy(indices);
}
