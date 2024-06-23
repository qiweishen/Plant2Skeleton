#ifndef SURFACE_BOUNDARY_FILTER_SURFACEBOUNDARYFILTER_H
#define SURFACE_BOUNDARY_FILTER_SURFACEBOUNDARYFILTER_H

#include "../../../Pybind/Pybind.h"

void numpy2cloud(const py::array_t<double>& cloud, std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& cloudPtr);
py::array_t<double> cloud2numpy(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPtr);
std::vector<int> numpy2vector(const py::array_t<int>& array);
py::array_t<int> vector2numpy(const std::vector<int>& vec);
void knnSearch(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPtr, const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr& kdtreePtr, const pcl::PointXYZ& searchPoint, std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& k_cloudPtr, int neighbors);
void project2PCA(const pcl::PointCloud<pcl::PointXYZ>::Ptr& k_cloudPtr, pcl::PointCloud<pcl::PointXY>::Ptr& projected_cloudPtr);
std::vector<double> getAngles(const pcl::PointCloud<pcl::PointXY>::Ptr& projected_cloudPtr);
std::pair<py::array_t<int>, py::array_t<int>> surface_boundary_filter(const py::array_t<double>& cloud, int neighbors, int n_iter, double threshold);


#endif //SURFACE_BOUNDARY_FILTER_SURFACEBOUNDARYFILTER_H
