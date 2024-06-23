#include "SurfaceBoundaryFilter.h"


void numpy2cloud(const py::array_t<double>& cloud, std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& cloudPtr) {
    int rows = cloud.shape(0);
    cloudPtr->width = rows;
    cloudPtr->height = 1;
    cloudPtr->is_dense = false;
    cloudPtr->points.resize(rows);

    for (size_t i = 0; i < rows; ++i) {
        cloudPtr->points[i].x = cloud.at(i, 0);
        cloudPtr->points[i].y = cloud.at(i, 1);
        cloudPtr->points[i].z = cloud.at(i, 2);
    }
}


py::array_t<double> cloud2numpy(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPtr) {
    int rows = cloudPtr->size();
    int cols = 3;
    auto result = py::array_t<double>(rows * cols);

    for (size_t i = 0; i < cloudPtr->size(); ++i) {
        result.mutable_at(i * 3 + 0) = cloudPtr->points[i].x;
        result.mutable_at(i * 3 + 1) = cloudPtr->points[i].y;
        result.mutable_at(i * 3 + 2) = cloudPtr->points[i].z;
    }

    result.resize({rows, cols});

    return result;
}


std::vector<int> numpy2vector(const py::array_t<int>& array) {
    size_t rows = array.shape(0);
    std::vector<int> result(rows);

    for (size_t i = 0; i < rows; ++i) {
        result[i] = array.at(i);
    }

    return result;
}


py::array_t<int> vector2numpy(const std::vector<int>& vec) {
    size_t rows = vec.size();
    auto result = py::array_t<int>(rows);

    std::memcpy(result.mutable_data(), vec.data(), rows * sizeof(int));

    return result;
}


void knnSearch(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPtr, const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr& kdtreePtr,
               const pcl::PointXYZ& searchPoint, pcl::PointCloud<pcl::PointXYZ>::Ptr& k_cloudPtr,
               int neighbors) {
    std::vector<int> k_indices_neighbors;
    std::vector<float> _;

    kdtreePtr->nearestKSearch(searchPoint, neighbors, k_indices_neighbors, _);

    for(int i : k_indices_neighbors) {
        k_cloudPtr->push_back((*cloudPtr)[i]);
    }
}


void project2PCA(const pcl::PointCloud<pcl::PointXYZ>::Ptr& k_cloudPtr, pcl::PointCloud<pcl::PointXY>::Ptr& projected_cloudPtr) {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*k_cloudPtr, centroid);

    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(k_cloudPtr);
    const Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
    Eigen::MatrixXf projection_matrix = eigenvectors.leftCols(2).transpose();

    projected_cloudPtr->points.resize(k_cloudPtr->size());

    for (size_t i = 0; i < k_cloudPtr->points.size(); ++i) {
        Eigen::Vector3f pt(k_cloudPtr->points[i].x - centroid[0], k_cloudPtr->points[i].y - centroid[1], k_cloudPtr->points[i].z - centroid[2]);
        Eigen::Vector2f projected_pt = projection_matrix * pt;
        projected_cloudPtr->points[i].x = projected_pt(0);
        projected_cloudPtr->points[i].y = projected_pt(1);
    }
}


std::vector<double> getAngles(const pcl::PointCloud<pcl::PointXY>::Ptr& projected_cloudPtr) {
    // Set the vector of u axis
    Eigen::Vector2f u(1, 0);

    std::vector<double> angles;
    pcl::PointXY& center_pt = (*projected_cloudPtr)[0];
    for (size_t i = 1; i < projected_cloudPtr->size(); ++i) {
        const pcl::PointXY& neighbor = (*projected_cloudPtr)[i];
        Eigen::Vector2f vec(neighbor.x - center_pt.x, neighbor.y - center_pt.y);

        double dot_product = vec.dot(u);
        double norm_product = vec.norm() * u.norm();

        double angle = 0.0;
        if (norm_product != 0) {
            angle = std::acos(dot_product / norm_product);
            double cross_product = (vec[0] * u[1]) - (vec[1] * u[0]);
            if (cross_product < 0) {
                angle = -angle;
            }
        }
        angles.push_back(angle);
    }
    sort(angles.begin(), angles.end());

    return angles;
}


//TODO: use k-neighbors or radius
std::pair<py::array_t<int>, py::array_t<int>> surface_boundary_filter(const py::array_t<double>& cloud, int neighbors, int n_iter, double threshold) {
    // Load original cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    numpy2cloud(cloud, cloudPtr);

    // Create center/edge cloudPtrs
    pcl::PointCloud<pcl::PointXYZ>::Ptr center_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloudPtr, *center_cloudPtr);
    pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);

    while (n_iter > 0) {
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreePtr(new pcl::KdTreeFLANN<pcl::PointXYZ>);// = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();
        kdtreePtr->setInputCloud(center_cloudPtr);

        pcl::PointIndices::Ptr edge_pts_indices(new pcl::PointIndices);// = std::make_shared<pcl::PointIndices>();

        #pragma omp parallel
        {
            pcl::PointIndices::Ptr local_edge_pts_indices(new pcl::PointIndices);// = std::make_shared<pcl::PointIndices>(); // Local storage for each thread

            #pragma omp for nowait
            for (size_t i = 0; i < center_cloudPtr->size(); ++i) {
                const pcl::PointXYZ& searchPoint = (*center_cloudPtr)[i];
                pcl::PointCloud<pcl::PointXYZ>::Ptr k_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);// = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
                knnSearch(center_cloudPtr, kdtreePtr, searchPoint, k_cloudPtr, neighbors);

                if (!k_cloudPtr->empty()) {
                    // Project to PCA plane
                    pcl::PointCloud<pcl::PointXY>::Ptr projected_cloudPtr(new pcl::PointCloud<pcl::PointXY>);// =  std::make_shared<pcl::PointCloud<pcl::PointXY>>();
                    project2PCA(k_cloudPtr, projected_cloudPtr);

                    // Get angles with signs
                    std::vector<double> angles = getAngles(projected_cloudPtr);

                    if (!angles.empty()) {
                        double max_diff = 0.0;
                        const double* prev = &angles[0];
                        for (const double* it = prev + 1; it != &angles[0] + angles.size(); ++it) {
                            double diff = *it - *prev;
                            max_diff = std::max(max_diff, diff);
                            if (max_diff >= threshold) {
                                local_edge_pts_indices->indices.push_back(i);
                                break;
                            }
                            prev = it;
                        }
                    }
                }
            }
            #pragma omp critical
            {
                edge_pts_indices->indices.insert(edge_pts_indices->indices.end(), local_edge_pts_indices->indices.begin(), local_edge_pts_indices->indices.end());
            }
        }


        if (!edge_pts_indices->indices.empty()) {
            // Add edge points to the edge_cloud
            for (int i : edge_pts_indices->indices) {
                edge_cloudPtr->push_back((*center_cloudPtr)[i]);
            }
            // Remove edge points from the central_cloud
            pcl::ExtractIndices<pcl::PointXYZ> extract_center;
            extract_center.setInputCloud(center_cloudPtr);
            extract_center.setIndices(edge_pts_indices);
            extract_center.setNegative(true);
            extract_center.filter(*center_cloudPtr);
        }

        n_iter--;
    }
    // Get indices of center/edge points in the original cloud
    pcl::Indices all_edge_pts_indices;
    pcl::Indices all_center_pts_indices;

    pcl::getApproximateIndices<pcl::PointXYZ>(edge_cloudPtr, cloudPtr, all_edge_pts_indices);
    pcl::getApproximateIndices<pcl::PointXYZ>(center_cloudPtr, cloudPtr, all_center_pts_indices);

    return std::make_pair(vector2numpy(all_edge_pts_indices), vector2numpy(all_center_pts_indices));
}
