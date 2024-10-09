#include "LOP.h"
#include "Tools.h"


double GetAlpha(Eigen::Vector3d &point_xi, Eigen::Vector3d &point_pj, double &h) {
    double r = (point_xi - point_pj).cwiseAbs().sum() + 1e-8;
    double numerator = exp(-pow(r, 2) / pow(h / 4, 2));
    return numerator / r;
}


Eigen::Vector3d GetFirstTerm(Eigen::Vector3d &point_xi, double &h, std::vector<size_t> &neighbor_indices_raw,
                             std::shared_ptr<Eigen::MatrixXd> &cloudPtr) {
    Eigen::Vector3d numerator(0, 0, 0);
    double denominator = 0.0;
    for (const size_t &neighbor_index: neighbor_indices_raw) {
        Eigen::Vector3d point_pj = cloudPtr->row(int(neighbor_index));
        double alpha = GetAlpha(point_xi, point_pj, h);
        numerator += (point_pj * alpha);
        denominator += alpha;
    }
    return numerator / denominator;
}


std::shared_ptr<Eigen::MatrixXd> LOPCalibrate(std::shared_ptr<Eigen::MatrixXd> &cloudPtr,
                                              std::shared_ptr<Eigen::MatrixXd> &skeleton_cloudPtr,
                                              nlohmann::json &config) {
    MyLogger.Log("--------------------------------------------------", 0, true, false);
    MyLogger.Log("Start LOP Iteration", 0, true, true);
    auto start = std::chrono::high_resolution_clock::now();
    const int n = static_cast<int>(skeleton_cloudPtr->rows());

    std::shared_ptr<Eigen::MatrixXd> new_skeleton_cloudPtr = std::make_shared<Eigen::MatrixXd>(n, 3);
    std::vector<geometrycentral::Vector3> original_cloud_points = tool::utility::Matrix2GCVector(cloudPtr);
    geometrycentral::NearestNeighborFinder original_cloud_finder(original_cloud_points);
    std::vector<geometrycentral::Vector3> skeleton_cloud_points = tool::utility::Matrix2GCVector(skeleton_cloudPtr);

    double h = config["Preprocess"]["Normalize_Diagonal_Length"].get<double>() *
               config["Skeleton_Building"]["LOP_Sphere_Radius_Ratio"].get<double>();

#pragma omp parallel for
    for (int i = 0; i < n; ++i) {
        Eigen::Vector3d query_point = skeleton_cloudPtr->row(i);
        geometrycentral::Vector3 query_point_gc = skeleton_cloud_points[i];
        std::vector<size_t> original_point_cloud_indices;
        original_point_cloud_indices = original_cloud_finder.radiusSearch(query_point_gc, h);

        // No need to move the skeleton point if it does not have any neighbors in the original point cloud.
        if (original_point_cloud_indices.empty()) {
            new_skeleton_cloudPtr->row(i) = cloudPtr->row(i);
            continue;
        }

        Eigen::Vector3d first_term = GetFirstTerm(query_point, h, original_point_cloud_indices, cloudPtr);

        if (first_term.hasNaN()) {
            MyLogger.Log(std::format("NaN value in LOP calibration! Index: {}", i), 1, true, true);
            MyLogger.Log(std::format("First term: {}, {}, {}", first_term(0), first_term(1), first_term(2)), 1, true, true);
        }

        new_skeleton_cloudPtr->row(i) = Eigen::Vector3d(first_term);
    }

    assert(new_skeleton_cloudPtr->rows() == n);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    MyLogger.Log(std::format("LOP-based Calibration complete! Elapsed time: {:.6f}s.", elapsed.count()), 0, true, false);

    return new_skeleton_cloudPtr;
}
