#include "LOP.h"
#include "Tools.h"



double GetAlpha(Eigen::Vector3d& point_xi, Eigen::Vector3d& point_pj, double& h) {
    double r = (point_xi - point_pj).cwiseAbs().sum() + 1e-8;
    double numerator = exp(-pow(r, 2) / pow(h / 4, 2));
    return numerator / r;
}


double GetBeta(Eigen::Vector3d& point_xi, Eigen::Vector3d& point_xj, double& h) {
    double term1 = GetAlpha(point_xi, point_xj, h);
    double term2 = std::abs(-1 / (pow((point_xi - point_xj).cwiseAbs().sum() + 1e-8, 4)));
    return term1 * term2;
}


double GetNewBeta(double& xi, double& xj, double& h) {
    double r = std::abs(xi - xj) + 1e-8;
    double numerator = exp(-pow(r, 2) / pow(h / 4, 2));
    double term2 = std::abs(-1 / (pow(r, 4)));
    return (numerator / r) * term2;
}


Eigen::Vector3d GetFirstTerm(Eigen::Vector3d& point_xi, double& h, std::vector<int>& neighbor_indices_raw,
                             std::shared_ptr<PointCloud>& cloudPtr) {
    Eigen::Vector3d numerator(0, 0, 0);
    double denominator = 0.0;
    for (const int &neighbor_index : neighbor_indices_raw) {
        Eigen::Vector3d point_pj = cloudPtr->points_[neighbor_index];
        double alpha = GetAlpha(point_xi, point_pj, h);
        numerator += (point_pj * alpha);
        denominator += alpha;
    }
    return numerator / denominator;
}


Eigen::Vector3d GetSecondTerm(Eigen::Vector3d& point_xi, double& h, double mu,
                              std::vector<int>& neighbor_indices_skeleton,
                              std::shared_ptr<PointCloud>& cloudPtr) {
    Eigen::Vector3d numerator(0, 0, 0);
    double denominator = 0.0;
    for (const int &neighbor_index : neighbor_indices_skeleton) {
        Eigen::Vector3d point_xj = cloudPtr->points_[neighbor_index];
        double beta = GetBeta(point_xi, point_xj, h);
        numerator += (beta * (point_xi - point_xj));
        denominator += beta;
    }
    return mu * (numerator / denominator);
}


Eigen::Vector3d GetNewSecondTerm(Eigen::Vector3d& point_xi, double& h, double mu,
                                 std::vector<int>& neighbor_indices_skeleton,
                                 std::shared_ptr<Eigen::MatrixXd>& ptsPtr) {
    Eigen::Vector3d numerator(0, 0, 0);
    double denominator = 0.0;

    std::shared_ptr<PointCloud> skeleton_neighbor_cloudPtr = std::make_shared<PointCloud>();
    for (const int &neighbor_index : neighbor_indices_skeleton) {
        skeleton_neighbor_cloudPtr->points_.emplace_back(ptsPtr->row(neighbor_index));
    }
    Eigen::Vector3d mean;
    Eigen::Matrix3d covariance;
    std::tie(mean, covariance) = skeleton_neighbor_cloudPtr->ComputeMeanAndCovariance();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
    // eigen_vectors The columns are inverted, col(2).value > col(1).value > col(0).value
    Eigen::Matrix3d eigen_vectors = eigen_solver.eigenvectors();

    double xi = (point_xi - mean).dot(eigen_vectors.col(1));

    for (const int &neighbor_index : neighbor_indices_skeleton) {
        Eigen::Vector3d point_xj = ptsPtr->row(neighbor_index);
        double xj = (point_xj - mean).dot(eigen_vectors.col(1));
        double beta = GetNewBeta(xi, xj, h);
        numerator += (beta * (xi - xj) * eigen_vectors.col(1));
        denominator += beta;
    }
    return mu * (numerator / denominator);
}


std::vector<double> GetSmoothSigma(std::shared_ptr<PointCloud>& cloudPtr, double& h0) {
    const int n = static_cast<int>(cloudPtr->points_.size());
    std::vector<double> sigma_list(n);
    std::vector<double> smooth_sigma_list(n);
    std::shared_ptr<KDTreeFlann> kdtree(new KDTreeFlann);
    kdtree->SetGeometry(*cloudPtr);

    std::vector<std::vector<int>> neighbors_indices;
    for (int i = 0; i < n; ++i) {
        Eigen::Vector3d query_point = cloudPtr->points_[i];
        std::vector<int> indices;
        std::vector<double> distances;
        double radius = 0.05 * h0;
        kdtree->SearchRadius(query_point, radius, indices, distances);
        indices.erase(indices.begin());
        neighbors_indices.emplace_back(indices);
        Eigen::MatrixXd Ci = Eigen::MatrixXd::Zero(3, 3);
        // Compute Ci for Pi using its neighborhood.
        for (const int &index : indices) {
            Eigen::Vector3d pj = cloudPtr->points_[index];; // Get the neighboring point Pj.
            Eigen::VectorXd diff = query_point - pj;
            // Compute the weight based on the distance and a theta function.
            double weight = (query_point - pj).cwiseAbs().sum();
            weight = exp(-pow(weight, 2) / pow(radius / 4, 2));
            // Add the weighted outer product to the covariance matrix Ci.
            Ci += weight * diff * diff.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(Ci);
        // eigen_vectors The columns are inverted, col(2).value > col(1).value > col(0).value
        Eigen::Vector3d eigen_values = eigen_solver.eigenvalues();
        double lambda_0 = eigen_values(0);
        double lambda_1 = eigen_values(1);
        double lambda_2 = eigen_values(2);
        double sigma = lambda_2 / (lambda_0 + lambda_1 + lambda_2);
        sigma_list[i] = sigma;
    }
    for (int i = 0; i < n; ++i) {
        smooth_sigma_list[i] += sigma_list[i];
        for (const int &index : neighbors_indices[i]) {
            smooth_sigma_list[i] += sigma_list[index];
        }
        smooth_sigma_list[i] /= neighbors_indices[i].size() + 1;
    }

    return smooth_sigma_list;
}


std::shared_ptr<PointCloud> LOPIterate(std::shared_ptr<PointCloud>& cloudPtr,
                                       std::shared_ptr<PointCloud>& skeleton_cloudPtr) {
    const int n = static_cast<int>(skeleton_cloudPtr->points_.size());

    std::shared_ptr<PointCloud> new_skeleton_cloudPtr = std::make_shared<PointCloud>(*skeleton_cloudPtr);
    std::shared_ptr<KDTreeFlann> cloud_kdtreePtr = std::make_shared<KDTreeFlann>();
    cloud_kdtreePtr->SetGeometry(*cloudPtr);

    double h0 = 1.6;
    double h_weight = 0.04;
    double mu = 0.0;

    std::cout << "--------------------------------------------------" << std::endl;
    std::cout << "Start LOP Iteration" << std::endl;
    double epsilon = 1e-4;
    int time = 0;
    while (h_weight <= 0.04) {
        double average_distance = 0.0;
        std::shared_ptr<PointCloud> temp_new_skeleton_cloudPtr = std::make_shared<PointCloud>();
        temp_new_skeleton_cloudPtr->points_.resize(n);

        std::shared_ptr<KDTreeFlann> skeleton_kdtreePtr = std::make_shared<KDTreeFlann>();
        skeleton_kdtreePtr->SetGeometry(*new_skeleton_cloudPtr);

        #pragma omp parallel for
        for (int i = 0; i < n; ++i) {
            double h = h_weight * h0;
            Eigen::Vector3d query_point = new_skeleton_cloudPtr->points_[i];
            std::vector<int> raw_indices;
            std::vector<int> skeleton_indices;
            std::vector<double> _;
            cloud_kdtreePtr->SearchRadius(query_point, h, raw_indices, _);
            skeleton_kdtreePtr->SearchRadius(query_point, h, skeleton_indices, _);

            if (raw_indices.empty() || skeleton_indices.size() == 1) {
                temp_new_skeleton_cloudPtr->points_[i] = new_skeleton_cloudPtr->points_[i];
                continue;
            } else {
                skeleton_indices.erase(skeleton_indices.begin());
            }

            Eigen::Vector3d first_term = GetFirstTerm(query_point, h, raw_indices, cloudPtr);
            Eigen::Vector3d second_term = GetSecondTerm(query_point, h, mu, skeleton_indices, new_skeleton_cloudPtr);

            if (first_term.hasNaN() || second_term.hasNaN()) {
                std::cerr << "Index: " << i << std::endl;
                std::cerr << "First term: " << first_term(0) << ", " << first_term(1) << ", " << first_term(2) << std::endl;
                std::cerr << "Second term: " << second_term(0) << ", " << second_term(1) << ", " << second_term(2) << std::endl;
            }

            temp_new_skeleton_cloudPtr->points_[i] = Eigen::Vector3d(first_term + second_term);
            average_distance += (temp_new_skeleton_cloudPtr->points_[i] - query_point).norm();
        }

        assert(temp_new_skeleton_cloudPtr->points_.size() == n);

        average_distance /= n;
        *new_skeleton_cloudPtr = *temp_new_skeleton_cloudPtr;
        h_weight = h_weight + 0.01;
        time++;
        std::cout << "Inner LOP Iteration time: " << time << std::endl;
        std::cout << "Average distance: " << average_distance << std::endl;
    }

    return new_skeleton_cloudPtr;
}

