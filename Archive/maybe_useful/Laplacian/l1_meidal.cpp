#include <utility>
#include <vector>
#include <algorithm>
#include <cmath>
#include <set>
#include <memory>
#include <iostream>
#include <string>
#include <fstream>

#include <omp.h>
#include <chrono>

#include <limits>
#include <pcl/types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/common/distances.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/random_sample.h>
#include <pcl/features/moment_of_inertia_estimation.h>


class Center {
    public:
        explicit Center(int id, pcl::PointXYZ point, double sigma, std::string type) : id(id), point(point), sigma(sigma), type(std::move(type)) {}

        // Getters
        int getId() const { return id; }
        pcl::PointXYZ getPoint() const { return point; }
        Eigen::Vector3d getEigenvalues() const { return eigenvalues; }
        double getSigma() const { return sigma; }
        std::vector<int> getKNearestCentersIdx() const { return k_nearest_centers_idx; }
        std::vector<int> getHRadiusCentersIdx() const { return h_radius_centers_idx; }
        std::vector<int> getHRadiusPointsIdx() const { return h_radius_points_idx; }
        std::vector<int> getKNearestBranchCandidatesIdx() const { return h_radius_branch_candidates_idx; }
        std::string getType() const { return type; }

        // Setters
        void setPoint(const pcl::PointXYZ& newPoint) { point = newPoint; }
        void setEigenvalues(const Eigen::Vector3d& newEigenvalues) { eigenvalues = newEigenvalues; }
        void setSigma(double newSigma) { sigma = newSigma; }
        void setKNearestCentersIdx(const std::vector<int>& newKNearestCentersIdx) { k_nearest_centers_idx = newKNearestCentersIdx; }
        void setHRadiusCentersIdx(const std::vector<int>& newHRadiusCentersIdx) { h_radius_centers_idx = newHRadiusCentersIdx; }
        void setHRadiusPointsIdx(const std::vector<int>& newHRadiusPointsIdx) { h_radius_points_idx = newHRadiusPointsIdx; }
        void setKNearestBranchCandidatesIdx(const std::vector<int>& newKNearestBranchCandidatesIdx) { h_radius_branch_candidates_idx = newKNearestBranchCandidatesIdx; }
        void setType(const std::string& newType) { type = newType; }

    private:
        int id;
        pcl::PointXYZ point;
        Eigen::Vector3d eigenvalues;
        double sigma;
        std::vector<int> k_nearest_centers_idx;
        std::vector<int> h_radius_centers_idx;
        std::vector<int> h_radius_points_idx;
        std::vector<int> h_radius_branch_candidates_idx;
        std::string type;
};


class Skeleton {
    public:
        explicit Skeleton(std::shared_ptr<std::vector<Center>> center_pointsPtr, pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloudPtr, int K, double h) : center_pointsPtr(std::move(center_pointsPtr)), original_cloudPtr(std::move(original_cloudPtr)), K(K), h(h) {
            update();
        }

        void update() {
            updateKdTree();
            updateNeighborsIndex();
        }

        // Getters
        std::shared_ptr<std::vector<Center>> getCenterPointsPtr() const { return center_pointsPtr; }
        pcl::PointCloud<pcl::PointXYZ>::Ptr getOriginalCloudPtr() const { return original_cloudPtr; }
        double getH() const { return h; }

        std::vector<int> getBranchCandidateIdx() {
            updateBranchCandidatesIndex();
            updateHRadiusBranchCandidatesIndex();
            return branch_candidate_idx;
        }

        // Setters
        void setH(double newH) { h = newH; }

        // Functions
        bool hasBranchCandidate() {
            for (size_t i = 0; i < center_pointsPtr->size(); ++i) {
                if ((*center_pointsPtr)[i].getType() == "branch candidate") {
                    return true;
                }
            }
            return false;
        }

    private:
        std::shared_ptr<std::vector<Center>> center_pointsPtr;
        pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloudPtr;
        int K;
        double h;
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr points_kdtreePtr;
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr centers_kdtreePtr;
        std::vector<int> branch_candidate_idx;

        void updateKdTree() {
            points_kdtreePtr->setInputCloud(original_cloudPtr);
            pcl::PointCloud<pcl::PointXYZ>::Ptr center_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
            for (size_t i = 0; i < center_pointsPtr->size(); ++i) {
                center_cloudPtr->push_back((*center_pointsPtr)[i].getPoint());
            }
            centers_kdtreePtr->setInputCloud(center_cloudPtr);
        }

        void updateNeighborsIndex() {
            for (size_t i = 0; i < center_pointsPtr->size(); ++i) {
                std::vector<int> k_nearest_centers_idx;
                std::vector<float> _;
                centers_kdtreePtr->nearestKSearch((*center_pointsPtr)[i].getPoint(), K, k_nearest_centers_idx, _);
                (*center_pointsPtr)[i].setKNearestCentersIdx(k_nearest_centers_idx);

                std::vector<int> h_radius_centers_idx;
                centers_kdtreePtr->radiusSearch((*center_pointsPtr)[i].getPoint(), h, h_radius_centers_idx, _);
                (*center_pointsPtr)[i].setHRadiusCentersIdx(h_radius_centers_idx);

                std::vector<int> h_radius_points_idx;
                points_kdtreePtr->radiusSearch((*center_pointsPtr)[i].getPoint(), h, h_radius_points_idx, _);
                (*center_pointsPtr)[i].setHRadiusPointsIdx(h_radius_points_idx);
            }
        }

        void updateBranchCandidatesIndex() {
            branch_candidate_idx.clear();
            for (size_t i = 0; i < center_pointsPtr->size(); ++i) {
                if ((*center_pointsPtr)[i].getType() == "branch candidate") {
                    branch_candidate_idx.push_back(i);
                }
            }
        }

        void updateHRadiusBranchCandidatesIndex() {
            pcl::PointCloud<pcl::PointXYZ>::Ptr branch_candidates_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
            for (size_t i = 0; i < branch_candidate_idx.size(); ++i) {
                branch_candidates_cloudPtr->push_back((*center_pointsPtr)[branch_candidate_idx[i]].getPoint());
            }
            pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr branch_candidates_kdtreePtr(new pcl::KdTreeFLANN<pcl::PointXYZ>);
            branch_candidates_kdtreePtr->setInputCloud(branch_candidates_cloudPtr);

            std::vector<int> h_radius_branch_candidates_idx_within_branch_candidates;
            std::vector<float> _;
            for (size_t i = 0; i < branch_candidate_idx.size(); ++i) {
                branch_candidates_kdtreePtr->radiusSearch((*center_pointsPtr)[branch_candidate_idx[i]].getPoint(), h,
                                                          h_radius_branch_candidates_idx_within_branch_candidates, _);
                std::vector<int> h_radius_branch_candidates_idx;
                for (int idx: h_radius_branch_candidates_idx_within_branch_candidates) {
                    h_radius_branch_candidates_idx.push_back(branch_candidate_idx[idx]);
                }
                (*center_pointsPtr)[branch_candidate_idx[i]].setKNearestBranchCandidatesIdx(
                        h_radius_branch_candidates_idx);
            }
        }
};





pcl::PointCloud<pcl::PointXYZ> l1_meidal_skeleton(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr, int num_centers, int k_neighbours) {
    // Compute the h0
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloudPtr);
    feature_extractor.compute();
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);
    double diag = pcl::euclideanDistance(min_point_AABB, max_point_AABB);
    double h0 = (2 * diag) / std::cbrt(static_cast<double>(cloudPtr->size()));

    // Sample centers (random sampling, seed = 8)
    pcl::PointCloud<pcl::PointXYZ>::Ptr center_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RandomSample<pcl::PointXYZ> random_sampler;
    random_sampler.setInputCloud(cloudPtr);
    random_sampler.setSample(num_centers);
    random_sampler.setSeed(8);
    random_sampler.filter(*center_cloudPtr);
    // Store centers in Center class
    std::shared_ptr<std::vector<Center>> centersPtr(new std::vector<Center>);
    for (size_t i = 0; i < center_cloudPtr->size(); ++i) {
        Center c(i, center_cloudPtr->points[i], 0, "non-branch");
        centersPtr->push_back(c);
    }
    assert(centersPtr->size() == center_cloudPtr->size());

    // Store centers in Skeleton class
    Skeleton skeleton(centersPtr, cloudPtr, k_neighbours, h0);

    int iteration = 50;
    while (iteration > 0) {
        // Compute sigma for each Center
        for (size_t i = 0; i < skeleton.getCenterPointsPtr()->size(); ++i) {
            std::shared_ptr<Center> xiPtr(new Center(skeleton.getCenterPointsPtr()->at(i)));

            // Transform to Eigen
            Eigen::Vector3d xi_vector = xiPtr->getPoint().getVector3fMap().cast<double>();
            Eigen::MatrixXd neighbor_pointsMatrix(xiPtr->getHRadiusPointsIdx().size(), 3);
            for (size_t j = 0; j < xiPtr->getHRadiusPointsIdx().size(); ++j) {
                neighbor_pointsMatrix.row(j) = (skeleton.getOriginalCloudPtr()->points[xiPtr->getHRadiusPointsIdx()[j]]).getVector3fMap().cast<double>();
            }
            Eigen::MatrixXd neighbor_centersMatrix(xiPtr->getHRadiusCentersIdx().size(), 3);
            for (size_t j = 0; j < xiPtr->getHRadiusCentersIdx().size(); ++j) {
                neighbor_centersMatrix.row(j) = (skeleton.getCenterPointsPtr()->at(xiPtr->getHRadiusCentersIdx()[j])).getPoint().getVector3fMap().cast<double>();
            }

            // Compute weighted covariance matrix
            Eigen::Matrix3d sigma_cov;
            sigma_cov.setZero();
            Eigen::MatrixXd diffMatrix_centers = neighbor_centersMatrix.rowwise() - xi_vector.transpose();
            Eigen::VectorXd l1_distances_centers = diffMatrix_centers.cwiseAbs().rowwise().sum();
            Eigen::VectorXd thetas_centers = (-l1_distances_centers.array().square() / pow(h0 / 2, 2)).exp();
            for (size_t j = 0; j < xiPtr->getHRadiusCentersIdx().size(); ++j) {
                Eigen::Vector3d diff = diffMatrix_centers.row(j);
                sigma_cov += thetas_centers(j) * (diff * diff.transpose());
            }

            // Compute eigenvalues
            Eigen::EigenSolver<Eigen::Matrix3d> es(sigma_cov);
            Eigen::Vector3d eigenvalues = es.eigenvalues().real();
            std::sort(eigenvalues.data(), eigenvalues.data() + 3, std::greater<double>());
            xiPtr->setEigenvalues(eigenvalues);

            // Compute sigma
            double sigma = eigenvalues[0] / (eigenvalues[0] + eigenvalues[1] + eigenvalues[2]);
            xiPtr->setSigma(sigma);

            // Compute A
            Eigen::MatrixXd diffMatrix_points = neighbor_pointsMatrix.rowwise() - xi_vector.transpose();
            Eigen::VectorXd l1_distances_points = diffMatrix_points.cwiseAbs().rowwise().sum();
            Eigen::VectorXd thetas_points = (-l1_distances_points.array().square() / pow(h0 / 2, 2)).exp();
            Eigen::VectorXd alphas = thetas_points.array() / (l1_distances_points.array() + 1e-8);
            double denominator_A = alphas.sum();
            Eigen::Vector3d A = (neighbor_pointsMatrix.transpose() * alphas).rowwise().sum() / denominator_A;

            // Compute R
            Eigen::VectorXd betas = thetas_centers.array() / (l1_distances_centers.array().square() + 1e-8);
            double denominator_R = betas.sum();
            Eigen::Vector3d R = 0.35 * sigma * ((neighbor_centersMatrix.transpose() * betas).rowwise().sum() / denominator_R);

            // Compute new center
            Eigen::Vector3d new_center = A + R;
            xiPtr->setPoint(pcl::PointXYZ(new_center[0], new_center[1], new_center[2]));

            // --- Finish contraction --- //

            // Compute smooth sigma
            if (xiPtr->getType() == "non-branch") {
                // TODO: Remove the center which is too close to each other
                // Compute smooth sigma
                // Transform to Eigen
                Eigen::MatrixXd k_neighbor_centersMatrix(xiPtr->getKNearestCentersIdx().size(), 3);
                for (size_t j = 0; j < xiPtr->getKNearestCentersIdx().size(); ++j) {
                    k_neighbor_centersMatrix.row(j) = (skeleton.getCenterPointsPtr()->at(xiPtr->getKNearestCentersIdx()[j])).getPoint().getVector3fMap().cast<double>();
                }

                double smooth_sigma = 0;
                for (size_t j = 0; j < xiPtr->getKNearestCentersIdx().size(); ++j) {
                    Eigen::Vector3d xj_vector = (skeleton.getCenterPointsPtr()->at(xiPtr->getKNearestCentersIdx()[j])).getPoint().getVector3fMap().cast<double>();

                    // Compute weighted covariance matrix
                    Eigen::Matrix3d sigma_cov;
                    sigma_cov.setZero();
                    Eigen::MatrixXd diffMatrix = k_neighbor_centersMatrix.rowwise() - xj_vector.transpose();
                    Eigen::VectorXd l1_distances = diffMatrix.cwiseAbs().rowwise().sum();
                    Eigen::VectorXd thetas = (-l1_distances.array().square() / pow(h0 / 2, 2)).exp();
                    for (size_t j = 0; j < xiPtr->getKNearestCentersIdx().size(); ++j) {
                        Eigen::Vector3d diff = diffMatrix.row(j);
                        sigma_cov += thetas(j) * (diff * diff.transpose());
                    }

                    // Compute eigenvalues
                    Eigen::EigenSolver<Eigen::Matrix3d> es(sigma_cov);
                    Eigen::Vector3d eigenvalues = es.eigenvalues().real();
                    std::sort(eigenvalues.data(), eigenvalues.data() + 3, std::greater<double>());

                    // Compute sigma
                    double sigma = eigenvalues[0] / (eigenvalues[0] + eigenvalues[1] + eigenvalues[2]);
                    smooth_sigma += sigma;
                }
                smooth_sigma /= k_neighbours;
                xiPtr->setSigma(smooth_sigma);

                if (smooth_sigma > 0.9) {
                    xiPtr->setType("branch candidate");
                }

                // --- Finish identify branch candidates after one contraction --- //

            }

            // Update centersPtr
            skeleton.getCenterPointsPtr()->at(i) = *xiPtr;

        }
        // Update skeleton
        skeleton.update();

        // Identify branch centers from branch candidates
        while (skeleton.hasBranchCandidate()) {

            // Get branch centers
            std::vector<int> branch_candidates_idx = skeleton.getBranchCandidateIdx();

            // Locate a seed point
            double max_sigma = 0;
            int max_sigma_idx = 0;
            for (size_t i = 0; i < branch_candidates_idx.size(); ++i) {
                if (skeleton.getCenterPointsPtr()->at(branch_candidates_idx[i]).getSigma() > max_sigma) {
                    max_sigma = skeleton.getCenterPointsPtr()->at(branch_candidates_idx[i]).getSigma();
                    max_sigma_idx = i;
                }
            }
            std::shared_ptr<Center> seedPtr(new Center(skeleton.getCenterPointsPtr()->at(branch_candidates_idx[max_sigma_idx]));

            // Get neighbors within h0
            std::vector<int> neighbor_branch_candidates_idx = seedPtr->getKNearestBranchCandidatesIdx();

            // Trace along the dominant PCA direction
            // Initial head and tail points
            Center tail = *seedPtr;
            Eigen::Vector3d tail_point = seedPtr->getPoint().getVector3fMap().cast<double>();
            // Beign with the closest neighbor
            Center head = skeleton.getCenterPointsPtr()->at(neighbor_branch_candidates_idx[1]);
            Eigen::Vector3d head_point = head.getPoint().getVector3fMap().cast<double>();
            Eigen::Vector3d head_vector = head_point - tail_point;
            Eigen::Vector3d tail_vector;

            int branch_count = 0;
            for (size_t i = 2; i < neighbor_branch_candidates_idx.size(); ++i) {
                Eigen::Vector3d neighbor_point = skeleton.getCenterPointsPtr()->at(neighbor_branch_candidates_idx[i]).getPoint().getVector3fMap().cast<double>();
                Eigen::Vector3d diff_vector = neighbor_point - head_point;

                // Determine neighbor_2 is on the head or tail of the branch
                if (head_vector.dot(diff_vector) < 0) {
                    // neighbor_point is on the tail of the branch
                    tail_vector = neighbor_point - tail_point;
                    double cos_angle = tail_vector.dot(diff_vector) / (tail_vector.norm() * diff_vector.norm());
                    if (cos_angle <= 0.9) {
                        tail_point = neighbor_point;
                        branch_count++;
                    } else {
                        tail_point = neighbor_point;
                    }
                } else {
                    // neighbor_2 is on the head of the branch
                    head_vector = neighbor_point - head_point;
                    double cos_angle = head_vector.dot(diff_vector) / (head_vector.norm() * diff_vector.norm());
                    if (cos_angle <= 0.9) {
                        head_point = neighbor_point;
                        branch_count++;
                    } else {
                        head_point = neighbor_point;
                    }
                }
            }
            if (branch_count >= 6) {
                // This is a branch
                for (size_t i = 0; i < neighbor_branch_candidates_idx.size(); ++i) {
                    skeleton.getCenterPointsPtr()->at(i).setType("branch");
                }
                int tail_idx = tail.getId();
                int head_idx = head.getId();
                skeleton.getCenterPointsPtr()->at(tail_idx).setType("bridge");
                skeleton.getCenterPointsPtr()->at(head_idx).setType("bridge");
            } else {
                // This is not a branch
                for (size_t i = 0; i < neighbor_branch_candidates_idx.size(); ++i) {
                    skeleton.getCenterPointsPtr()->at(i).setType("non-branch");
                }
            }
        }
        // Update skeleton
        skeleton.update();

        iteration--;
    }







};


void loadXYZFile(std::string& file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudXYZ_Ptr) {
    std::ifstream file(file_path);
    std::string suffix = file_path.substr(file_path.find_last_of("."));
    if (!file.is_open()) {
        throw std::runtime_error("No File Found");
    } else if (suffix != ".txt" && suffix != ".xyz") {
        throw std::runtime_error("Only Support .txt and .xyz Format");
    } else {
        double x, y, z;
        while (file >> x >> y >> z) {
            cloudXYZ_Ptr->points.push_back(pcl::PointXYZ(x, y, z));
        }
        file.close();
        cloudXYZ_Ptr->width = cloudXYZ_Ptr->size();
        cloudXYZ_Ptr->height = 1;
        cloudXYZ_Ptr->is_dense = false;
    }
}


int main() {
    std::string input_file_path = "/mnt/d/MScThesis/PlantSeg/preprocessing/00_example_data/downsampled_1000000.xyz";
    std::string output_file_path = "../edge_points.pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);// = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    loadXYZFile(input_file_path, cloudPtr);
    std::cout << "data loaded!" << std::endl;



    pcl::PCDWriter writer;
    writer.write(output_file_path, *cloudPtr);
    std::cout << "data saved!" << std::endl;

    return 0;
}
