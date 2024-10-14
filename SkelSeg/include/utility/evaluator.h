#ifndef EVALUATE_H
#define EVALUATE_H


#include "../../Tools.h"



namespace evaluate {
	namespace skeleton {
		// TODO: Compute the distance between each original point and its nearest skeleton edge.
		double QuantitativeSkeletonDistance(std::shared_ptr<Eigen::MatrixXd> &cloudPtr, std::shared_ptr<Eigen::MatrixXd> &skeletonPtr,
											const std::string &distance_type) {
			std::vector<geometrycentral::Vector3> original_points = tool::utility::Matrix2GCVector(cloudPtr);
			std::vector<geometrycentral::Vector3> skeleton_points = tool::utility::Matrix2GCVector(skeletonPtr);
			geometrycentral::NearestNeighborFinder finder(original_points);
			std::vector<double> distance;
			for (const geometrycentral::Vector3 &vert: skeleton_points) {
				geometrycentral::Vector3 pt = original_points[finder.kNearest(vert, 1)[0]];
				double temp = (pt - vert).norm();
				distance.push_back(temp);
			}
			if (distance_type == "average") {
				return std::accumulate(distance.begin(), distance.end(), 0.0) / distance.size();
			} else if (distance_type == "max") {
				return *std::max_element(distance.begin(), distance.end());
			} else {
				std::cerr << "Invalid distance type." << std::endl;
				std::exit(EXIT_FAILURE);
			}
		}
	}  // namespace skeleton



	namespace segmentation {
		void QuantitativeSegmentMetrics() {}
	}  // namespace segmentation
}  // namespace evaluate



#endif	// EVALUATE_H
