#ifndef LOP_H
#define LOP_H


#include "skeleton.h"
#include "tools.h"



double GetAlpha(const Eigen::Vector3d &point_xi, const Eigen::Vector3d &point_pj, const double &h);


Eigen::Vector3d GetFirstTerm(const Eigen::Vector3d &point_xi, const double &h, const std::vector<size_t> &neighbor_indices_raw,
							 const Eigen::MatrixXd &cloud);


Eigen::MatrixXd LOPCalibrate(const Eigen::MatrixXd &cloud, const Eigen::MatrixXd &skeleton_cloud, const nlohmann::json &config);



#endif	// LOP_H
