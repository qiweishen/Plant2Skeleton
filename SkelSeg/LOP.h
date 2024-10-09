#ifndef LOP_H
#define LOP_H


#include "Skeleton.h"
#include "Tools.h"



double GetAlpha(Eigen::Vector3d &point_xi, Eigen::Vector3d &point_pj, double &h);


Eigen::Vector3d GetFirstTerm(Eigen::Vector3d &point_xi, double &h, std::vector<size_t> &neighbor_indices_raw, std::shared_ptr<Eigen::MatrixXd> &cloudPtr);


std::shared_ptr<Eigen::MatrixXd> LOPCalibrate(std::shared_ptr<Eigen::MatrixXd> &cloudPtr,
											  std::shared_ptr<Eigen::MatrixXd> &skeleton_cloudPtr,
											  nlohmann::json &config);



#endif	// LOP_H
