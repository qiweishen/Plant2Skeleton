#ifndef CPPTEST_LOP_H
#define CPPTEST_LOP_H


#include "Skeleton.h"
#include "Tools.h"






double GetAlpha(Eigen::Vector3d& point_xi, Eigen::Vector3d& point_pj, double& h);
double GetBeta(Eigen::Vector3d& point_xi, Eigen::Vector3d& point_xj, double& h);
double GetNewBeta(double& point_xi, double& point_xj, double& h);
Eigen::Vector3d GetFirstTerm(Eigen::Vector3d& point_xi, double& h, std::vector<int>& neighbor_indices_raw,
                             std::shared_ptr<PointCloud>& cloudPtr);
Eigen::Vector3d GetSecondTerm(Eigen::Vector3d& point_xi, double& h, double mu,
                              std::vector<int>& neighbor_indices_skeleton,
                              std::shared_ptr<PointCloud>& cloudPtr);
Eigen::Vector3d GetNewSecondTerm(Eigen::Vector3d& point_xi, double& h, double mu,
                                 std::vector<int>& neighbor_indices_skeleton,
                                 std::shared_ptr<Eigen::MatrixXd>& ptsPtr);
std::vector<double> GetSmoothSigma(std::shared_ptr<PointCloud>& cloudPtr, double& h0);
std::shared_ptr<PointCloud> LOPIterate(std::shared_ptr<PointCloud>& cloudPtr,
                                       std::shared_ptr<PointCloud>& skeleton_cloudPtr);



#endif //CPPTEST_LOP_H
