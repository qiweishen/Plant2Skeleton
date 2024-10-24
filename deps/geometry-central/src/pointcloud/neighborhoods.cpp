#include "geometrycentral/pointcloud/neighborhoods.h"

#include "geometrycentral/pointcloud/point_cloud.h"
#include "geometrycentral/utilities/knn.h"

namespace geometrycentral {
	namespace pointcloud {

		Neighborhoods::Neighborhoods(PointCloud& cloud_, const PointData<Vector3>& positions, unsigned int nNeighbors) :
			cloud(cloud_),
			neighbors(cloud)

		{
			GC_SAFETY_ASSERT(cloud.isCompressed(), "cloud must be compressed");

			std::vector<Vector3> pointVec;
			pointVec.reserve(cloud.nPoints());
			for (Point p: cloud.points()) {
				pointVec.push_back(positions[p]);
			}

			// Find neighbors
			NearestNeighborFinder knn(pointVec);
			for (Point p: cloud.points()) {
				neighbors[p].resize(nNeighbors);
				std::vector<size_t> neighInd = knn.kNearestNeighbors(p.getIndex(), nNeighbors);
				for (size_t j = 0; j < neighInd.size(); j++) {
					neighbors[p][j] = cloud.point(neighInd[j]);
				}
			}
		}


		Neighborhoods::Neighborhoods(PointCloud& cloud_, const PointData<Vector3>& positions, double radius) :
			cloud(cloud_),
			neighbors(cloud)

		{
			GC_SAFETY_ASSERT(cloud.isCompressed(), "cloud must be compressed");

			std::vector<Vector3> pointVec;
			pointVec.reserve(cloud.nPoints());
			for (Point p: cloud.points()) {
				pointVec.push_back(positions[p]);
			}

			// Find neighbors
			NearestNeighborFinder kdtree(pointVec);
			for (Point p: cloud.points()) {
				std::vector<size_t> neighInd = kdtree.radiusSearch(pointVec[p.getIndex()], radius);
				neighInd.erase(std::remove(neighInd.begin(), neighInd.end(), p.getIndex()), neighInd.end());
				neighbors[p].resize(neighInd.size());
				for (size_t j = 0; j < neighInd.size(); j++) {
					neighbors[p][j] = cloud.point(neighInd[j]);
				}
			}
		}

	}  // namespace pointcloud
}  // namespace geometrycentral
