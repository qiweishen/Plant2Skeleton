//@Authors: Sushil B. and Paul M. - (C) 2019, MIT License

#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>

#include "KDTree.hpp"
#include "geometrycentral/utilities/knn.h"


#define DIM 3

double getNum() {
	return ((double) rand() / (RAND_MAX));
}

std::vector<double> generateVector() {
	std::vector<double> temp(DIM);
	for (size_t idx = 0; idx < DIM; idx++) {
		temp[idx] = getNum();
	}
	return temp;
}

std::vector<std::vector<double>> getListofGeneratedVectors(int length) {
	std::vector<std::vector<double>> temp(length);
	for (size_t idx = 0; idx < length; idx++) {
		temp[idx] = generateVector();
	}
	return temp;
}

double sumSqrdErr(std::vector<double>& p1, std::vector<double>& p2) {
	std::vector<double> diff(DIM);
	std::vector<double> square(DIM);
	std::transform(p1.begin(), p1.end(), p2.begin(), diff.begin(), std::minus<double>());
	std::transform(diff.begin(), diff.end(), diff.begin(), square.begin(), std::multiplies<double>());
	return std::accumulate(square.begin(), square.end(), 0.0);
}


int main() {
	// seed
	srand(5);

	const std::vector<int> dataPointSizes = { 50, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 2000 };
	constexpr int nIter = 50;

	std::cout << "Total number of iterations ran: " << nIter << std::endl;

	for (auto& sizes: dataPointSizes) {
		std::vector<std::vector<double>> points(sizes, std::vector<double>(DIM)), pointToRetrieve(sizes, std::vector<double>(DIM));
		long long kdTreeRetTotalTime = 0.0;
		long long bruteForceRetTotalTime = 0.0;

		int correct = 0;
		int gc_correct = 0;
		for (int i = 0; i < nIter; i++) {
			// generate test points to build a tree
			points = getListofGeneratedVectors(sizes);
			std::vector<geometrycentral::Vector3> gc_points(sizes);
			for (size_t idx = 0; idx < sizes; idx++) {
				gc_points[idx] = geometrycentral::Vector3(points[idx][0], points[idx][1], points[idx][2]);
			}

			// generate KDTree
			KDTree tree(points);
			geometrycentral::NearestNeighborFinder finder(gc_points);

			// generate retrieve test data points
			pointToRetrieve = getListofGeneratedVectors(sizes);
			std::vector<geometrycentral::Vector3> gc_pointsToRetrieve(sizes);
			for (size_t idx = 0; idx < sizes; idx++) {
				gc_pointsToRetrieve[idx] = geometrycentral::Vector3(pointToRetrieve[idx][0], pointToRetrieve[idx][1], pointToRetrieve[idx][2]);
			}

			for (auto& vals: pointToRetrieve) {
				double minSumSqdErr = std::numeric_limits<double>::max();
				std::vector<double> groundTruthVec(DIM);
				for (auto& gtvals: points)	// loop through all the points that built KDTRee
				{
					double sumSqdErr = sumSqrdErr(gtvals, vals);
					if (sumSqdErr < minSumSqdErr) {
						minSumSqdErr = sumSqdErr;
						groundTruthVec = gtvals;
					}
				}
				std::vector<double> checkVec(DIM);
				checkVec = tree.nearest_point(vals);
				std::vector<double> gc_checkVec(DIM);
				gc_checkVec = { points[finder.kNearest(geometrycentral::Vector3(vals[0], vals[1], vals[2]), 1)[0]][0],
								points[finder.kNearest(geometrycentral::Vector3(vals[0], vals[1], vals[2]), 1)[0]][1],
								points[finder.kNearest(geometrycentral::Vector3(vals[0], vals[1], vals[2]), 1)[0]][2] };
				if (std::equal(groundTruthVec.begin(), groundTruthVec.end(), checkVec.begin())) {
					correct += 1;
				}
				if (std::equal(groundTruthVec.begin(), groundTruthVec.end(), gc_checkVec.begin())) {
					gc_correct += 1;
				}
			}
		}
		std::cout << "\n\nAccuracy (tested with " << sizes << " datasets per iter) = " << ((correct * 100.0) / (sizes * nIter))
				  << " %.  Avg. Total Number Correct: " << (int) (correct / nIter) << " / " << sizes;
		std::cout << "\nGC Accuracy (tested with " << sizes << " datasets per iter) = " << ((gc_correct * 100.0) / (sizes * nIter))
				  << " %.  Avg. Total Number Correct: " << (int) (gc_correct / nIter) << " / " << sizes;
	}
	return 0;
}
