/* \author Elena Yudovina */
// Euclidean clustering algorithm
// borrows heavily from the course quiz

#ifndef CLUSTER_H_
#define CLUSTER_H_

#include "kdtree.h"

class EuclideanCluster {
public:
	// main clustering function, returns a list of lists, each inner list containing the indices of one cluster
	// only returns clusters with size ranging from minClusterSize to maxClusterSize
	std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol,
		int minClusterSize, int maxClusterSize);
};


#endif /*CLUSTER_H_*/