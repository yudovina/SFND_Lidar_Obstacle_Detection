/* \author Elena Yudovina */
// Euclidean clustering algorithm
// borrows heavily from the course quiz

#include "cluster.h"

// helper function for adding a point and its neighbors to a tree
void addPointAndNeighborsToTree(int pointIdx, const std::vector<std::vector<float>> points, std::vector<bool>& processed,
    std::vector<int>& cluster, float distanceTol, KdTree* tree)
{
    if (processed[pointIdx])
        return;

    processed[pointIdx] = true;
    cluster.push_back(pointIdx);

    std::vector<int> nearbyPoints = tree->search(points[pointIdx], distanceTol);
    for (int nearbyPointIdx : nearbyPoints) {
        if (!processed[nearbyPointIdx]) {
            addPointAndNeighborsToTree(nearbyPointIdx, points, processed, cluster, distanceTol, tree);
        }
    }
}

// main clustering function, returns a list of lists, each inner list containing the indices of one cluster
// only returns clusters with size ranging from minClusterSize to maxClusterSize
std::vector<std::vector<int>> EuclideanCluster::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol,
    int minClusterSize, int maxClusterSize)
{
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(), false);

    for (int pointIdx = 0; pointIdx < points.size(); pointIdx++) {
        if (!processed[pointIdx]) {
            std::vector<int> newCluster;
            addPointAndNeighborsToTree(pointIdx, points, processed, newCluster, distanceTol, tree);
            if (minClusterSize < newCluster.size() && newCluster.size() < maxClusterSize)
                clusters.push_back(newCluster);
        }
    }
    return clusters;
}
