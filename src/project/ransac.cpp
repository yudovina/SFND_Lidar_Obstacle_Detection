/* \author Elena Yudovina */
// Simple RANSAC plane fitting
// borrows heavily from the quiz

#include "ransac.h"

template<typename PointT>
std::unordered_set<int> Ransac<PointT>::SACSegmentation(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
    for (int i = 0; i < maxIterations; i++) {
        std::unordered_set<int> inliers;

        // pick two points by adding them to the set, note a set will make sure they're distinct
        while (inliers.size() < 3) {
            // pick a random number from 0 to N-1: take rand() mod N
            // (rand() is a random number from 1 to infinity, more or less)
            inliers.insert(rand() % (cloud->points.size()));
        }

        // distance from (x,y,z) to plane spanned by (x1, y1, z1), (x2, y2, z2), and (x3, y3, z3):
        // first let (a,b,c) = (x-x1, y-y1, z-z1)
        // then project (a,b,c) onto the normal vector (n1, n2, n3)
        // to find the normal vector, take cross product of (x2-x1,y2-y1,z2-z1) and (x3-x1,y3-y1,z3-z1) and normalize
        float x1, x2, x3, y1, y2, y3, z1, z2, z3;
        auto itr = inliers.begin();
        auto pt1 = cloud->points[*itr];
        x1 = pt1.x;
        y1 = pt1.y;
        z1 = pt1.z;
        itr++;
        auto pt2 = cloud->points[*itr];
        x2 = pt2.x;
        y2 = pt2.y;
        z2 = pt2.z;
        itr++;
        auto pt3 = cloud->points[*itr];
        x3 = pt3.x;
        y3 = pt3.y;
        z3 = pt3.z;

        // normal vector
        float nx = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        float ny = - (x2 - x1) * (z3 - z1) + (z2 - z1) * (x3 - x1);
        float nz = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        float norm = sqrt(nx * nx + ny * ny + nz * nz);
        nx /= norm;
        ny /= norm;
        nz /= norm;

        for (int idx = 0; idx < cloud->points.size(); idx++) {

            // this is how you check if idx is in the set of inliers already
            if (inliers.count(idx) > 0)
                continue;

            auto pt = cloud->points[idx];
            float x = pt.x;
            float y = pt.y;
            float z = pt.z;

            float a = x - x1;
            float b = y - y1;
            float c = z - z1;

            float dist = fabs(a * nx + b * ny + c * nz);
            if (dist < distanceTol) {
                inliers.insert(idx);
            }
        }

        if (inliers.size() > inliersResult.size())
            inliersResult = inliers;
    }

	return inliersResult;
}