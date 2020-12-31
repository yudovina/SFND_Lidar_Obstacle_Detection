/* \author Elena Yudovina */
// Simple RANSAC plane fitting
// borrows heavily from the quiz

#ifndef RANSAC_H_
#define RANSAC_H_

#include <unordered_set>
#include <pcl/common/common.h>
//#include <pcl/common/transforms.h>

template<typename PointT>
class Ransac {
public:
    //constructor
    Ransac() {};
    //deconstructor
    ~Ransac() {};

	std::unordered_set<int> SACSegmentation(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);
};

#endif /* RANSAC_H_ */