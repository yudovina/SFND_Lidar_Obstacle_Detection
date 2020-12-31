// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    pcl::VoxelGrid<PointT> vg;
    vg.setDownsampleAllData(true);
    vg.setLeafSize(Eigen::Vector4f(filterRes,filterRes,filterRes,0));
    vg.setInputCloud(cloud);
    typename pcl::PointCloud<PointT>::Ptr voxelGridCloud(new pcl::PointCloud<PointT>());
    vg.filter(*voxelGridCloud);

    pcl::CropBox<PointT> cb;
    cb.setMin(Eigen::Vector4f(minPoint));
    cb.setMax(Eigen::Vector4f(maxPoint));
    cb.setInputCloud(voxelGridCloud);
    typename pcl::PointCloud<PointT>::Ptr croppedCloud(new pcl::PointCloud<PointT>());
    cb.filter(*croppedCloud);

    // removing car roof requires jumping through hoops:
    // crop box will extract the selected indices into std::vector<int> (if you ask it to allow it at all),
    // while the Extract (to take the complement) requires a pcl::PointIndices::Ptr
    pcl::CropBox<PointT> roof;
    roof.setMin(Eigen::Vector4f(-3, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(3, 1.7, 0, 1));
    roof.setInputCloud(croppedCloud);
    std::vector<int> roofIndices;
    roof.filter(roofIndices);

    pcl::PointIndices::Ptr roofPointIndices(new pcl::PointIndices());
    for (int i : roofIndices)
        roofPointIndices->indices.push_back(i);

    pcl::ExtractIndices<PointT> ex;
    ex.setInputCloud(croppedCloud);
    ex.setIndices(roofPointIndices);
    ex.setNegative(true);
    typename pcl::PointCloud<PointT>::Ptr noRoofCloud(new pcl::PointCloud<PointT>());
    ex.filter(*noRoofCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return noRoofCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud (new pcl::PointCloud<PointT>());

    for (int idx : inliers->indices)
        planeCloud->points.push_back(cloud->points[idx]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacleCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    Ransac<PointT> ransac;
    std::unordered_set<int> inlierIndicesSet = ransac.SACSegmentation(cloud, maxIterations, distanceThreshold);
    if (inlierIndicesSet.size() == 0)
        std::cout << "Failed to fit a plane to the dataset! (This should be impossible if it has >=3 points)" << std::endl;

    // convert unordered_set to pcl::PointIndices
    std::vector<int> inlierIndices;
    inlierIndices.insert(inlierIndices.end(), inlierIndicesSet.begin(), inlierIndicesSet.end());
    pcl::PointIndices::Ptr inliers{ new pcl::PointIndices };
    inliers->indices = inlierIndices;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Create KdTree to search through
    KdTree *tree = new KdTree();
    std::vector<std::vector<float>> cloudPoints;
    for (int i = 0; i < cloud->points.size(); i++)
    {
        PointT p = cloud->points[i];
        std::vector<float> pVec;
        pVec.push_back(p.x);
        pVec.push_back(p.y);
        pVec.push_back(p.z);
        tree->insert(pVec, i);
        cloudPoints.push_back(pVec);
    }

    EuclideanCluster ec;
    std::vector<std::vector<int>> cluster_indices = ec.euclideanCluster(cloudPoints, tree, clusterTolerance, minSize, maxSize);

    for (std::vector<int> clusterIxs : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
        for (int ix : clusterIxs)
            clusterCloud->points.push_back(cloud->points[ix]);

        clusters.push_back(clusterCloud);
    }
        
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}