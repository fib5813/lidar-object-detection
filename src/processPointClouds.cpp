// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "ransac_impl/ransac_impl.h"
#include <unordered_set>
// #include "cluster.h"
#include "quiz/cluster/kdtree.h"


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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr 
cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::
SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) {

  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr groundPlane( new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr nonGroundPlane(new pcl::PointCloud<PointT>);
    // int k = 0;
    // for(size_t i = 0U; i < cloud->points.size(); i++){
    //     // std::cout << inliers->indices[1] << "  " << i << std::endl;
    //     if(i ==  inliers->indices[i]){
    //         groundPlane->push_back(cloud->points[inliers->indices[i]]) ;
    //         std::cout << "groundplane" << cloud->points[inliers->indices[i]] << std::endl;
    //     }
    //     else{
    //         nonGroundPlane->push_back(cloud->points[inliers->indices[i]]);
    //         // std::cout << cloud->points[inliers->indices[i]] << std::endl;
    //     }

    // }
    for(int index : inliers->indices){
        groundPlane->points.push_back(cloud->points[index]);
    
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*nonGroundPlane);

    
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(groundPlane, nonGroundPlane);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::
SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, double distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	// pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    
    ////////////////////////////// either
    // Create the segmentation object
    // pcl::SACSegmentation<pcl::PointXYZ> seg;
    // // Optional
    // seg.setOptimizeCoefficients (true);
    // // Mandatory
    // seg.setModelType (pcl::SACMODEL_PLANE);
    // seg.setMethodType (pcl::SAC_RANSAC);
    // seg.setDistanceThreshold (distanceThreshold);
  
    // seg.setInputCloud (cloud);
    
    
    // seg.segment (*inliers, *coefficients);
    // std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;


    ////////////////////////////// or
    Ransac R;
    R.setMaxIteration(maxIterations);
    R.setThreshold(distanceThreshold);
    std::unordered_set<int> inliers_ransac = R.RansacPlane(cloud);

    for(auto i : inliers_ransac){
        inliers->indices.push_back(i);
    }
    std::cout << "inliers count = " << inliers->indices.size() << "  " << inliers_ransac.size() << std::endl; 
    /////////////////////////////// end
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = 
    SeparateClouds(inliers,cloud);
    return segResult;
    
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::
Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    //create kd tree with point cloud
    // typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    KdTree* tree = new KdTree;
  
    for (int i=0; i< cloud->points.size(); i++){
        tree->insert(cloud->points[i],i); 
    }
    std::cout << "points size = " << cloud->points.size() << std::endl;
    
    // tree->setInputCloud(cloud);
    // std::vector<pcl::PointIndices> clusterIndices;
    // pcl::EuclideanClusterExtraction<PointT> ec;
    // ec.setClusterTolerance(clusterTolerance);
    // ec.setMinClusterSize(minSize);
    // ec.setMaxClusterSize(maxSize);
    // ec.setSearchMethod(tree);
    // ec.setInputCloud(cloud);
    // ec.extract(clusterIndices);

    //cluster kd tree elements
    std::vector<std::vector<int>> ECclusters = euclideanCluster(cloud->points, tree, clusterTolerance);
  	for (int i = 0; i < ECclusters.size(); i++){
        pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud (new pcl::PointCloud<pcl::PointXYZI>);
        if ((ECclusters[i].size() > minSize) && (ECclusters[i].size() < maxSize)){
            
            for (int p = 0; p <ECclusters[i].size(); p++){
                outCloud->points.push_back(cloud->points[p]);
            }
        }
        clusters.push_back(outCloud);
    }




    // for(pcl::PointIndices getIndices : clusterIndices){
    //     typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
    //     for (int index : getIndices.indices){
    //         cloudCluster->points.push_back(cloud->points[index]);
    //     }
    //     cloudCluster->width = cloudCluster->points.size();
    //     cloudCluster->height = 1;
    //     cloudCluster->is_dense = true;

    //     clusters.push_back(cloudCluster);
        
    // }


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