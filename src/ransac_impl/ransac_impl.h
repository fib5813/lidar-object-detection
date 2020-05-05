#ifndef RANSAC_IMPL_H
#define RANSAC_IMPL_H

#include <unordered_set>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>

class Ransac{
public:
    Ransac(){
        maxIteration = 100;
        threshold = 0.05;
    };
    Ransac(int& iterations, double& thresh){
        maxIteration = iterations;
        threshold = thresh;
    };
    void setMaxIteration(int in){maxIteration = in;};
    void setThreshold(double in){threshold = in;};
    void run();
    std::unordered_set<int> RansacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
								 double maxIterations, double distanceTol);
    
    std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);



private:
    int maxIteration;
    double threshold;

};

#endif