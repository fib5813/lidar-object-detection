/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "setConfig.h"
#include "ransac_impl/ransac_impl.h"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        // car1.render(viewer);
        // car2.render(viewer);
        // car3.render(viewer);
    }

    return cars;
}


// void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
// {
//     // ----------------------------------------------------
//     // -----Open 3D viewer and display simple highway -----
//     // ----------------------------------------------------
    

//     Config config;
//     setConfiguration(config);

//     // RENDER OPTIONS
//     bool renderScene = true;
//     std::vector<Car> cars = initHighway(renderScene, viewer);
//     double groundSlope = config.kGroundSlope;
//     double minDistance = config.kMinDistance;
//     double maxDistance = config.kMaxDistance;
//     double resolution = config.kResolution;
//     double sderr = config.kSderr;
//     double numLayers = config.kNumLayers;
//     double steepestAngle = (pi/180)*config.kSteepestAngle;
//     double angleRange = (pi/180) * config.kAngleRange;
//     double horizontalAngleInc = (pi/180) * config.kHorizontalAngleInc;
//     double numIterationsPlaneDetection = config.kNumIterationsPlaneDetection;
//     double distanceThresholdPlaneDetection = config.kDistanceThresholdPlaneDetection;
//     float clusterTolerance = config.kClusterTolerance; 
//     int clusterMinSize = config.kClusterMinSize;
//     int clusterMaxSize = config.kClusterMaxSize;
//     // std::cout << groundSlope << "  " << minDistance << "  " << maxDistance << "  " << resolution << "  " << sderr << "  " << numLayers << "  " << steepestAngle << "  " << angleRange << "  " << horizontalAngleInc << std::endl;


//     // TODO:: Create lidar sensor 
//     Lidar* lidar = new Lidar(cars, groundSlope);
//     lidar->modifyLidarParams(minDistance, 
//                              maxDistance, 
//                              groundSlope, 
//                              resolution, 
//                              sderr,
//                              numLayers,
//                              steepestAngle,
//                              angleRange,
//                              horizontalAngleInc);

//     pcl::PointCloud<pcl::PointXYZ>::Ptr lidarOutput = lidar->scan();

//     // TODO:: Create point processor

//     ProcessPointClouds<pcl::PointXYZ> pointCloudProcessor;
//     std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> 
//     separatedClouds = pointCloudProcessor.SegmentPlane(lidarOutput, numIterationsPlaneDetection, distanceThresholdPlaneDetection);
//     std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = 
//     pointCloudProcessor.Clustering(separatedClouds.second, clusterTolerance, clusterMinSize, clusterMaxSize);
    
//     std::vector<Color> colors = {Color(1.0,0,0), Color(1.0,1.0,0), Color(0,0,1.0)};

//     for(auto i=0; i<clusters.size(); i++){
//         renderPointCloud(viewer, clusters[i], std::to_string(i), colors[i]);

//         Box box = pointCloudProcessor.BoundingBox(clusters[i]);
//         renderBox(viewer, box, i);
//     }
//     // renderRays(viewer, lidar->position, lidarOutput);
//     // renderPointCloud(viewer, separatedClouds.first, "ground", Color(1, 0, 0));
//     // renderPointCloud(viewer, separatedClouds.second, "object cloud", Color(0,1,0));
    
    
  
// }


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

    Config1 c;
    setConfiguration1(c);

    // RENDER OPTIONS
    float filterRes = c.filterRes;

    // filter cloud
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud;
    filterCloud = pointProcessorI->FilterCloud(inputCloud, c.filterRes , Eigen::Vector4f (c.minx, c.miny, c.minz, 1), Eigen::Vector4f ( c.maxx, c.maxy, c.maxz, 1));
    // renderPointCloud(viewer,filterCloud,"filterCloud");
    // renderPointCloud(viewer,inputCloud,"inputCloud");

    //segment road plane
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> 
    separatedClouds = pointProcessorI->SegmentPlane(filterCloud, c.numIterationsPlaneDetection, c.distanceThresholdPlaneDetection);
    // renderPointCloud(viewer, separatedClouds.first, "ground", Color(1, 0, 0));
    renderPointCloud(viewer, separatedClouds.second, "objects", Color(0,1,0));
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = 
    pointProcessorI->Clustering(separatedClouds.second, c.clusterTolerance, c.clusterMinSize, c.clusterMaxSize);
    
    std::vector<Color> colors = {Color(1.0,0,0), Color(1.0,1.0,0), Color(0,0,1.0)};

    for(auto i=0; i<clusters.size(); i++){
        renderPointCloud(viewer, clusters[i], std::to_string(i), colors[0]);

        Box box = pointProcessorI->BoundingBox(clusters[i]);
        renderBox(viewer, box, i);
    }

    delete pointProcessorI;
}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);

    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}