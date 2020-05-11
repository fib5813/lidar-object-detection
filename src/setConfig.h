#include "json.hpp"
#include "fstream"


struct Config {
        double kGroundSlope;
        double kMinDistance;
        double kMaxDistance;
        double kResolution;
        double kSderr;
        double kNumLayers;
        double kSteepestAngle;
        double kAngleRange;
        double kHorizontalAngleInc;
        double kNumIterationsPlaneDetection;
        double kDistanceThresholdPlaneDetection;
        float kClusterTolerance;
        int kClusterMinSize;
        int kClusterMaxSize;
    };

struct Config1{
    float filterRes;
    float minx;
    float miny;
    float minz;
    float maxx;
    float maxy;
    float maxz;
    float numIterationsPlaneDetection;
    float distanceThresholdPlaneDetection;
};

void setConfiguration(Config& c){
    
    std::ifstream ifs("/home/user/projects/lidar-object-detection/src/config.json");
    nlohmann::json j = nlohmann::json::parse(ifs);
    
    c.kGroundSlope = j.at("groundSlope");
    c.kMinDistance = j.at("minDistance");
    c.kMaxDistance = j.at("maxDistance");
    c.kResolution = j.at("resolution");
    c.kSderr = j.at("sderr");
    c.kNumLayers = j.at("numLayers");
    c.kSteepestAngle = j.at("steepestAngle");
    c.kAngleRange = j.at("angleRange");
    c.kHorizontalAngleInc = j.at("horizontalAngleInc");
    c.kNumIterationsPlaneDetection = j.at("numIterationsPlaneDetection");
    c.kDistanceThresholdPlaneDetection = j.at("distanceThresholdPlaneDetection");
    c.kClusterTolerance = j.at("clusterTolerance");
    c.kClusterMinSize = j.at("clusterMinSize");
    c.kClusterMaxSize = j.at("clusterMaxSize");
    

}

void setConfiguration1(Config1& c){
    std::ifstream ifs("/home/user/projects/lidar-object-detection/src/config1.json");
    nlohmann::json j = nlohmann::json::parse(ifs);
    
    c.filterRes = j.at("filterRes");
    c.minx = j.at("minx");
    c.miny = j.at("miny");
    c.minz = j.at("minz");
    c.maxx = j.at("maxx");
    c.maxy = j.at("maxy");
    c.maxz = j.at("maxz");
    c.numIterationsPlaneDetection = j.at("numIterationsPlaneDetection");
    c.distanceThresholdPlaneDetection = j.at("distanceThresholdPlaneDetection");
}