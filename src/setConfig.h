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

}