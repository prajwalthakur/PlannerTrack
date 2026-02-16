#include "lane_change_example/CreateLane.h"

////////////////////////////////////////////////////////////////////////////////

CreateLanes::CreateLanes(YAML::Node& rosParams):
            mRosParams(rosParams){};

////////////////////////////////////////////////////////////////////////////////

void CreateLanes::createStLanes(std::vector<std::vector<double>>& leftWaypoints, std::vector<std::vector<double>>& rightWaypoints)
{
    YAML::Node laneConfig = mRosParams["laneChangeExample"];
    YAML::Node vehConfig = mRosParams["vehicle1_param"];
    std::vector<double> initPoseLeft = \
        laneConfig["initPoseLeft"].as<std::vector<double>>();

    std::vector<double> finalPoseLeft = \
        laneConfig["finalPoseLeft"].as<std::vector<double>>();
    
    double laneWidth = 5*vehConfig["vehWidth"].as<double>();

    std::vector<double> initPoseRight = {initPoseLeft[0],initPoseLeft[1]-laneWidth,initPoseLeft[2]};
    std::vector<double> finalPoseRight = {finalPoseLeft[0],finalPoseLeft[1]-laneWidth,finalPoseLeft[2]};
    double waypointStep = laneConfig["waypointStep"].as<double>();
    double dx = finalPoseLeft[0] - initPoseLeft[0];
    double dy = finalPoseLeft[1] - initPoseLeft[1];

    double length = std::sqrt(dx*dx + dy*dy);

    int numWaypoints = static_cast<int>(length / waypointStep);

    
    leftWaypoints.clear();
    rightWaypoints.clear();
    leftWaypoints.push_back({initPoseLeft});
    rightWaypoints.push_back({initPoseRight});


    double dir_x = dx / length;
    double dir_y = dy / length;

    for(int i = 0; i <= numWaypoints; ++i)
    {
        double x = initPoseLeft[0] + i * waypointStep * dir_x;
        double y = initPoseLeft[1] + i * waypointStep * dir_y;

        leftWaypoints.push_back({x, y, 0.0});
        rightWaypoints.push_back({x, y - laneWidth, 0.0});
    }

}   