#pragma once
#include <chrono>
#include "core/CoreCollection.h"
#include "utils/UtilsCollection.h"
#include <cmath>
#include <yaml-cpp/yaml.h>


class CreateLanes
{
    public:
        CreateLanes()=default;
        CreateLanes(YAML::Node& rosParams);
        ~CreateLanes()=default;
        void createStLanes(std::vector<std::vector<double>>& leftWaypoints, std::vector<std::vector<double>>& rightWaypoints);
    private:
        YAML::Node mRosParams;

};
