#pragma once
#include <Eigen/Dense>
#include "core/CoreCollection.h"
struct stPose
{   
    public:
        stPose()=default;
        stPose(double x, double y , double z , double yaw );
        stPose(double x, double y , double z , double yaw, double  steerAngle);
        double xCoord{0.0};
        double yCoord{0.0};
        double zCoord{0.0};
        double steeringAngle{0.0};
        double yaw{0.0};
        void setCoord(double x, double y , double z , double yaw );
        template <typename T>
        Eigen::Vector2<T> toEigenVector(const std::vector<T> &v) noexcept;
        Eigen::Vector2<float> toEigenVector(const ptSharedPtr<stPose> &v) noexcept;

};