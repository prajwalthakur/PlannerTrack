// Author Prajwal Thakur 
#include "project_utils/pose_definition.hpp"

stPose::stPose(double x, double y , double z , double yaw )
{
    xCoord = x;
    yCoord = y;
    zCoord = z;
    this->yaw = yaw;
}

//////////////////////////////////////////////////////////////////////////

stPose::stPose(double x, double y , double z , double yaw, double  steerAngle)
{
    xCoord = x;
    yCoord = y;
    zCoord = z;
    this->yaw = yaw;
    steeringAngle = steerAngle;
}

//////////////////////////////////////////////////////////////////////////

void stPose::setCoord(double x, double y , double z , double yaw )
{
    xCoord = x;
    yCoord = y;
    zCoord = z;
    this->yaw = yaw;
}

//////////////////////////////////////////////////////////////////////////

template <typename T>
Eigen::Vector2<T> stPose::toEigenVector(const std::vector<T> &v) noexcept 
{ 
    return { v[0], v[1] }; 
}

//////////////////////////////////////////////////////////////////////////

Eigen::Vector2<float> stPose::toEigenVector(const std::shared_ptr<stPose> &v) noexcept 
{ 
    return { v->xCoord, v->yCoord}; 
}


