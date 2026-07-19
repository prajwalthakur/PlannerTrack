#pragma once

#include "project_utils/geometry_utils.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <project_utils_msgs/msg/path_point.hpp>
#include <project_utils_msgs/msg/trajectory_point.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include <cmath>

namespace mpl::planning_utils
{
std::pair<bool, int32_t> findClosestIdxWithDistAngThr(
    const std::vector<geometry_msgs::msg::Pose> & poses,
    const geometry_msgs::msg::Pose & current_pose, double th_dist, double th_yaw);

int8_t getPathDirection(const std::vector<geometry_msgs::msg::Pose> & poses, double th_dist);

geometry_msgs::msg::Point transformToRelativeCoordinate2D(
    const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & origin);

double calcRadius(
    const geometry_msgs::msg::Point & target, const geometry_msgs::msg::Pose & current_pose);

double calcCurvature(
    const geometry_msgs::msg::Point & target, const geometry_msgs::msg::Pose & current_pose);

double convertCurvatureToSteeringAngle(double wheel_base, double kappa);

double calcLateralError2D(const geometry_msgs::msg::Point & line_s,
    const geometry_msgs::msg::Point & line_e, const geometry_msgs::msg::Point & point);
}  // namespace mpl::planning_utils
