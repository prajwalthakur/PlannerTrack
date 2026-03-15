#pragma once

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mpl_geometry_utils
{
    template<class T>
    geometry_msgs::msg::Point getPoint(const T& p)
    {
        return geometry_msgs::build<geometry_msgs::msg::Point>().x(p.x).y(p.y).z(p.z);
    }

    template <>
    inline geometry_msgs::msg::Point get_point(const geometry_msgs::msg::Point & p)
    {
    return p;
    }
    
    template <>
    inline geometry_msgs::msg::Point get_point(const geometry_msgs::msg::Pose & p)
    {
    return p.position;
    }

    template <>
    inline geometry_msgs::msg::Point get_point(const geometry_msgs::msg::PoseStamped & p)
    {
    return p.pose.position;
    }
    
    template <>
    inline geometry_msgs::msg::Point get_point(const autoware_planning_msgs::msg::PathPoint & p)
    {
    return p.pose.position;
    }
}