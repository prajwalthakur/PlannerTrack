#pragma once
/**
 * \file
 * \brief Short local aliases for the `nav2_costmap_2d` types/constants
 * `mppi_controller` uses (costmap, collision checker, inflation layer),
 * so the rest of the package doesn't spell out the full `nav2_costmap_2d::`
 * namespace everywhere.
 */
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/inflation_layer.hpp>
using CostMapRos = nav2_costmap_2d::Costmap2DROS;
using CostMap =  nav2_costmap_2d::Costmap2D;


constexpr auto INSCRIBED_INFLATED_OBSTACLE = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

constexpr auto LETHAL_OBSTACLE = nav2_costmap_2d::LETHAL_OBSTACLE;

constexpr auto NO_INFORMATION = nav2_costmap_2d::NO_INFORMATION;

using CollisionChecker = nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>;


using InflationLayer = nav2_costmap_2d::InflationLayer;


// #ifdef ROS2
// #include "rclcpp/rclcpp.hpp"
// template<typename T>
// using PublisherT = typename rclcpp::Publisher<T>::SharedPtr;
// #else
// #include "ros/ros.h"
// template<typename T>
// using PublisherT = ros::Publisher;
// #endif