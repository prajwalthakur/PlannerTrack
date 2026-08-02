#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "mppi_controller/models/trajectory.hpp"
#include "mppi_controller/parameters.hpp"
#include "mppi_controller/utils/types.hpp"

namespace controller::mppi_controller
{

class TrajectoryVisualizer
{
public:
  TrajectoryVisualizer() = default;

  void configure(
    rclcpp::Node & node,
    const std::string & frame_id,
    Parameters * params,
    const std::string & name);

  // Add optimal trajectory: builds sphere markers + nav path
  void add(
    const Eigen::ArrayXXf & trajectory,
    const std::string & marker_namespace,
    const builtin_interfaces::msg::Time & stamp);

  // Add all candidate trajectories colored by cost gradient
  void add(
    const models::Trajectories & trajectories,
    const Eigen::ArrayXf & costs,
    const std::vector<bool> & collisions,
    const builtin_interfaces::msg::Time & stamp);

  void visualize();
  void reset();

protected:
  void addCostColoredTrajectory(
    size_t trajectory_idx,
    const models::Trajectories & trajectories,
    float normalized_cost,
    bool in_collision,
    const builtin_interfaces::msg::Time & stamp);

  static std_msgs::msg::ColorRGBA costToColor(float normalized);

  std::string frame_id_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectories_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimal_path_pub_;

  std::unique_ptr<nav_msgs::msg::Path> optimal_path_;
  std::unique_ptr<visualization_msgs::msg::MarkerArray> points_;
  int marker_id_{0};

  size_t trajectory_step_{5};
  size_t time_step_{3};
};

}  // namespace controller::mppi_controller
