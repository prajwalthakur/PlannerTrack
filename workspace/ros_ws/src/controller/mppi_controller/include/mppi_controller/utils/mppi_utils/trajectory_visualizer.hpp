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

/**
 * \brief RViz visualization for \c Optimizer's rollouts: the optimized
 * trajectory as a `nav_msgs::Path` + sphere markers, and (optionally) the
 * full candidate-rollout batch colored by normalized cost (collisions
 * highlighted separately).
 */
class TrajectoryVisualizer
{
public:
  TrajectoryVisualizer() = default;

  /// \brief Create the marker/path publishers under \p node.
  void configure(
    rclcpp::Node & node,
    const std::string & frame_id,
    Parameters * params,
    const std::string & name);

  /// \brief Stage the optimized trajectory for the next \ref visualize call (builds sphere markers + a `nav_msgs::Path`).
  void add(
    const Eigen::ArrayXXf & trajectory,
    const std::string & marker_namespace,
    const builtin_interfaces::msg::Time & stamp);

  /// \brief Stage all candidate rollouts, colored by cost gradient, for the next \ref visualize call.
  void add(
    const models::Trajectories & trajectories,
    const Eigen::ArrayXf & costs,
    const std::vector<bool> & collisions,
    const builtin_interfaces::msg::Time & stamp);

  /// \brief Publish everything staged since the last \ref reset via the two \ref add overloads.
  void visualize();
  /// \brief Clear staged markers/path, ready for the next cycle.
  void reset();

protected:
  /// \brief Build one rollout's marker, colored between "cost-free" and "max-cost" (or a distinct collision color).
  void addCostColoredTrajectory(
    size_t trajectory_idx,
    const models::Trajectories & trajectories,
    float normalized_cost,
    bool in_collision,
    const builtin_interfaces::msg::Time & stamp);

  /// \brief Map a cost normalized to `[0, 1]` to an RGBA color gradient.
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
