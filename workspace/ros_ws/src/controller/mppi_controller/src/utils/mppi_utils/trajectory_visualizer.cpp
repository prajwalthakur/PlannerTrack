#include "mppi_controller/utils/mppi_utils/trajectory_visualizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace controller::mppi_controller
{

// ── Inline marker helpers ──────────────────────────────────────────────────

static geometry_msgs::msg::Pose makePose(float x, float y, float z)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation.w = 1.0;
  return p;
}

static geometry_msgs::msg::Vector3 makeScale(float x, float y, float z)
{
  geometry_msgs::msg::Vector3 s;
  s.x = x; s.y = y; s.z = z;
  return s;
}

static std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r; c.g = g; c.b = b; c.a = a;
  return c;
}

static visualization_msgs::msg::Marker makeMarker(
  int id,
  const geometry_msgs::msg::Pose & pose,
  const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color,
  const std::string & frame_id,
  const std::string & ns)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.ns    = ns;
  m.id    = id;
  m.type  = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose  = pose;
  m.scale = scale;
  m.color = color;
  return m;
}

// ── TrajectoryVisualizer ───────────────────────────────────────────────────

void TrajectoryVisualizer::configure(
  rclcpp::Node & node,
  const std::string & frame_id,
  Parameters * params,
  const std::string & name)
{
  frame_id_ = frame_id;

  if (params) {
    auto getParam = params->getParamGetter(name + ".TrajectoryVisualizer");
    int traj_step = 5, time_step_val = 3;
    getParam(traj_step,     "trajectory_step", 5);
    getParam(time_step_val, "time_step",       3);
    trajectory_step_ = static_cast<size_t>(traj_step);
    time_step_       = static_cast<size_t>(time_step_val);
  }

  trajectories_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>(
      "mppi/candidate_trajectories", 10);

  optimal_path_pub_ =
    node.create_publisher<nav_msgs::msg::Path>(
      "mppi/optimal_path", 10);

  reset();
}

// ── Optimal trajectory (sphere markers + nav path) ────────────────────────

void TrajectoryVisualizer::add(
  const Eigen::ArrayXXf & trajectory,
  const std::string & marker_namespace,
  const builtin_interfaces::msg::Time & stamp)
{
  if (optimal_path_pub_->get_subscription_count() == 0 &&
    trajectories_publisher_->get_subscription_count() == 0)
  {
    return;
  }

  const size_t size = static_cast<size_t>(trajectory.rows());
  if (size == 0) { return; }

  optimal_path_->header.stamp    = stamp;
  optimal_path_->header.frame_id = frame_id_;

  for (size_t i = 0; i < size; ++i) {
    const float component = static_cast<float>(i) / static_cast<float>(size);

    auto pose  = makePose(trajectory(i, 0), trajectory(i, 1), 0.06f);
    auto scale = (i != size - 1) ?
      makeScale(0.03f, 0.03f, 0.07f) : makeScale(0.07f, 0.07f, 0.09f);
    auto color  = makeColor(0.f, component, component, 1.f);
    auto marker = makeMarker(marker_id_++, pose, scale, color, frame_id_, marker_namespace);
    points_->markers.push_back(std::move(marker));

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = frame_id_;
    pose_stamped.pose            = pose;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, static_cast<double>(trajectory(i, 2)));
    pose_stamped.pose.orientation = tf2::toMsg(q);
    optimal_path_->poses.push_back(pose_stamped);
  }
}

// ── Candidate trajectories colored by cost ────────────────────────────────

void TrajectoryVisualizer::add(
  const models::Trajectories & trajectories,
  const Eigen::ArrayXf & costs,
  const std::vector<bool> & collisions,
  const builtin_interfaces::msg::Time & stamp)
{
  if (trajectories_publisher_->get_subscription_count() == 0) { return; }

  const size_t n_rows = static_cast<size_t>(trajectories.x.rows());
  if (n_rows == 0 || costs.size() == 0 ||
    static_cast<size_t>(costs.size()) < n_rows)
  {
    return;
  }

  // Normalize costs excluding collision trajectories for better gradient resolution
  float min_val = std::numeric_limits<float>::max();
  float max_val = std::numeric_limits<float>::lowest();
  for (Eigen::Index k = 0; k < costs.size(); ++k) {
    if (!collisions.empty() && static_cast<size_t>(k) < collisions.size() && collisions[k]) {
      continue;
    }
    if (costs(k) < min_val) { min_val = costs(k); }
    if (costs(k) > max_val) { max_val = costs(k); }
  }
  if (max_val < min_val) {
    min_val = costs.minCoeff();
    max_val = costs.maxCoeff();
  }
  const float range = max_val - min_val;

  for (size_t i = 0; i < n_rows; i += trajectory_step_) {
    const float norm = (range > 0.0f) ? (costs(i) - min_val) / range : 0.0f;
    const bool in_collision =
      !collisions.empty() && i < collisions.size() && collisions[i];
    addCostColoredTrajectory(i, trajectories, norm, in_collision, stamp);
  }
}

void TrajectoryVisualizer::addCostColoredTrajectory(
  size_t trajectory_idx,
  const models::Trajectories & trajectories,
  float normalized_cost,
  bool in_collision,
  const builtin_interfaces::msg::Time & stamp)
{
  const size_t n_cols = static_cast<size_t>(trajectories.x.cols());

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp    = stamp;
  marker.ns     = "Candidate Trajectories";
  marker.id     = marker_id_++;
  marker.type   = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.01;
  marker.color   = in_collision ?
    makeColor(1.0f, 0.0f, 1.0f, 0.6f) :  // magenta for collisions
    costToColor(normalized_cost);

  marker.points.reserve(n_cols / time_step_ + 1);
  for (size_t j = 0; j < n_cols; j += time_step_) {
    geometry_msgs::msg::Point pt;
    pt.x = trajectories.x(trajectory_idx, j);
    pt.y = trajectories.y(trajectory_idx, j);
    pt.z = 0.03;
    marker.points.push_back(pt);
  }

  points_->markers.push_back(std::move(marker));
}

std_msgs::msg::ColorRGBA TrajectoryVisualizer::costToColor(float normalized)
{
  normalized = std::clamp(normalized, 0.0f, 1.0f);
  float r, g;
  if (normalized < 0.5f) {
    r = 2.0f * normalized;
    g = 1.0f;
  } else {
    r = 1.0f;
    g = 2.0f * (1.0f - normalized);
  }
  return makeColor(r, g, 0.0f, 0.8f);
}

void TrajectoryVisualizer::reset()
{
  marker_id_    = 0;
  points_       = std::make_unique<visualization_msgs::msg::MarkerArray>();
  optimal_path_ = std::make_unique<nav_msgs::msg::Path>();
}

void TrajectoryVisualizer::visualize()
{
  if (trajectories_publisher_->get_subscription_count() > 0) {
    trajectories_publisher_->publish(std::move(points_));
  }
  if (optimal_path_pub_->get_subscription_count() > 0) {
    optimal_path_pub_->publish(std::move(optimal_path_));
  }
  reset();
}

}  // namespace controller::mppi_controller
