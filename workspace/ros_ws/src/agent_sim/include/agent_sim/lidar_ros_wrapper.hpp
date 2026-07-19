// Author Prajwal Thakur 
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "project_utils/parameter.hpp"
#include "lidar_sensor/lidar_sensor.hpp"

namespace lidar_sim
{

class LidarSensorWrapper : public rclcpp::Node
{
public:
  LidarSensorWrapper();
  ~LidarSensorWrapper() = default;

  void onConfigure();
  void onActivate();

private:
  // Per-agent bookkeeping: geometry (from config) + live pose (from odom)
  struct AgentEntry
  {
    bool  isRect{true};       // true → OBB, false → circle
    float hl{0.0f};           // half-length (OBB only)
    float hw{0.0f};           // half-width  (OBB only)
    float r{0.0f};            // radius      (circle only)
    // live pose — updated each odom callback
    float x{0.0f};
    float y{0.0f};
    float phi{0.0f};
  };

  void loadConfig();
  void publishStaticTFs();
  void timerCallback();

  // ── compute helpers ────────────────────────────────────────────────────────
  LidarSensor  mLidar;
  LidarParams  mLidarParams;
  float        mVisDt{0.1f};
  int          mNumAgents{0};

  // ── world model ────────────────────────────────────────────────────────────
  std::vector<AgentEntry>   mAgents;           // indexed 0…N-1
  std::vector<WorldCircle>  mObstacleCircles;  // static obstacles
  std::vector<WorldSegment> mWallSegments;     // corridor walls (static)

  // ── ROS I/O ────────────────────────────────────────────────────────────────
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> mOdomSubs;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> mScanPubs;
  rclcpp::TimerBase::SharedPtr mTimer;

  // agent_N_base_link → agent_N_lidar_link (identity, published once at startup)
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> mStaticTfBroadcaster;

  // ── utilities ──────────────────────────────────────────────────────────────
  std::unique_ptr<mpl::rclcpp_utils::Parameters> mParameters;
};

}  // namespace lidar_sim
