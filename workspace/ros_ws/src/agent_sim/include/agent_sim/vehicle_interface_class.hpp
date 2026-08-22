/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
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

#include "motion_model_base/agent_model.hpp"
#include "motion_model_base/occupancy_grid_map.hpp"
#include "motion_model_base/vehicle_model_factory.hpp"
#include "project_utils/logger.hpp"
#include "project_utils/macros_expression.hpp"
#include "project_utils/main.hpp"
#include "project_utils_msgs/msg/eigen_vector.hpp"
#include "project_utils_msgs/msg/eigen_vector_stamped.hpp"
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

using EigenVector = project_utils_msgs::msg::EigenVector;
using EigenVectorStamped = project_utils_msgs::msg::EigenVectorStamped;


/**
 * \brief ROS2 node that simulates every agent in the scenario (ego + scripted
 * conflict agents) and publishes their state.
 *
 * Owns one \ref AgentModel plugin instance per agent (created via
 * \c VehicleModelFactory from `agents.yaml`), steps them all on a fixed-rate
 * timer, and publishes each agent's odometry/TF, lidar \c LaserScan, and
 * RViz visualization markers (true-shape body + collision footprint). Also
 * subscribes to `/map` to feed a shared \ref OccupancyGridMap into every
 * agent's \ref WorldSnapshot each step.
 *
 * Configuration is two-phase (\ref onConfigure then \ref onActivate) rather
 * than done in the constructor so that \c shared_from_this() is valid when
 * agent models are constructed (see \ref addAgents).
 */
class AgentInterface : public rclcpp::Node
{
  public:
	AgentInterface();

	/**
	 * \brief Load parameters/YAML configs, set up publishers/subscriptions
	 * and the static-obstacle list.
	 *
	 * Must run before \ref onActivate. Split out of the constructor because
	 * it needs a fully-constructed node.
	 */
	void onConfigure();

	/**
	 * \brief Create all agents (\ref addAgents), publish static TFs, and
	 * start the state-update/state-publish/lidar timers.
	 *
	 * Requires \ref onConfigure to have already run.
	 */
	void onActivate();

  private:
	struct AgentData
	{
		UniqueId id{"agent"};
		ptSharedPtr<AgentModel> model;

		// RGBA for this agent's true-shape CUBE marker, from agents.yaml's
		// per-agent "color: [r, g, b, a]" (falls back to this default --
		// the old hardcoded body color -- if an agent doesn't set one).
		std::array<float, 4> color{0.1f, 0.6f, 1.0f, 0.8f};

        rclcpp::Subscription<EigenVectorStamped>::SharedPtr controlSub;
		rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scanPub;
		// Respawns this agent at its configured init pose/velocity on
		// receipt (e.g. from a watchdog like intersection/scripts/
		// reset_state.py that trips when the agent leaves the scenario).
		rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr resetSub;

		// Cached from this agent's own sensor_params (YAML) for building
		// LaserScan headers -- not queryable from SensorModel itself, since
		// angle/range metadata is lidar-specific and doesn't belong on the
		// generic sensor interface.
		float lidarAngleMin{-3.14159265f};
		float lidarAngleMax{3.14159265f};
		float lidarRangeMin{0.05f};
		float lidarRangeMax{10.0f};
	};

	/// \brief Declared but not yet defined/called; parameter loading currently
	/// happens inline in \ref onConfigure instead.
	void loadRosParams();

	/**
	 * \brief Instantiate one \ref AgentModel plugin (via \c VehicleModelFactory)
	 * plus its ROS I/O (control subscription, scan publisher, reset
	 * subscription) for every agent listed in `agents.yaml`.
	 */
	void addAgents();

	/// \brief Timer callback: build this cycle's \ref WorldSnapshot and step every agent's model.
	void stateUpdateTimerCallback();
	/// \brief Timer callback: publish each agent's odometry/TF and RViz markers (body + collision footprint).
	void statePubTimerCallback();
	/// \brief Timer callback: publish each agent's simulated \c LaserScan.
	void lidarTimerCallback();

	/// \brief Broadcast the fixed lidar-link static transform for every agent.
	void publishStaticTFs();
	/// \brief Broadcast the fixed-frame -> agent base_link transform for one agent's current pose.
	void broadcastAgentTF(int agentNum, const stPose & pose);

	/**
	 * \brief Latched `/map` subscription callback; fills \ref mMapGrid.
	 *
	 * Fires once whenever map_server (re)publishes. \ref mMapGrid stays
	 * nullptr-guarded in \ref WorldSnapshot until this has run at least once.
	 */
	void mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  private:
	mpl::rclcpp_utils::Logger mLogger;
	std::unique_ptr<mpl::rclcpp_utils::Parameters> mParameters;
	std::unique_ptr<VehicleModelFactory> mVehicleFactory;
	std::vector<AgentData> mAgents;
	int mNumAgents{0};

	std::vector<ShapeDescriptor> mObstacleCircles;

	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mMapSub;
	OccupancyGridMap mMapGrid;  // mMapGrid.valid() == false until /map arrives

	rclcpp::TimerBase::SharedPtr mStateUpdateTimer;
	rclcpp::TimerBase::SharedPtr mStatePubTimer;
	rclcpp::TimerBase::SharedPtr mLidarTimer;

	std::unique_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> mStaticTfBroadcaster;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mMarkerPub;



	double mSimTimeStep{0.01};
	double mStatePublisherTimeStep{0.05};
	double mVisDt{0.1};
	std::string mFixedFrame{"world"};

	std::string mSimConfigFile;
	std::string mAgentsConfigFile;
	YAML::Node mSimConfig;
	YAML::Node mAgentsConfig;
};
