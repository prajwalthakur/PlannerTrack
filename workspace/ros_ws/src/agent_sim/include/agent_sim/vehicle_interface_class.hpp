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
#include "motion_model_base/vehicle_model_factory.hpp"
#include "project_utils/logger.hpp"
#include "project_utils/macros_expression.hpp"
#include "project_utils/main.hpp"
#include "project_utils_msgs/msg/eigen_vector.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

using EigenVector = project_utils_msgs::msg::EigenVector;


class AgentInterface : public rclcpp::Node
{
  public:
	AgentInterface();
	void onConfigure();
	void onActivate();

  private:
	struct AgentData
	{
		UniqueId id{"agent"};
		ptSharedPtr<AgentModel> model;


        rclcpp::Subscription<EigenVector>::SharedPtr controlSub;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;
		rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scanPub;

		// Cached from this agent's own sensor_params (YAML) for building
		// LaserScan headers -- not queryable from SensorModel itself, since
		// angle/range metadata is lidar-specific and doesn't belong on the
		// generic sensor interface.
		float lidarAngleMin{-3.14159265f};
		float lidarAngleMax{3.14159265f};
		float lidarRangeMin{0.05f};
		float lidarRangeMax{10.0f};
	};

	void loadRosParams();
	void addAgents();

	void stateUpdateTimerCallback();  // step every agent's model, then refresh the world snapshot
	void statePubTimerCallback();  // odom + TF
	void lidarTimerCallback();  // LaserScan

	void publishStaticTFs();
	void broadcastAgentTF(int agentNum, const stPose & pose);

  private:
	mpl::rclcpp_utils::Logger mLogger;
	std::unique_ptr<mpl::rclcpp_utils::Parameters> mParameters;
	std::unique_ptr<VehicleModelFactory> mVehicleFactory;
	std::vector<AgentData> mAgents;
	int mNumAgents{0};

	std::vector<ShapeDescriptor> mObstacleCircles;


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
