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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

using EigenVector = project_utils_msgs::msg::EigenVector;

// // Pull lidar types into global scope
// using lidar_sim::LidarParams;
// using lidar_sim::LidarSensor;
// using lidar_sim::WorldCircle;
// using lidar_sim::WorldOBB;
// using lidar_sim::WorldSegment;

// ── Simple PID with output clamp ──────────────────────────────────────────────
// struct PidController
// {
// 	double kp{1.0}, ki{0.0}, kd{0.0};
// 	double out_min{-1e9}, out_max{1e9};
// 	double integral{0.0};
// 	double prev_error{0.0};

// 	double compute(double error, double dt)
// 	{
// 		integral += error * dt;
// 		const double derivative = (dt > 1e-9) ? (error - prev_error) / dt : 0.0;
// 		prev_error = error;
// 		return std::clamp(kp * error + ki * integral + kd * derivative, out_min, out_max);
// 	}

// 	void reset()
// 	{
// 		integral = 0.0;
// 		prev_error = 0.0;
// 	}
// };

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

		// std::shared_ptr<SingleTrackDynStateModel> stateModel;
		// std::shared_ptr<RectangularGeometry> geomModel;
		// std::shared_ptr<EllipseCollisionFootPrint> footprint;

		// OBB half-extents for lidar
		float hl{0.0f}, hw{0.0f};

		// Live pose for lidar ray-casting
		float x{0.0f}, y{0.0f}, phi{0.0f};

		// Subscribes to a raw control input, forwarded straight to
		// AgentModel::updateCommandedControl() -- no PID loop here, that
		// belongs to the controller/ package family.
        rclcpp::Subscription<EigenVector>::SharedPtr controlSub;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;
		// rclcpp::Publisher<project_utils_msgs::msg::LaneId>::SharedPtr laneLocPub;
		// rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePub;
		rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scanPub;
	};

	void loadRosParams();
	void addAgents();

	void stateUpdateTimerCallback();  // step every agent's model, then refresh the world snapshot
	void statePubTimerCallback();  // odom + TF
	// void lidarTimerCallback();  // LaserScan

	void publishStaticTFs();
	void broadcastAgentTF(int agentNum, const stPose & pose);

  private:
	mpl::rclcpp_utils::Logger mLogger;
	std::unique_ptr<mpl::rclcpp_utils::Parameters> mParameters;
	std::unique_ptr<VehicleModelFactory> mVehicleFactory;
	std::vector<AgentData> mAgents;
	int mNumAgents{0};

	// LidarSensor mLidar;
	// LidarParams mLidarParams;

	// float mVisDt{0.1f};
	std::vector<ShapeDescriptor> mObstacleCircles;
	// std::vector<WorldSegment> mWallSegments;
	// GraphLocalization mGraphLoc;

	rclcpp::TimerBase::SharedPtr mStateUpdateTimer;
	rclcpp::TimerBase::SharedPtr mStatePubTimer;
	rclcpp::TimerBase::SharedPtr mLidarTimer;

	std::unique_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> mStaticTfBroadcaster;

	// // PID gains (shared across agents, loaded from params)
	// double mKpV{2.0}, mKiV{0.0}, mKdV{0.05};
	// double mKpW{2.0}, mKiW{0.0}, mKdW{0.05};
	// double mAccMax{3.0}, mAlphaMax{3.5};

	double mSimTimeStep{0.01};
	double mStatePublisherTimeStep{0.05};
	std::string mFixedFrame{"world"};

	std::string mSimConfigFile;
	std::string mAgentsConfigFile;
	YAML::Node mSimConfig;
	YAML::Node mAgentsConfig;
};
