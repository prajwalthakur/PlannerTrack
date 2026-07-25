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
#include "pid_controller/simple_pid.hpp"
#include "project_utils/logger.hpp"
#include "project_utils/parameter.hpp"
#include "project_utils/polling_subscriber.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

// msgs
#include "nav_msgs/msg/odometry.hpp"
// custom msgs
#include "project_utils_msgs/msg/control.hpp"
#include "project_utils_msgs/msg/eigen_vector_stamped.hpp"
#include "project_utils_msgs/msg/steering_report.hpp"

namespace mpl::control::pid_controller
{
namespace mpl_utils = ::mpl_rclcpp_utils;

// Low-level actuator tracking loop, independent of and downstream from
// trajectory_follower_node::Controller. Controller publishes a target
// velocity/steering-angle Control at ~controller_frequency (e.g. 50 Hz);
// this node closes the loop against measured odometry/steering feedback at
// its own, typically much higher, rate and publishes the resulting
// acceleration/steering-rate actuator command that agent_sim (or real
// hardware) actually consumes.
class PidControllerNode : public rclcpp::Node
{
  public:
	explicit PidControllerNode(const rclcpp::NodeOptions & nodeOptions);
	virtual ~PidControllerNode() {}

  private:
	// Build this agent's vehicle model, purely so DynamicModel::
	// packAccelSteerRate() (vehicle-specific input-vector layout) can be
	// reused instead of hardcoded here.
	void createAgentModel();
	// Poll the three subscribers; returns true only once all three have
	// delivered at least one message.
	bool processData();
	// True if the last received reference control is older than
	// mTimeoutThrSec -- in that case we must not keep commanding a stale
	// setpoint.
	bool isTimeOut() const;
	void callbackTimerControl();

  private:
	// logger
	mpl::rclcpp_utils::Logger mLogger;
	// parameters
	std::unique_ptr<mpl::rclcpp_utils::Parameters> mParameters;

	rclcpp::TimerBase::SharedPtr mTimerControl;
	double mTimeoutThrSec{0.5};
	// 1/controller_frequency -- passed explicitly into PIDController::tick(dt)
	// so ki/kd stay in real per-second units regardless of rate.
	double mCtrlPeriodSec{0.01};

	// Which agent this node drives -- same sim_config_file/agents_config_file
	// convention as trajectory_follower_node::Controller, so the same YAML
	// blocks (and hence the same vehicle model) are used consistently.
	std::string mSimConfigFile;
	std::string mAgentsConfigFile;
	int mAgentNumber{0};

	std::unique_ptr<VehicleModelFactory> mVehicleFactory;
	ptSharedPtr<AgentModel> mAgent;

	// Two independent PID loops, ticked every callbackTimerControl() call --
	// longitudinal: velocity error -> acceleration.
	// lateral: steering-angle error -> steering rate.
	PIDController<double> mLonPid;
	PIDController<double> mLatPid;
	double mComputedAcc{0.0};
	double mComputedSv{0.0};

	// Subscribers
	mpl_utils::InternalProcessPollingSubscriber<nav_msgs::msg::Odometry> mSubOdometry{
	    this, "~/input/current_odometry"};
	mpl_utils::InternalProcessPollingSubscriber<project_utils_msgs::msg::SteeringReport>
	    mSubSteering{this, "~/input/current_steering"};
	mpl_utils::InternalProcessPollingSubscriber<project_utils_msgs::msg::Control> mSubControl{
	    this, "~/input/control_cmd"};

	nav_msgs::msg::Odometry::ConstSharedPtr mCurrentOdometryPtr;
	project_utils_msgs::msg::SteeringReport::ConstSharedPtr mCurrentSteeringPtr;
	project_utils_msgs::msg::Control::ConstSharedPtr mCurrentControlPtr;

	// Publisher: the actual low-level actuator command agent_sim consumes.
	rclcpp::Publisher<project_utils_msgs::msg::EigenVectorStamped>::SharedPtr mControlCmdPub;

	static constexpr double loggerThrottleInterval = 5000;  // in ms -> 5 sec
};

}  // namespace mpl::control::pid_controller
