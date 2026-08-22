// Copyright 2026 Prajwal Thakur
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
#include "dummy_lateral_controller/dummy_lateral_controller.hpp"
#include "dummy_longitudinal_controller/dummy_longitudinal_controller.hpp"
#include "project_utils/logger.hpp"
#include "project_utils/parameter.hpp"
#include "project_utils/polling_subscriber.hpp"
#include "trajectory_follower_base/trajectory_follower_base_collection.h"
#include "trajectory_follower_node/controller_mode.hpp"

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// msgs
// ros
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
// custom msgs
#include "project_utils_msgs/msg/control.hpp"
#include "project_utils_msgs/msg/control_horizon.hpp"
#include "project_utils_msgs/msg/longitudinal.hpp"
#include "project_utils_msgs/msg/trajectory.hpp"
#include "project_utils_msgs/msg/wpnt_array.hpp"

using project_utils_msgs::msg::Float64Stamped;

namespace mpl::control
{
using trajectory_follower::LateralHorizon;
using trajectory_follower::LateralOutput;
using trajectory_follower::LongitudinalHorizon;
using trajectory_follower::LongitudinalOutput;
namespace trajectory_follower_node
{
using project_utils_msgs::msg::ControlHorizon;
namespace trajectory_follower = ::mpl::control::trajectory_follower;
namespace mpl_utils = ::mpl_rclcpp_utils;
/**
 * \brief The node that runs whichever lateral/longitudinal/hybrid
 * controller its config names (see \ref controller_mode.hpp), on a fixed
 * timer, and publishes the resulting \c Control command.
 *
 * Either \ref mLateralController + \ref mLongitudinalController (separate
 * lateral/longitudinal controllers) or \ref mHybridController (a single
 * combined controller, see \ref mHybridControllerMode) is used each cycle,
 * never both.
 */
class Controller : public rclcpp::Node
{
  public:
	explicit Controller(const rclcpp::NodeOptions & nodeOptions);
	virtual ~Controller() {}

  private:
	/// \brief Build this cycle's \ref trajectory_follower::InputData "InputData" from the latest polled subscriber messages.
	std::unique_ptr<trajectory_follower::InputData> createInputData(rclcpp::Clock & clock);
	/// \brief Control-timer callback: poll inputs, run the configured controller(s), publish the command.
	void callbackTimerControl();
	/**
	 * \brief Check if the node has all the data to run the controller.
	 * \return False if any required input hasn't arrived yet -- in that
	 * case the caller should skip this cycle's control computation and
	 * log instead.
	 */
	bool processData(rclcpp::Clock & clock);
	/// \brief Whether the separate lateral/longitudinal outputs are stale relative to \ref mCyclicMessageTimeoutThrSec.
	bool isTimeOut(trajectory_follower::LongitudinalOutput & lonOut,
	    trajectory_follower::LateralOutput & latOut);
	/// \brief Whether the combined hybrid-controller output is stale relative to \ref mCyclicMessageTimeoutThrSec.
	bool isTimeOut(trajectory_follower::HybridOutput & hybridOut);
	// LateralControllerMode getLateralControllerMode(const std::string& algoName) const;
	// LongitudinalControllerMode getLongitudinalControllerMode(const std::string& algoName) const;
	/// \brief Publish RViz debug markers for the current input/lateral/longitudinal state.
	void publishDebugMarker([[maybe_unused]] const trajectory_follower::InputData & inputData,
	    [[maybe_unused]] const trajectory_follower::LateralOutput & latOut,
	    [[maybe_unused]] const trajectory_follower::LongitudinalOutput & longOut) const;
	/**
	 * @brief merge lateral and longitudinal horizons
	 * @details If one of the commands has only one control, repeat the control to match the other
	 *          horizon. If each horizon has different time intervals, resample them to match the
	 * size with the greatest common divisor.
	 * @param lateral_horizon lateral horizon
	 * @param longitudinal_horizon longitudinal horizon
	 * @param stamp stamp
	 * @return merged control horizon
	 */
	static std::optional<ControlHorizon> mergeLatLonHorizon(const LateralHorizon & lateralHorizon,
	    const LongitudinalHorizon & longitudinalHorizon, const rclcpp::Time & timeStamp);

	/// \brief Publish \p tMs (this cycle's controller compute time, in ms) on \p pub.
	void publishProcessingTime(
	    const double tMs, [[maybe_unused]] const rclcpp::Publisher<Float64Stamped>::SharedPtr pub);

	/// \brief Build this node's \ref AgentModel (via \c ControllerBase::createAgent) for the agent named by \ref mAgentNumber.
	void createAgentModel();

  private:
	// logger
	mpl::rclcpp_utils::Logger mLogger;
	// parameters
	std::unique_ptr<mpl::rclcpp_utils::Parameters> mParameters;

	rclcpp::TimerBase::SharedPtr mTimerControl;
	double mTimeoutThrSec;
	double mCyclicMessageTimeoutThrSec;
	bool mEnableControlCmdHorizonPub{false};
	bool mHybridControllerMode{false};

	// Which agent this controller instance drives -- read from the per-agent
	// launch params (scenarios/.../launch/ground_vehicle_racing.launch.py),
	// used to pick this agent's YAML block out of agents_config_file so the
	// controller can build the same vehicle model agent_sim uses.
	std::string mSimConfigFile;
	std::string mAgentsConfigFile;
	int mAgentNumber{0};

	std::optional<LongitudinalOutput> mLongitudinalOutput{std::nullopt};
	std::unique_ptr<trajectory_follower::LongitudinalControllerBase> mLongitudinalController{
	    nullptr};
	std::unique_ptr<trajectory_follower::LateralControllerBase> mLateralController{nullptr};
	std::unique_ptr<trajectory_follower::HybridControllerBase> mHybridController{nullptr};

	// Subscriber
	mpl_utils::InternalProcessPollingSubscriber<project_utils_msgs::msg::Trajectory> mSubRefPath{
	    this, "~/input/reference_trajectory", rclcpp::QoS{1}.transient_local()};
	mpl_utils::InternalProcessPollingSubscriber<nav_msgs::msg::Odometry> mSubOdometry{
	    this, "~/input/current_odometry"};
	mpl_utils::InternalProcessPollingSubscriber<project_utils_msgs::msg::SteeringReport>
	    mSubSteering{this, "~/input/current_steering"};
	mpl_utils::InternalProcessPollingSubscriber<geometry_msgs::msg::AccelWithCovarianceStamped>
	    mSubAccel{this, "~/input/current_accel"};
	mpl_utils::InternalProcessPollingSubscriber<std_msgs::msg::String> mSubStateMachine{
	    this, "~/input/state_machine"};
	mpl_utils::InternalProcessPollingSubscriber<project_utils_msgs::msg::WpntArray> mSubLocalWp{
	    this, "~/input/local_waypoints"};  // waypoints (x, y, v, norm trackbound, s, kappa)

	// Publishers
	rclcpp::Publisher<project_utils_msgs::msg::Control>::SharedPtr mControlCmdPub;
	rclcpp::Publisher<Float64Stamped>::SharedPtr mPubProcessingTimeLatMs;
	rclcpp::Publisher<Float64Stamped>::SharedPtr mPubProcessingTimeLonMs;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mDebugMarkerPub;
	rclcpp::Publisher<project_utils_msgs::msg::ControlHorizon>::SharedPtr mControlCmdHorizonPub;

	project_utils_msgs::msg::Trajectory::ConstSharedPtr mCurrentTrajectoryPtr;
	nav_msgs::msg::Odometry::ConstSharedPtr mCurrentOdometryPtr;
	project_utils_msgs::msg::SteeringReport::ConstSharedPtr mCurrentSteeringPtr;
	geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr mCurrentAccelPtr;
	std_msgs::msg::String::ConstSharedPtr mCurrentStatePtr;
	project_utils_msgs::msg::WpntArray::ConstSharedPtr mCurrentLocalWpPtr;

	static constexpr double loggerThrottleInterval = 5000;  // in ms -> 5 sec
};

}  // namespace trajectory_follower_node

}  // namespace mpl::control
