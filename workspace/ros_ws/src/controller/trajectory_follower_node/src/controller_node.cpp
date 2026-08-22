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
#include "trajectory_follower_node/controller_node.hpp"

/** \file
 * \brief \ref mpl::control::trajectory_follower_node::Controller
 * "Controller" implementation: input polling, controller dispatch, horizon
 * merging/resampling, and debug/processing-time publishing.
 */

namespace
{

//////////////////////////////////////////////////////////////////////////

/// \brief Upsample \p origHorizon from \p origTimeStepMs to \p newTimeStepMs by holding each command constant across the finer steps.
template <typename T>
std::vector<T> resampleHorizonByZeroOrderHold(
    const std::vector<T> & origHorizon, const double origTimeStepMs, const double newTimeStepMs)
{
	std::vector<T> resampledHorizon{};
	const size_t stepFactor = static_cast<size_t>(origTimeStepMs / newTimeStepMs);
	const size_t resampledSize = origHorizon.size() * stepFactor;
	resampledHorizon.reserve(resampledSize);
	for (const auto & cmd : origHorizon) {
		for (size_t i = 0; i < stepFactor; ++i) {
			resampledHorizon.push_back(cmd);
		}
	}
	return resampledHorizon;
}
}  // namespace

////////////////////////////////////////////////////////////////////////////////

namespace mpl::control::trajectory_follower_node
{
using std::placeholders::_1;

//////////////////////////////////////////////////////////////////////////

Controller::Controller(const rclcpp::NodeOptions & nodeOptions) : Node("controller", nodeOptions)
{
	// Logger
	mLogger = mpl::rclcpp_utils::Logger(this->get_logger());
	mLogger.info("Controller constructor");

	// Pointer to parameter pointer
	mParameters = std::make_unique<mpl::rclcpp_utils::Parameters>(*this);
	auto getParam = mParameters->getParamGetter("");

	// get Parameters
	float ctrlPeriodHz = 0.0f;
	getParam(ctrlPeriodHz, "controller_frequency", 20.0f);  // controller
	double ctrlPeriodSec = 1.0 / ctrlPeriodHz;
	// mLogger.info("ctrlPeriodSec %f", ctrlPeriodSec);
	getParam(mTimeoutThrSec, "timeout_thr_sec", 0.48f);

	// per-agent identity, set by ground_vehicle_racing.launch.py
	getParam(mSimConfigFile, "sim_config_file", std::string(""));
	getParam(mAgentsConfigFile, "agents_config_file", std::string(""));
	getParam(mAgentNumber, "agent_number", 0);

	// not used
	mCyclicMessageTimeoutThrSec = mpl::rclcpp_utils::get_or_declare_parameter<double>(
	    *this, "cyclic_message_timeout_thr_sec", 0.0);
	// NOTE: It is possible that using control_horizon could be expected to enhance performance,
	// but it is not a formal interface topic, only an experimental one.
	// So it is disabled by default.
	mEnableControlCmdHorizonPub = mpl::rclcpp_utils::get_or_declare_parameter<bool>(
	    *this, "enable_control_cmd_horizon_pub", false);

	// check if hybrid mode is active or not
	getParam(mHybridControllerMode, "hybrid_mode_active", false);
	if (!mHybridControllerMode) {
		// get lateral and longitudninal controller

		// lateral controller
		std::string latContMode = "";
		getParam(latContMode, "lateral_controller_mode", std::string("none"));
		mLogger.info("lateral_controller_mode %s", latContMode.c_str());
		mLateralController = getLateralController(latContMode, *this);
		if (!mLateralController) {
			throw std::domain_error("[LateralController] invalid algorithm");
		}

		// longitudinal controller
		std::string longContMode = "";
		getParam(longContMode, "longitudinal_controller_mode", std::string("none"));
		mLongitudinalController = getLongitudinalController(longContMode, *this);

		if (!mLongitudinalController) {
			throw std::domain_error("[LongitudinalController] invalid algorithm");
		}

	} else {
		std::string hybridControl = "";
		getParam(hybridControl, "hybrid_mode", "mpc");
		mHybridController = getHybridController(hybridControl, *this);
		// mLogger.info("hybrid controller created.");
		if (!mHybridController) {
			throw std::domain_error("[mHybridController] invalid algorithm");
		}
	}

	// Build this agent's vehicle model -- ControllerBase::createAgent()
	// already does the VehicleModelFactory/pluginlib work, so just call it
	// on whichever algorithm object(s) got constructed above rather than
	// duplicating that construction here.
	createAgentModel();

	// select the lateral controller mode

	mControlCmdPub = create_publisher<project_utils_msgs::msg::Control>(
	    "~/output/control_cmd", rclcpp::QoS{1}.transient_local());

	mPubProcessingTimeLatMs =
	    create_publisher<Float64Stamped>("~/lateral/debug/processing_time_ms", 1);

	mPubProcessingTimeLonMs =
	    create_publisher<Float64Stamped>("~/longitudinal/debug/processing_time_ms", 1);

	mDebugMarkerPub = create_publisher<visualization_msgs::msg::MarkerArray>(
	    "~/output/debug_marker", rclcpp::QoS{1});

	if (mEnableControlCmdHorizonPub) {
		mControlCmdHorizonPub = create_publisher<project_utils_msgs::msg::ControlHorizon>(
		    "~/debug/control_cmd_horizon", 1);
	}

	// Timer
	{
		// std::ratio<1, 1000>
		const auto preiodNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
		    std::chrono::duration<double, std::ratio<1, 1>>(ctrlPeriodSec));

		mTimerControl = rclcpp::create_timer(
		    this, get_clock(), preiodNs, [this]() { this->callbackTimerControl(); });
	}
}

//////////////////////////////////////////////////////////////////////////

void Controller::createAgentModel()
{
	if (mSimConfigFile.empty() || mAgentsConfigFile.empty()) {
		mLogger.info("sim_config_file/agents_config_file not set, skipping agent model creation.");
		return;
	}

	const YAML::Node simConfig = YAML::LoadFile(mSimConfigFile);
	const YAML::Node agentsConfig = YAML::LoadFile(mAgentsConfigFile);
	const YAML::Node agentConfig = agentsConfig["agents"]["agent_" + std::to_string(mAgentNumber)];

	// ControllerBase::createAgent() owns the actual VehicleModelFactory/
	// pluginlib construction -- just invoke it on whichever algorithm
	// object(s) this instance is actually running.
	if (mHybridControllerMode) {
		if (!mHybridController->createAgent(simConfig, agentConfig, mAgentNumber)) {
			throw std::runtime_error("Controller: failed to create agent model (hybrid)");
		}
	} else {
		if (!mLateralController->createAgent(simConfig, agentConfig, mAgentNumber)) {
			throw std::runtime_error("Controller: failed to create agent model (lateral)");
		}
	}
}

//////////////////////////////////////////////////////////////////////////

bool Controller::processData(rclcpp::Clock & clock)
{
	bool isReady = true;
	// Logger if the data is not available, print every loggerThrottleInterval ms
	const auto & logData = [&clock, this](const std::string & dataType) {
		RCLCPP_INFO_THROTTLE(
		    get_logger(), clock, loggerThrottleInterval, "waiting for %s data", dataType.c_str());
	};

	// try to get data from subscriber buffer
	const auto & getData = [&logData](auto & dest, auto & sub, const std::string dataType = "") {
		const auto temp = sub.takeData();
		if (temp) {
			dest = temp;
			return true;
		}
		if (!dataType.empty()) {
			logData(dataType);
		}
		return false;
	};
	bool odomReady = getData(mCurrentOdometryPtr, mSubOdometry, "odometry");
	// bool stateReady = getData(mCurrentStatePtr, mSubStateMachine, "state-machine");
	// bool wpReady = getData(mCurrentLocalWpPtr, mSubLocalWp, "local-wp");
	bool trajReady = getData(mCurrentTrajectoryPtr, mSubRefPath, "reference-trajectory");
	bool steerReady = getData(mCurrentSteeringPtr,mSubSteering,"steering");
	mLogger.info_throttle(clock, loggerThrottleInterval,
	    "odom=%d, traj-ready=%d, steer-ready %d", odomReady, trajReady, steerReady);

	isReady = odomReady  && trajReady && steerReady;
	// isReady&= getData(mCurrentAccelPtr,mSubAccel,"acceleration"); #TODO:
	// isReady&= getData(mCurrentSteeringPtr,mSubSteering,"steering"); #TODO:
	// isReady&= getData(mCurrentTrajectoryPtr,mSubRefPath,"trajectory"); #TODO:
	// isReady &= getData(mCurrentOdometryPtr, mSubOdometry, "odometry");
	// isReady &= getData(mCurrentStatePtr, mSubStateMachine, "state-machine");
	// isReady &= getData(mCurrentLocalWpPtr, mSubLocalWp, "local-wp");
	return isReady;
}

//////////////////////////////////////////////////////////////////////////

bool Controller::isTimeOut(
    trajectory_follower::LongitudinalOutput & longOut, trajectory_follower::LateralOutput & latOut)
{
	const auto now = this->now();
	if ((now - latOut.mControlCmd.stamp).seconds() > mTimeoutThrSec) {
		RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), loggerThrottleInterval,
		    "Lateral control command too old, control-cmd will not be published.");
		return true;
	}
	if ((now - longOut.mControlCmd.stamp).seconds() > mTimeoutThrSec) {
		RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), loggerThrottleInterval,
		    "Longitudinal control command too old, control-cmd will not be published.");
		return true;
	}

	return false;
}

//////////////////////////////////////////////////////////////////////////

bool Controller::isTimeOut(trajectory_follower::HybridOutput & hybridOut)
{
	const auto now = this->now();
	if ((now - hybridOut.mControlCmd.stamp).seconds() > mTimeoutThrSec) {
		RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), loggerThrottleInterval,
		    "Hybrid control command too old, control-cmd will not be published.");
		return true;
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////

std::unique_ptr<trajectory_follower::InputData> Controller::createInputData(rclcpp::Clock & clock)
{
	if (!processData(clock)) {
		return nullptr;
	}
	auto inputData = std::make_unique<trajectory_follower::InputData>();
	inputData->mCurrentOdometry = *(mCurrentOdometryPtr);
	// inputData->mLocalWpArray = *(mCurrentLocalWpPtr);
	// inputData->mStateMachine = *(mCurrentStatePtr);
	inputData->mCurrentTrajectory = *(mCurrentTrajectoryPtr);
	inputData->mCurrentSteering = *(mCurrentSteeringPtr);
	// inputData.mCurrentAcc = *mCurrentAccelPtr;
	return inputData;
}

//////////////////////////////////////////////////////////////////////////

void Controller::callbackTimerControl()
{
	mLogger.info_throttle(*get_clock(), loggerThrottleInterval, "In controll callback");
	project_utils_msgs::msg::Control controlCmdOut;
	controlCmdOut.stamp = this->now();
	// 1.Create Input data
	const auto inputData = createInputData(*get_clock());
	// mLogger.info("checking if input ready.");
	if (!inputData) {
		mLogger.info_throttle(*get_clock(), loggerThrottleInterval,
		    "Control is skipped since input data is not ready.");
		return;
	}
	// mLogger.info("checking if controller ready");
	// 2. check if controllers are ready
	if (!mHybridControllerMode) {
		const bool isLatReady = mLateralController->isReady(*inputData);

		const bool isLongReady = mLongitudinalController->isReady(*inputData);
		if (!isLatReady || !isLongReady) {
			RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), loggerThrottleInterval,
			    "Control is skipped since lateral and/or longitudinal controllers are not ready to "
			    "run.");
			return;
		}
	} else {
		const bool isHybridReady = mHybridController->isReady(*inputData);
		if (!isHybridReady) {
			RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), loggerThrottleInterval,
			    "Control is skipped since hybrid controllers are not ready to run.");
			return;
		}
	}

	// mLogger.info("controller ready");

	// 3 . run controllers
	trajectory_follower::LateralOutput latOut;
	trajectory_follower::LongitudinalOutput longOut;
	trajectory_follower::HybridOutput hybridOut;
	if (!mHybridControllerMode) {
		mLogger.info("going to run laterla run");
		if (!mLateralController) {
			mLogger.info("lateral control nullptr");
		}
		if (!inputData) {
			mLogger.info("input data nullptr");
		}
		latOut = mLateralController->run(*inputData);
		// mLogger.info("laterla run done ");
		// mLogger.info("going long run");
		longOut = mLongitudinalController->run(*inputData);
		// mLogger.info("long run");
	} else {
		// mLogger.info("calling hybrid controller ");
		hybridOut = mHybridController->run(*inputData);
		mLogger.info_throttle(*get_clock(), loggerThrottleInterval, "called hybrid controller ");
	}

	// 4 . sync with other controller
	// TODO: uderstand
	if (!mHybridControllerMode) {
		mLongitudinalController->sync(latOut.mSyncData);
		mLateralController->sync(longOut.mSyncData);
	}

	if (!mHybridControllerMode) {
		if (isTimeOut(longOut, latOut)) {
			mLogger.error("timeout");
			return;
		}

	} else {
		if (isTimeOut(hybridOut)) {
			return;
		}
	}

	// 4.5 Update diagonistics
	// TODO:

	// 5 publish control command
	if (!mHybridControllerMode) {
		controlCmdOut.lateral = latOut.mControlCmd;
		controlCmdOut.longitudinal = longOut.mControlCmd;
		mControlCmdPub->publish(controlCmdOut);
		mLogger.info("published the controller cmd");
	} else {
		controlCmdOut = hybridOut.mControlCmd;
		mControlCmdPub->publish(controlCmdOut);
	}

	// 6. publish debug
	if (!mHybridControllerMode) {
		publishDebugMarker(*inputData, latOut, longOut);
	}
	// 7. publish experimental topic
	if (mEnableControlCmdHorizonPub) {
		const auto controlHorizon =
		    mergeLatLonHorizon(latOut.mControlCmdHorizon, longOut.mControlCmdHorizon, this->now());
		if (controlHorizon.has_value()) {
			mControlCmdHorizonPub->publish(controlHorizon.value());
		}
	}
}

//////////////////////////////////////////////////////////////////////////

void Controller::publishDebugMarker(
    [[maybe_unused]] const trajectory_follower::InputData & inputData,
    [[maybe_unused]] const trajectory_follower::LateralOutput & latOut,
    [[maybe_unused]] const trajectory_follower::LongitudinalOutput & longOut) const
{
	// visualization_msgs::msg::MarkerArray debug_marker_array{};
	// {
	//     auto marker = autoware_utils::create_default_marker(
	//     "map", this->now(), "steer_converged", 0,
	//     visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
	//     autoware_utils::create_marker_scale(0.0, 0.0, 1.0),
	//     autoware_utils::create_marker_color(1.0, 1.0, 1.0, 0.99)); marker.pose =
	//     inputData.mCurrentOdometry.pose.pose;

	//     std::stringstream ss;
	//     const double current = inputData.current_steering.steering_tire_angle;
	//     const double cmd = latOut.mControlCmd.steering_tire_angle;
	//     const double diff = current - cmd;
	//     ss << "current:" << current << " cmd:" << cmd << " diff:" << diff
	//     << (latOut.sync_data.is_steer_converged ? " (converged)" : " (not converged)");
	//     marker.text = ss.str();

	//     debug_marker_array.markers.push_back(marker);
	// }

	// debug_marker_pub_->publish(debug_marker_array);
}

//////////////////////////////////////////////////////////////////////////

void Controller::publishProcessingTime(
    const double t_ms, [[maybe_unused]] const rclcpp::Publisher<Float64Stamped>::SharedPtr pub)
{
	Float64Stamped msg{};
	msg.stamp = this->now();
	msg.data = t_ms;
	// pub->publish(msg);
}

//////////////////////////////////////////////////////////////////////////

std::optional<ControlHorizon> Controller::mergeLatLonHorizon(const LateralHorizon & lateralHorizon,
    const LongitudinalHorizon & longitudinalHorizon, const rclcpp::Time & stamp)
{
	if (lateralHorizon.mControls.empty() || longitudinalHorizon.mControls.empty()) {
		return std::nullopt;
	}

	project_utils_msgs::msg::ControlHorizon controlHorizon{};
	controlHorizon.stamp = stamp;

	// If either of the horizons has only one control, repeat the control to match the other
	// horizon.
	if (lateralHorizon.mControls.size() == 1) {
		controlHorizon.time_step_ms = longitudinalHorizon.mTimeStepMs;
		const auto lateral = lateralHorizon.mControls.front();
		for (const auto & longitudinal : longitudinalHorizon.mControls) {
			project_utils_msgs::msg::Control control;
			control.longitudinal = longitudinal;
			control.lateral = lateral;
			control.stamp = stamp;
			controlHorizon.controls.push_back(control);
		}
		return controlHorizon;
	}
	if (longitudinalHorizon.mControls.size() == 1) {
		controlHorizon.time_step_ms = lateralHorizon.mTimeStepMs;
		const auto longitudinal = longitudinalHorizon.mControls.front();
		for (const auto & lateral : lateralHorizon.mControls) {
			project_utils_msgs::msg::Control control;
			control.longitudinal = longitudinal;
			control.lateral = lateral;
			control.stamp = stamp;
			controlHorizon.controls.push_back(control);
		}
		return controlHorizon;
	}

	// If both horizons have multiple controls, align the time steps and zero-order hold the
	// controls.

	// calculate greatest common divisor of time steps
	const auto gcd_double = [](const double a, const double b) {
		const double precision = 1e9;
		const int int_a = static_cast<int>(round(a * precision));
		const int int_b = static_cast<int>(round(b * precision));
		return static_cast<double>(std::gcd(int_a, int_b)) / precision;
	};

	const double timeStepMs =
	    gcd_double(lateralHorizon.mTimeStepMs, longitudinalHorizon.mTimeStepMs);

	controlHorizon.time_step_ms = timeStepMs;

	const auto lateralControls = resampleHorizonByZeroOrderHold(
	    lateralHorizon.mControls, lateralHorizon.mTimeStepMs, timeStepMs);
	const auto longitudinalControls = resampleHorizonByZeroOrderHold(
	    longitudinalHorizon.mControls, longitudinalHorizon.mTimeStepMs, timeStepMs);

	if (lateralControls.size() != longitudinalControls.size()) {
		return std::nullopt;
	}

	const size_t num_steps = lateralControls.size();
	for (size_t i = 0; i < num_steps; ++i) {
		project_utils_msgs::msg::Control control{};
		control.stamp = stamp;
		control.lateral = lateralControls.at(i);
		control.longitudinal = longitudinalControls.at(i);
		controlHorizon.controls.push_back(control);
	}

	return controlHorizon;
}

}  // namespace mpl::control::trajectory_follower_node

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(mpl::control::trajectory_follower_node::Controller)
