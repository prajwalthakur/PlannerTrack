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
#include "pid_controller/pid_controller.hpp"

////////////////////////////////////////////////////////////////////////////////

namespace mpl::control::pid_controller
{

//////////////////////////////////////////////////////////////////////////

PidControllerNode::PidControllerNode(const rclcpp::NodeOptions & nodeOptions)
    : Node("pid_controller", nodeOptions),
      mLonPid(
          0.0, 0.0, 0.0,
          [this]() { return mCurrentOdometryPtr ? mCurrentOdometryPtr->twist.twist.linear.x : 0.0; },
          [this](double acc) { mComputedAcc = acc; }),
      mLatPid(
          0.0, 0.0, 0.0,
          [this]() {
	          return mCurrentSteeringPtr ? static_cast<double>(mCurrentSteeringPtr->steering_tire_angle)
	                                      : 0.0;
          },
          [this](double sv) { mComputedSv = sv; })
{
	// Logger
	mLogger = mpl::rclcpp_utils::Logger(this->get_logger());
	mLogger.info("PidControllerNode constructor");

	// Pointer to parameter pointer
	mParameters = std::make_unique<mpl::rclcpp_utils::Parameters>(*this);
	auto getParam = mParameters->getParamGetter("");

	float ctrlPeriodHz = 150.0f;
	getParam(ctrlPeriodHz, "controller_frequency", 150.0f);
	mCtrlPeriodSec = 1.0 / ctrlPeriodHz;
	getParam(mTimeoutThrSec, "timeout_thr_sec", 0.5f);

	// per-agent identity, set by ground_vehicle_racing.launch.py
	getParam(mSimConfigFile, "sim_config_file", std::string(""));
	getParam(mAgentsConfigFile, "agents_config_file", std::string(""));
	getParam(mAgentNumber, "agent_number", 0);

	double lonKp = 1.0, lonKi = 0.0, lonKd = 0.0, lonMin = -9.51, lonMax = 9.51;
	auto getLon = mParameters->getParamGetter("accel_pararms");
	getLon(lonKp, "kp", 1.0);
	getLon(lonKi, "ki", 0.0);
	getLon(lonKd, "kd", 0.0);
	getLon(lonMin, "min", -9.51);
	getLon(lonMax, "max", 9.51);
	mLonPid.setPID(lonKp, lonKi, lonKd);
	mLonPid.setOutputBounds(lonMin, lonMax);

	double latKp = 1.0, latKi = 0.0, latKd = 0.0, latMin = -3.2, latMax = 3.2;
	auto getLat = mParameters->getParamGetter("steering_params");
	getLat(latKp, "kp", 1.0);
	getLat(latKi, "ki", 0.0);
	getLat(latKd, "kd", 0.0);
	getLat(latMin, "min", -3.2);
	getLat(latMax, "max", 3.2);
	mLatPid.setPID(latKp, latKi, latKd);
	mLatPid.setOutputBounds(latMin, latMax);

	createAgentModel();

	mControlCmdPub = create_publisher<project_utils_msgs::msg::EigenVectorStamped>(
	    "~/output/control_cmd", rclcpp::QoS{1});

	// Timer
	{
		const auto periodNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
		    std::chrono::duration<double, std::ratio<1, 1>>(mCtrlPeriodSec));

		mTimerControl = rclcpp::create_timer(
		    this, get_clock(), periodNs, [this]() { this->callbackTimerControl(); });
	}
}

//////////////////////////////////////////////////////////////////////////

void PidControllerNode::createAgentModel()
{
	if (mSimConfigFile.empty() || mAgentsConfigFile.empty()) {
		mLogger.info(
		    "sim_config_file/agents_config_file not set, skipping agent model creation.");
		return;
	}

	const YAML::Node simConfig = YAML::LoadFile(mSimConfigFile);
	const YAML::Node agentsConfig = YAML::LoadFile(mAgentsConfigFile);
	const YAML::Node agentConfig = agentsConfig["agents"]["agent_" + std::to_string(mAgentNumber)];

	// This node only needs the dynamics model (for InputToVector's input
	// layout) -- node/fixedFrame/createStPublisher stay disabled since this
	// isn't the model that should be publishing odom/steering_report (that's
	// agent_sim's/the real vehicle's own model instance).
	mVehicleFactory = std::make_unique<VehicleModelFactory>(mLogger);
	const UniqueId id("controller_agent", mAgentNumber);
	mAgent = mVehicleFactory->create(simConfig, agentConfig, id, nullptr, "", false);
	if (!mAgent) {
		throw std::runtime_error("PidControllerNode: failed to create agent model");
	}
}

//////////////////////////////////////////////////////////////////////////

bool PidControllerNode::processData()
{
	const auto & getData = [this](auto & dest, auto & sub, const std::string & dataType) {
		const auto temp = sub.takeData();
		if (temp) {
			dest = temp;
			return true;
		}
		mLogger.info_throttle(
		    *get_clock(), loggerThrottleInterval, "waiting for %s data", dataType.c_str());
		return false;
	};

	const bool odomReady = getData(mCurrentOdometryPtr, mSubOdometry, "odometry");
	const bool steerReady = getData(mCurrentSteeringPtr, mSubSteering, "steering");
	const bool controlReady = getData(mCurrentControlPtr, mSubControl, "reference-control");

	mLogger.info_throttle(*get_clock(), loggerThrottleInterval,
	    "odom=%d, steer=%d, control=%d", odomReady, steerReady, controlReady);

	return odomReady && steerReady && controlReady;
}

//////////////////////////////////////////////////////////////////////////

bool PidControllerNode::isTimeOut() const
{
	const auto now = this->now();
	if ((now - mCurrentControlPtr->stamp).seconds() > mTimeoutThrSec) {
		RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), loggerThrottleInterval,
		    "Reference control command too old, actuator command will not be published.");
		return true;
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////

void PidControllerNode::callbackTimerControl()
{
	if (!processData()) {
		return;
	}
	if (!mAgent) {
		mLogger.info_throttle(*get_clock(), loggerThrottleInterval,
		    "Agent model not created yet, actuator command will not be published.");
		return;
	}
	if (isTimeOut()) {
		return;
	}

	mLonPid.setTarget(mCurrentControlPtr->longitudinal.velocity);
	mLatPid.setTarget(mCurrentControlPtr->lateral.steering_tire_angle);
	mLonPid.tick(mCtrlPeriodSec);
	mLatPid.tick(mCtrlPeriodSec);

	const InputVector inputVector =
	    mAgent->dynamicModel().packAccelSteerRate(mComputedAcc, mComputedSv);

	project_utils_msgs::msg::EigenVectorStamped msg;
	msg.stamp = this->now();
	msg.data.assign(inputVector.data(), inputVector.data() + inputVector.size());
	mControlCmdPub->publish(msg);
}

////////////////////////////////////////////////////////////////////////

}  // namespace mpl::control::pid_controller

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(mpl::control::pid_controller::PidControllerNode)
