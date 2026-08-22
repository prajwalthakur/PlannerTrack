/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "motion_model_ground_vehicles/singleTrackDynStateModel/SingleTrackDynStateModel.hpp"

#include "project_utils/geometry_utils.hpp"

#include <pluginlib/class_list_macros.hpp>

/** \file
 * \brief \ref SingleTrackDynStateModel implementation.
 * `PLUGINLIB_EXPORT_CLASS` at the bottom of this file is what actually
 * registers the class with pluginlib at runtime -- see
 * \ref plugin_architecture.
 */

//////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::initialze(
    YAML::Node & simConfig, YAML::Node & vehConfig, const UniqueId & id)
{
	mSimConfig = simConfig;
	mVehConfig = vehConfig;
	mId = id;
	NX = mVehConfig["numStates"].as<int>();
	NU = mVehConfig["numControlInputs"].as<int>();
	mInitXPose = mVehConfig["initXPose"].as<double>();
	mInitYPose = mVehConfig["initYPose"].as<double>();
	mInitYaw = mVehConfig["initYaw"].as<double>();
	mInitYawRate = mVehConfig["initYawRate"].as<double>();
	mInitVx = mVehConfig["initVx"].as<double>();
	mInitVy = mVehConfig["initVy"].as<double>();
	mInitSf = mVehConfig["initSf"].as<double>();
	mInitAcc = mVehConfig["initAcc"].as<double>();
	mInitSv = mVehConfig["initSv"].as<double>();
	mMass = mVehConfig["mass"].as<double>();
	mIz = mVehConfig["Iz"].as<double>();
	mLf = mVehConfig["lf"].as<double>();
	mLr = mVehConfig["lr"].as<double>();
	mCf = mVehConfig["cf"].as<double>();
	mCr = mVehConfig["cr"].as<double>();
}

//////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::createIntegrator()
{
	createIntegrator(mSimConfig, mVehConfig);
	reset();
	updateCommandedControl();
}

//////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::createIntegrator(YAML::Node & simConfig, YAML::Node & vehYamlConfig)
{
	double intTimeStep = vehYamlConfig["integration"]["integrationStepSize"].as<double>();
	double simTimeStep = simConfig["simTimeStep"].as<double>();
	// Plain `this`, not shared_from_this(): mIntegrator is a member of this
	// object, so it can't outlive it, and shared_from_this() throws
	// bad_weak_ptr for instances constructed via pluginlib::ClassLoader
	// (the shared_ptr gets built through the type-erased base-class
	// pointer, so enable_shared_from_this's hookup never fires).
	mIntegrator = std::make_shared<IntegratorClass>(
	    [this](const StateVector & state, const InputVector & input) -> StateVector {
		    return xdot(state, input);
	    },
	    [this]() -> const StateVector & { return getState(); },
	    [this](const StateVector & state) -> void { return setState(state); },
	    [this](const InputVector & input) -> void { return setInput(input); }, intTimeStep,
	    simTimeStep);
}

//////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::reset()
{
	mStateStruct.x = mInitXPose;
	mStateStruct.y = mInitYPose;
	mStateStruct.yaw = mInitYaw;
	mStateStruct.vx = mInitVx;
	mStateStruct.vy = mInitVy;
	mStateStruct.yaw_rate = mInitYawRate;
	mStateStruct.steer = mInitSf;
	mInputStruct.acc = mInitAcc;
	mInputStruct.sv = mInitSv;

	mStateVector.resize(NX);
	mInputVector.resize(NU);

	mStateVector = StateToVector(mStateStruct);

	mInputVector(0) = mInitAcc;
	mInputVector(1) = mInitSv;
}

//////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::setState(const StateVector & statevector)
{
	mStateVector = statevector;
	mStateStruct = VectorToState(mStateVector);
	// std::cerr << " updated states " <<  mStateStruct.x << " " << mStateStruct.y << std::endl;
}

//////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::setInput(const InputVector & input_vector)
{
	mInputVector = input_vector;
	mInputStruct.acc = mInputVector(0);
	mInputStruct.sv = mInputVector(1);
}

//////////////////////////////////////////////////////////////////////////

const StateVector & SingleTrackDynStateModel::getState() const
{
	return mStateVector;
}

//////////////////////////////////////////////////////////////////////////

const StateVector & SingleTrackDynStateModel::getInput() const
{
	return mInputVector;
}

//////////////////////////////////////////////////////////////////////////

StateVector SingleTrackDynStateModel::xdot(
    const StateVector & statevector, const InputVector & inputvector) const
{
	StateVector statevector_dot;
	statevector_dot.resize(NX);

	auto xk = this->VectorToState(statevector);
	auto uk = this->VectorToInput(inputvector);

	// Linear tire model: front/rear slip angles from body-frame velocity at
	// each axle, guarded against the vx->0 singularity in atan2's argument

	constexpr double kMinVx = 0.05;
	const double vx_safe =
	    std::abs(xk.vx) < kMinVx ? std::copysign(kMinVx, xk.vx == 0.0 ? 1.0 : xk.vx) : xk.vx;
	const double alpha_f = xk.steer - std::atan2(xk.vy + mLf * xk.yaw_rate, vx_safe);
	const double alpha_r = -std::atan2(xk.vy - mLr * xk.yaw_rate, vx_safe);
	const double fyf = mCf * alpha_f;
	const double fyr = mCr * alpha_r;

	statevector_dot(0) = xk.vx * std::cos(xk.yaw) - xk.vy * std::sin(xk.yaw);
	statevector_dot(1) = xk.vx * std::sin(xk.yaw) + xk.vy * std::cos(xk.yaw);
	statevector_dot(2) = xk.yaw_rate;
	statevector_dot(3) = uk.acc + xk.vy * xk.yaw_rate;
	statevector_dot(4) = (fyf * std::cos(xk.steer) + fyr) / mMass - xk.vx * xk.yaw_rate;
	statevector_dot(5) = (mLf * fyf * std::cos(xk.steer) - mLr * fyr) / mIz;
	statevector_dot(6) = uk.sv;
	return statevector_dot;
}

//////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::updateCommandedControl()
{
	mCommandedControl = mInputVector;
}

//////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::updateCommandedControl(const InputVector & u)
{
	mCommandedControl = u;
}

//////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::step()
{
	mIntegrator->simNextState(mCommandedControl);  // state space step
}

//////////////////////////////////////////////////////////////////////////

StateVector SingleTrackDynStateModel::StateToVector(const StateStruct & state_struct) const
{
	StateVector state_vector;
	state_vector.resize(NX);
	state_vector(0) = state_struct.x;
	state_vector(1) = state_struct.y;
	state_vector(2) = state_struct.yaw;
	state_vector(3) = state_struct.vx;
	state_vector(4) = state_struct.vy;
	state_vector(5) = state_struct.yaw_rate;
	state_vector(6) = state_struct.steer;
	return state_vector;
}

//////////////////////////////////////////////////////////////////////////

InputVector SingleTrackDynStateModel::InputToVector(const InputStruct & input_struct) const
{
	InputVector input_vector;
	input_vector.resize(NU);
	input_vector(0) = input_struct.acc;
	input_vector(1) = input_struct.sv;
	return input_vector;
}

//////////////////////////////////////////////////////////////////////////

InputVector SingleTrackDynStateModel::packAccelSteerRate(double acc, double steeringRate) const
{
	return InputToVector(InputStruct{acc, steeringRate});
}

//////////////////////////////////////////////////////////////////////////

StateStruct SingleTrackDynStateModel::VectorToState(const StateVector & statevector) const
{
	StateStruct st;
	st.x = statevector(0);
	st.y = statevector(1);
	st.yaw = statevector(2);
	st.vx = statevector(3);
	st.vy = statevector(4);
	st.yaw_rate = statevector(5);
	st.steer = statevector(6);
	return st;
}

//////////////////////////////////////////////////////////////////////////

stPose SingleTrackDynStateModel::getStatePose() const
{
	stPose st = stPose(
	    mStateStruct.x, mStateStruct.y, mStateStruct.z, mStateStruct.yaw, mStateStruct.steer);
	return st;
}

//////////////////////////////////////////////////////////////////////////

InputStruct SingleTrackDynStateModel::VectorToInput(const InputVector & inputvector) const
{
	InputStruct input;
	input.acc = inputvector(0);
	input.sv = inputvector(1);
	return input;
}

//////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::setupRos(
    const rclcpp::Node::SharedPtr & node, const std::string & ns, const std::string & fixedFrame)
{
	DynamicModel::setupRos(node, ns, fixedFrame);
	mOdomPub = mNode->create_publisher<nav_msgs::msg::Odometry>("/" + mNamespace + "/odom", 10);
	mSteeringPub = mNode->create_publisher<project_utils_msgs::msg::SteeringReport>(
	    "/" + mNamespace + "/steering_report", 10);
}

//////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::publishStates() const
{
	if (!mOdomPub) {
		return;
	}
	nav_msgs::msg::Odometry odom;
	odom.header.stamp = mNode->now();
	odom.header.frame_id = mFixedFrame;
	odom.child_frame_id = mNamespace + "_base_link";
	odom.pose.pose.position.x = mStateStruct.x;
	odom.pose.pose.position.y = mStateStruct.y;
	odom.pose.pose.position.z = mStateStruct.z;
	odom.pose.pose.orientation = mpl::geometry_utils::createQuaternionFromYaw(mStateStruct.yaw);
	odom.twist.twist.linear.x = mStateStruct.vx;
	odom.twist.twist.linear.y = mStateStruct.vy;
	odom.twist.twist.angular.z = mStateStruct.yaw_rate;
	mOdomPub->publish(odom);

	if (mSteeringPub) {
		project_utils_msgs::msg::SteeringReport steering;
		steering.stamp = mNode->now();
		steering.steering_tire_angle = static_cast<float>(mStateStruct.steer);
		mSteeringPub->publish(steering);
	}
}

PLUGINLIB_EXPORT_CLASS(SingleTrackDynStateModel, DynamicModel)
