/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "motion_model_ground_vehicles/singleTrackDynStateModel/SingleTrackDynStateModel.hpp"

#include "project_utils/geometry_utils.hpp"

#include <pluginlib/class_list_macros.hpp>

//////////////////////////////////////////////////////////////////////////

// SingleTrackDynStateModel::SingleTrackDynStateModel(
//     YAML::Node & simConfig, YAML::Node & vehConfig, const UniqueId & id)
//     : BaseType(simConfig, vehConfig, id)
// {
// 	NX = mVehConfig["numStates"].as<int>();
// 	NU = mVehConfig["numControlInputs"].as<int>();
// 	mInitXPose = mVehConfig["initXPose"].as<double>();
// 	mInitYPose = mVehConfig["initYPose"].as<double>();
// 	mInitYaw = mVehConfig["initYaw"].as<double>();

// 	mInitVx = mVehConfig["initVx"].as<double>();
// 	mInitSf = mVehConfig["initSf"].as<double>();
// 	mInitAcc = mVehConfig["initAcc"].as<double>();
// 	mInitSv = mVehConfig["initSv"].as<double>();
// 	mVehWheelBase = mVehConfig["vehWheelBase"].as<double>();
// }

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
	mInitVx = mVehConfig["initVx"].as<double>();
	mInitSf = mVehConfig["initSf"].as<double>();
	mInitAcc = mVehConfig["initAcc"].as<double>();
	mInitSv = mVehConfig["initSv"].as<double>();
	mVehWheelBase = mVehConfig["vehWheelBase"].as<double>();
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
	mStateStruct.sf = mInitSf;
	mInputStruct.acc = mInitAcc;
	mInputStruct.sv = mInitSv;

	mStateVector.resize(NX);
	mInputVector.resize(NU);

	mStateVector(0) = mStateStruct.x;
	mStateVector(1) = mStateStruct.y;
	mStateVector(2) = mStateStruct.yaw;
	mStateVector(3) = mStateStruct.vx;
	mStateVector(4) = mStateStruct.sf;

	mInputVector(0) = mInitAcc;
	mInputVector(1) = mInitSv;
}

//////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::setState(const StateVector & statevector)
{
	mStateVector = statevector;
	mStateStruct.x = mStateVector(0);
	mStateStruct.y = mStateVector(1);
	mStateStruct.yaw = mStateVector(2);
	mStateStruct.vx = mStateVector(3);
	mStateStruct.sf = mStateVector(4);
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
	// xdot,ydot,yawdot,vfodt,sfdot
	statevector_dot(0) = xk.vx * std::cos(xk.yaw);
	statevector_dot(1) = xk.vx * std::sin(xk.yaw);
	statevector_dot(2) = xk.vx * std::tan(xk.sf) / mVehWheelBase;
	statevector_dot(3) = uk.acc;
	statevector_dot(4) = uk.sv;
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
	state_vector(0) = state_struct.x;
	state_vector(1) = state_struct.y;
	state_vector(2) = state_struct.yaw;
	state_vector(3) = state_struct.vx;
	state_vector(4) = state_struct.sf;
	return state_vector;
}

//////////////////////////////////////////////////////////////////////////

InputVector SingleTrackDynStateModel::InputToVector(const InputStruct & input_struct) const
{
	InputVector input_vector;
	input_vector(0) = input_struct.sv;
	input_vector(1) = input_struct.acc;
	return input_vector;
}

//////////////////////////////////////////////////////////////////////////

StateStruct SingleTrackDynStateModel::VectorToState(const StateVector & statevector) const
{
	StateStruct st;
	st.x = statevector(0);
	st.y = statevector(1);
	st.yaw = statevector(2);
	st.vx = statevector(3);
	st.sf = statevector(4);
	return st;
}

//////////////////////////////////////////////////////////////////////////

stPose SingleTrackDynStateModel::getStatePose() const
{
	stPose st =
	    stPose(mStateStruct.x, mStateStruct.y, mStateStruct.z, mStateStruct.yaw, mStateStruct.sf);
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
	mOdomPub->publish(odom);

	if (mSteeringPub) {
		project_utils_msgs::msg::SteeringReport steering;
		steering.stamp = mNode->now();
		steering.steering_tire_angle = static_cast<float>(mStateStruct.sf);
		mSteeringPub->publish(steering);
	}
}

PLUGINLIB_EXPORT_CLASS(SingleTrackDynStateModel, DynamicModel)
