/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "motion_model_ground_vehicles/singleTrackAugDynModel/SingleTrackAugDynModel.hpp"

#include "project_utils/geometry_utils.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

/** \file
 * \brief \ref SingleTrackAugDynModel implementation. The vehicle physics in
 * xdot() is a copy of SingleTrackDynStateModel::xdot(); keep both in sync
 * (plannertrack_py test_aug_sim.py checks they give the same trajectory).
 */

//////////////////////////////////////////////////////////////////////////

void SingleTrackAugDynModel::initialze(
    YAML::Node & simConfig, YAML::Node & vehConfig, const UniqueId & id)
{
	mSimConfig = simConfig;
	mVehConfig = vehConfig;
	mId = id;
	NX = mVehConfig["numStates"].as<int>();
	NU = mVehConfig["numControlInputs"].as<int>();
	if (NX != kNumStates || NU != kNumInputs) {
		throw std::invalid_argument("SingleTrackAugDynModel: numStates must be 8 and "
		                            "numControlInputs 2 (state [x, y, yaw, vx, vy, yaw_rate, "
		                            "steer, acc], input [jerk, sv]), got " +
		    std::to_string(NX) + " and " + std::to_string(NU));
	}
	mInitXPose = mVehConfig["initXPose"].as<double>();
	mInitYPose = mVehConfig["initYPose"].as<double>();
	mInitYaw = mVehConfig["initYaw"].as<double>();
	mInitYawRate = mVehConfig["initYawRate"].as<double>();
	mInitVx = mVehConfig["initVx"].as<double>();
	mInitVy = mVehConfig["initVy"].as<double>();
	mInitSf = mVehConfig["initSf"].as<double>();
	mInitAcc = mVehConfig["initAcc"].as<double>();  // initial acceleration STATE
	mInitJerk = mVehConfig["initJerk"] ? mVehConfig["initJerk"].as<double>() : 0.0;
	mInitSv = mVehConfig["initSv"].as<double>();
	mMass = mVehConfig["mass"].as<double>();
	mIz = mVehConfig["Iz"].as<double>();
	mLf = mVehConfig["lf"].as<double>();
	mLr = mVehConfig["lr"].as<double>();
	mCf = mVehConfig["cf"].as<double>();
	mCr = mVehConfig["cr"].as<double>();
	// mCf/mCr are load-normalized coefficients C_Sf/C_Sr [1/rad]; the physical
	// cornering stiffness is mu * C_S * F_z (see xdot()). mu and hcg are
	// optional -- defaults give unit friction and the static axle-load split.
	mMu = mVehConfig["mu"] ? mVehConfig["mu"].as<double>() : 1.0;
	mHcg = mVehConfig["hcg"] ? mVehConfig["hcg"].as<double>() : 0.0;
	loadActuatorLimits();
	loadCommandMode();
}

//////////////////////////////////////////////////////////////////////////

void SingleTrackAugDynModel::loadCommandMode()
{
	const std::string mode =
	    mVehConfig["command_mode"] ? mVehConfig["command_mode"].as<std::string>() : "jerk";
	if (mode == "jerk") {
		mCommandMode = CommandMode::kJerk;
		return;
	}
	if (mode != "acceleration") {
		throw std::invalid_argument("SingleTrackAugDynModel: command_mode must be 'jerk' or "
		                            "'acceleration', got '" + mode + "'");
	}
	mCommandMode = CommandMode::kAcceleration;
	if (!mVehConfig["accelTimeConstant"]) {
		throw std::invalid_argument(
		    "SingleTrackAugDynModel: command_mode 'acceleration' needs accelTimeConstant");
	}
	mAccelTimeConstant = mVehConfig["accelTimeConstant"].as<double>();
	if (!(mAccelTimeConstant > 0.0)) {
		throw std::invalid_argument("SingleTrackAugDynModel: accelTimeConstant must be > 0");
	}
}

//////////////////////////////////////////////////////////////////////////

void SingleTrackAugDynModel::loadActuatorLimits()
{
	mClipControl = mVehConfig["clip_control_in_sim"] ? mVehConfig["clip_control_in_sim"].as<bool>() : false;
	if (!mClipControl) {
		return;
	}
	const YAML::Node limits = mVehConfig["vehicle_limits"];
	if (!limits) {
		throw std::invalid_argument(
		    "SingleTrackAugDynModel: clip_control_in_sim is true but the agent has no "
		    "vehicle_limits block");
	}
	const auto positive = [&limits](const char * key) {
		if (!limits[key]) {
			throw std::invalid_argument(
			    std::string("SingleTrackAugDynModel: clip_control_in_sim needs vehicle_limits.") + key);
		}
		const double value = limits[key].as<double>();
		if (!(value > 0.0)) {
			throw std::invalid_argument(
			    std::string("SingleTrackAugDynModel: vehicle_limits.") + key + " must be > 0");
		}
		return value;
	};
	mMaxJerk = positive("max_jerk");
	mMaxSteerRate = positive("max_steer_rate");
	mMaxSteer = positive("max_steer");
	mMaxAccel = positive("max_accel");
	mMaxDecel = positive("max_decel");  // magnitude: acc >= -max_decel
}

//////////////////////////////////////////////////////////////////////////

InputVector SingleTrackAugDynModel::clipInput(const InputVector & input) const
{
	if (!mClipControl) {
		return input;
	}
	InputVector clipped = input;
	clipped(kJerk) = mCommandMode == CommandMode::kAcceleration
	    ? std::clamp(input(kJerk), -mMaxDecel, mMaxAccel)  // u[0] is a_cmd
	    : std::clamp(input(kJerk), -mMaxJerk, mMaxJerk);
	clipped(kSteerRate) = std::clamp(input(kSteerRate), -mMaxSteerRate, mMaxSteerRate);
	return clipped;
}

//////////////////////////////////////////////////////////////////////////

void SingleTrackAugDynModel::projectToLimits(StateVector & state) const
{
	if (!mClipControl) {
		return;
	}
	state(kSteer) = std::clamp(state(kSteer), -mMaxSteer, mMaxSteer);
	state(kAcc) = std::clamp(state(kAcc), -mMaxDecel, mMaxAccel);
}

//////////////////////////////////////////////////////////////////////////

void SingleTrackAugDynModel::createIntegrator()
{
	createIntegrator(mSimConfig, mVehConfig);
	reset();
	updateCommandedControl();
}

//////////////////////////////////////////////////////////////////////////

void SingleTrackAugDynModel::createIntegrator(YAML::Node & simConfig, YAML::Node & vehYamlConfig)
{
	double intTimeStep = vehYamlConfig["integration"]["integrationStepSize"].as<double>();
	double simTimeStep = simConfig["simTimeStep"].as<double>();
	mIntStep = intTimeStep;
	mSimStep = simTimeStep;
	// Plain `this`, not shared_from_this() -- see SingleTrackDynStateModel.
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

void SingleTrackAugDynModel::reset()
{
	mStateStruct.x = mInitXPose;
	mStateStruct.y = mInitYPose;
	mStateStruct.yaw = mpl::geometry_utils::normalizeAngles(mInitYaw);  // [-pi, pi], as in setState
	mStateStruct.vx = mInitVx;
	mStateStruct.vy = mInitVy;
	mStateStruct.yaw_rate = mInitYawRate;
	mStateStruct.steer = mInitSf;
	mStateStruct.acc = mInitAcc;
	// Acceleration mode: command the initial acceleration (zero jerk).
	mInputStruct.jerk = mCommandMode == CommandMode::kAcceleration ? mInitAcc : mInitJerk;
	mInputStruct.sv = mInitSv;

	mStateVector = StateToVector(mStateStruct);
	mInputVector = InputToVector(mInputStruct);
}

//////////////////////////////////////////////////////////////////////////

void SingleTrackAugDynModel::setState(const StateVector & statevector)
{
	mStateVector = statevector;
	// Yaw kept in [-pi, pi]: every state update (integrator, clipping path, Python set_state) passes here.
	mStateVector(2) = mpl::geometry_utils::normalizeAngles(statevector(2));
	mStateStruct = VectorToState(mStateVector);
}

//////////////////////////////////////////////////////////////////////////

void SingleTrackAugDynModel::setInput(const InputVector & input_vector)
{
	mInputVector = input_vector;
	mInputStruct = VectorToInput(mInputVector);
}

//////////////////////////////////////////////////////////////////////////

const StateVector & SingleTrackAugDynModel::getState() const
{
	return mStateVector;
}

//////////////////////////////////////////////////////////////////////////

const StateVector & SingleTrackAugDynModel::getInput() const
{
	return mInputVector;
}

//////////////////////////////////////////////////////////////////////////

StateVector SingleTrackAugDynModel::xdot(
    const StateVector & statevector, const InputVector & inputvector) const
{
	StateVector statevector_dot;
	statevector_dot.resize(NX);

	auto xk = this->VectorToState(statevector);
	auto uk = this->VectorToInput(clipInput(inputvector));
	if (mCommandMode == CommandMode::kAcceleration) {
		// First-order drive actuator: uk.jerk holds a_cmd until here.
		uk.jerk = (uk.jerk - xk.acc) / mAccelTimeConstant;
		if (mClipControl) {
			uk.jerk = std::clamp(uk.jerk, -mMaxJerk, mMaxJerk);
		}
	}
	if (mClipControl) {
		// End stops: no rate pushing a state further past its limit.
		if ((xk.steer >= mMaxSteer && uk.sv > 0.0) || (xk.steer <= -mMaxSteer && uk.sv < 0.0)) {
			uk.sv = 0.0;
		}
		if ((xk.acc >= mMaxAccel && uk.jerk > 0.0) || (xk.acc <= -mMaxDecel && uk.jerk < 0.0)) {
			uk.jerk = 0.0;
		}
	}

	// ---- From here to the marked end: same physics as
	// SingleTrackDynStateModel::xdot(), with acc taken from the state.
	const double wheel_base = mLf + mLr;
	const double vx_dot = xk.acc + xk.vy * xk.yaw_rate;

	// Slip angles, guarded against the vx -> 0 singularity in atan2.
	constexpr double kMinVx = 1e-3;
	const double vx_safe =
	    std::abs(xk.vx) < kMinVx ? std::copysign(kMinVx, xk.vx == 0.0 ? 1.0 : xk.vx) : xk.vx;
	const double alpha_f = xk.steer - std::atan2(xk.vy + mLf * xk.yaw_rate, vx_safe);
	const double alpha_r = -std::atan2(xk.vy - mLr * xk.yaw_rate, vx_safe);

	// Load-normalized cornering stiffness with longitudinal load transfer.
	constexpr double kG = 9.81;
	constexpr double kFzMin = 1e-3;  // keep axle load positive under hard braking / wheel lift
	const double fzf = std::max(mMass * (kG * mLr - vx_dot * mHcg) / wheel_base, kFzMin);
	const double fzr = std::max(mMass * (kG * mLf + vx_dot * mHcg) / wheel_base, kFzMin);
	const double cornering_stiffness_f = mMu * mCf * fzf;  // [N/rad]
	const double cornering_stiffness_r = mMu * mCr * fzr;  // [N/rad]

	// Lateral force saturated smoothly at the friction limit mu * F_z.
	const double fyf_max = mMu * fzf;
	const double fyr_max = mMu * fzr;
	const double fyf = fyf_max * std::tanh(cornering_stiffness_f * alpha_f / fyf_max);
	const double fyr = fyr_max * std::tanh(cornering_stiffness_r * alpha_r / fyr_max);

	// Dynamic single-track lateral / yaw accelerations.
	const double vy_dot_dyn = (fyf * std::cos(xk.steer) + fyr) / mMass - xk.vx * xk.yaw_rate;
	const double r_dot_dyn = (mLf * fyf * std::cos(xk.steer) - mLr * fyr) / mIz;

	// Blend with the kinematic single-track model at low speed.
	// 0.5-2.5 m/s (was 1-3): above 2.5 m/s the model is exactly the dynamic one, which the
	// Diff-WMPC prediction model (dynamic only) matches at the tightest hairpin (v_ref 2.95 m/s; steps.md §5).
	constexpr double kBlendLo = 0.5;  // [m/s]
	constexpr double kBlendHi = 2.5;  // [m/s]
	constexpr double kTau = 0.1;      // [s] manifold relaxation time constant
	const double lambda =
	    std::clamp((std::abs(xk.vx) - kBlendLo) / (kBlendHi - kBlendLo), 0.0, 1.0);

	const double tan_d = std::tan(xk.steer);
	const double sec2_d = 1.0 + tan_d * tan_d;
	const double r_kin = xk.vx * tan_d / wheel_base;
	const double r_dot_kin = (vx_dot * tan_d + xk.vx * sec2_d * uk.sv) / wheel_base;
	const double vy_kin = mLr * r_kin;
	const double vy_dot_kin = mLr * r_dot_kin;

	const double vy_dot =
	    lambda * vy_dot_dyn + (1.0 - lambda) * (vy_dot_kin + (vy_kin - xk.vy) / kTau);
	const double r_dot =
	    lambda * r_dot_dyn + (1.0 - lambda) * (r_dot_kin + (r_kin - xk.yaw_rate) / kTau);
	// ---- End of the physics shared with SingleTrackDynStateModel.

	statevector_dot(0) = xk.vx * std::cos(xk.yaw) - xk.vy * std::sin(xk.yaw);
	statevector_dot(1) = xk.vx * std::sin(xk.yaw) + xk.vy * std::cos(xk.yaw);
	statevector_dot(2) = xk.yaw_rate;
	statevector_dot(3) = vx_dot;
	statevector_dot(4) = vy_dot;
	statevector_dot(5) = r_dot;
	statevector_dot(6) = uk.sv;
	statevector_dot(7) = uk.jerk;  // augmented state: acc_dot = jerk
	return statevector_dot;
}

//////////////////////////////////////////////////////////////////////////

void SingleTrackAugDynModel::updateCommandedControl()
{
	mCommandedControl = mInputVector;
}

//////////////////////////////////////////////////////////////////////////

void SingleTrackAugDynModel::updateCommandedControl(const InputVector & u)
{
	mCommandedControl = u;
}

//////////////////////////////////////////////////////////////////////////

void SingleTrackAugDynModel::step()
{
	if (mCommandedControl.size() != kNumControls) {
		std::string names;
		for (const char * name : kControlNames) {
			names += (names.empty() ? "" : ", ") + std::string(name);
		}
		throw std::invalid_argument("SingleTrackAugDynModel::step: expected " +
		    std::to_string(kNumControls) + " controls [" + names + "], got " +
		    std::to_string(mCommandedControl.size()));
	}
	if (!mClipControl) {
		mIntegrator->simNextState(mCommandedControl);
		return;
	}
	// Same RK4 sub-stepping as IntegratorClass::simNextState, plus the
	// projection onto the steer/acc limits after every sub-step.
	const InputVector u = clipInput(mCommandedControl);
	StateVector x = getState();
	const int subSteps = static_cast<int>(mSimStep / mIntStep);
	for (int i = 0; i < subSteps; ++i) {
		x = mIntegrator->rk4Integrator(x, u, mIntStep);
		projectToLimits(x);
	}
	setState(x);
	setInput(u);
}

//////////////////////////////////////////////////////////////////////////

StateVector SingleTrackAugDynModel::StateToVector(const AugStateStruct & s) const
{
	StateVector v(kNumStates);
	v << s.x, s.y, s.yaw, s.vx, s.vy, s.yaw_rate, s.steer, s.acc;
	return v;
}

//////////////////////////////////////////////////////////////////////////

AugStateStruct SingleTrackAugDynModel::VectorToState(const StateVector & v) const
{
	AugStateStruct s;
	s.x = v(0);
	s.y = v(1);
	s.yaw = v(2);
	s.vx = v(3);
	s.vy = v(4);
	s.yaw_rate = v(5);
	s.steer = v(6);
	s.acc = v(7);
	return s;
}

//////////////////////////////////////////////////////////////////////////

InputVector SingleTrackAugDynModel::InputToVector(const AugInputStruct & u) const
{
	InputVector v(kNumInputs);
	v << u.jerk, u.sv;
	return v;
}

//////////////////////////////////////////////////////////////////////////

AugInputStruct SingleTrackAugDynModel::VectorToInput(const InputVector & v) const
{
	return AugInputStruct{v(0), v(1)};
}

//////////////////////////////////////////////////////////////////////////

InputVector SingleTrackAugDynModel::packAccelSteerRate(double acc, double steeringRate) const
{
	if (mCommandMode != CommandMode::kAcceleration) {
		throw std::logic_error(
		    "SingleTrackAugDynModel takes [jerk, steer_rate]; an acceleration command cannot be "
		    "converted without the simulator's acceleration state. Set command_mode: "
		    "acceleration (with accelTimeConstant) for acceleration-commanded controllers.");
	}
	InputVector u(kNumInputs);
	u << acc, steeringRate;
	return u;
}

//////////////////////////////////////////////////////////////////////////

stPose SingleTrackAugDynModel::getStatePose() const
{
	return stPose(mStateStruct.x, mStateStruct.y, mStateStruct.z, mStateStruct.yaw, mStateStruct.steer);
}

//////////////////////////////////////////////////////////////////////////

void SingleTrackAugDynModel::setupRos(
    const rclcpp::Node::SharedPtr & node, const std::string & ns, const std::string & fixedFrame)
{
	DynamicModel::setupRos(node, ns, fixedFrame);
	mOdomPub = mNode->create_publisher<nav_msgs::msg::Odometry>("/" + mNamespace + "/odom", 10);
	mSteeringPub = mNode->create_publisher<project_utils_msgs::msg::SteeringReport>(
	    "/" + mNamespace + "/steering_report", 10);
}

//////////////////////////////////////////////////////////////////////////

void SingleTrackAugDynModel::publishStates() const
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

PLUGINLIB_EXPORT_CLASS(SingleTrackAugDynModel, DynamicModel)
