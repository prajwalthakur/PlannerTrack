/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
// Adapted from the Autoware Foundation
#pragma once
#include "motion_model_base/dynamic_model/dynamic_model.hpp"
#include "project_utils/main.hpp"

#include <Eigen/Dense>

#include <nav_msgs/msg/odometry.hpp>
#include <project_utils_msgs/msg/steering_report.hpp>

#include <yaml-cpp/yaml.h>

#include <cmath>

//////////////////////////////////////////////////////////////////////////
/*
 *    This is the master nonlinear plant model, in global frame, with a
 *    linear (small-slip-angle) tire model. It is the model actually
 *    integrated by xdot() below.
 *
 * x, y     : position (world frame)
 * yaw      : heading (world frame)
 * vx, vy   : body-frame longitudinal/lateral velocity
 * yaw_rate : yaw rate
 * steer    : front steering angle (state -- integrated from steering rate,
 *            matching every other actuator interface in this codebase:
 *            see DynamicModel::packAccelSteerRate())
 * acc      : commanded longitudinal acceleration (input)
 * sv       : commanded steering rate (input)
 * m        : mass
 * Iz       : yaw inertia
 * lf, lr   : distance from CG to front/rear axle
 * cf, cr   : front/rear tire cornering stiffness (linear tire model)
 *
 *    State & Input
 * x = [x, y, yaw, vx, vy, yaw_rate, steer]^T
 * u = [acc, sv]^T
 *
 *    Nonlinear dynamics
 * af           = steer - atan2(vy + lf*yaw_rate, vx)   (front slip angle)
 * ar           = -atan2(vy - lr*yaw_rate, vx)           (rear slip angle)
 * Fyf          = cf * af,  Fyr = cr * ar                (linear tire forces)
 * vx_dot       = acc + vy*yaw_rate
 * vy_dot       = (Fyf*cos(steer) + Fyr)/m - vx*yaw_rate
 * yaw_rate_dot = (lf*Fyf*cos(steer) - lr*Fyr) / Iz
 * x_dot        = vx*cos(yaw) - vy*sin(yaw)
 * y_dot        = vx*sin(yaw) + vy*cos(yaw)
 * yaw_dot      = yaw_rate
 * steer_dot    = sv
 *
 * Reference : Jarrod M. Snider, "Automatic Steering Methods for Autonomous Automobile Path
 * Tracking", Robotics Institute, Carnegie Mellon University, February 2009.
 *
 *    A controller (e.g. LQR) that wants the reduced 4-state path-error form
 *    below derives it by linearizing the above around a reference speed
 *    vr and dropping the steer/sv actuator states -- that reduction is the
 *    controller's own design-model concern, not this plant's.
 *
 * e, de, th, dth : lateral error, its rate, heading error, its rate
 * k              : curvature on reference trajectory point
 *
 * x = [e, de, th, dth]^T,  u = steer (commanded directly, no rate state)
 *
 *          [0,                   1,                0,                        0]       [       0] [
 * 0] dx/dt = [0,
 * -(cf+cr)/m/vr,        (cf+cr)/m,       (lr*cr-lf*cf)/m/vr] * x + [    cf/m] * u +
 * [(lr*cr-lf*cf)/m/vr*k - vr*k] [0, 0,                0,                        1]       [       0]
 * [                          0] [0, (lr*cr-lf*cf)/Iz/vr, (lf*cf-lr*cr)/Iz,
 * -(lf^2*cf+lr^2*cr)/Iz/vr]       [lf*cf/Iz]       [   -(lf^2*cf+lr^2*cr)/Iz/vr]
 */
struct StateStruct
{
	double x;
	double y;
	double z{0.0};
	double yaw;
	double vx;
	double vy;
	double yaw_rate;
	double steer;
};

//////////////////////////////////////////////////////////////////////////

struct InputStruct
{
	double acc;
	double sv;
};

//////////////////////////////////////////////////////////////////////////

class SingleTrackDynStateModel : public DynamicModel
{
	using BaseType = DynamicModel;

  public:
	// Constructor.
	SingleTrackDynStateModel() = default;

	// SingleTrackDynStateModel(YAML::Node & simConfig, YAML::Node & vehConfig, const UniqueId &
	// id);
	//  Destructor.
	~SingleTrackDynStateModel() = default;
	void initialze(YAML::Node & simConfig, YAML::Node & vehConfig, const UniqueId & id) override;
	// Step the simulation (updates the state space of the car)
	void step() override;
	// Update commanded control.
	void updateCommandedControl(const InputVector & u) override;
	// Get the current state of the vehicle.
	const StateVector & getState() const override;
	// Set the state of the vehicle.
	void setState(const StateVector &) override;
	// Get the input.
	const StateVector & getInput() const;
	// Set the input.
	void setInput(const InputVector &);
	// Get the pose of the vehicle
	stPose getStatePose() const override;
	// Vehicle Dynamics
	StateVector xdot(const StateVector &, const InputVector &) const;
	void createIntegrator() override;
	void setupRos(const rclcpp::Node::SharedPtr & node, const std::string & ns,
	    const std::string & fixedFrame) override;
	void publishStates() const override;
	InputVector packAccelSteerRate(double acc, double steeringRate) const override;

  private:
	void reset();
	void updateCommandedControl();
	void createIntegrator(YAML::Node & simConfig, YAML::Node & vehConfig);
	StateVector StateToVector(const StateStruct &) const;
	StateStruct VectorToState(const StateVector &) const;
	InputVector InputToVector(const InputStruct &) const;
	InputStruct VectorToInput(const InputVector &) const;

  private:
	int NX;
	int NU;
	float T_fwd;
	double mInitXPose;
	double mInitYPose;
	double mInitYaw;
	double mInitYawRate;
	double mInitVx;
	double mInitVy;
	double mInitSf;
	double mInitSv;
	double mInitAcc;
	double mMass;
	double mIz;
	double mLf;
	double mLr;
	double mCf;
	double mCr;
	InputStruct mInputStruct;
	StateStruct mStateStruct;
	InputVector mInputVector;
	StateVector mStateVector;

	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mOdomPub;
	rclcpp::Publisher<project_utils_msgs::msg::SteeringReport>::SharedPtr mSteeringPub;
};

//////////////////////////////////////////////////////////////////////////
