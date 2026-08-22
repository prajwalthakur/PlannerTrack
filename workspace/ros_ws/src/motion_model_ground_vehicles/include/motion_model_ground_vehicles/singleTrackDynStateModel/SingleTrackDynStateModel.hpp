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
/**
 * \class SingleTrackDynStateModel
 * \brief Nonlinear single-track (bicycle) dynamic vehicle model with a
 * linear (small-slip-angle) tire model, in global frame.
 *
 * This is the master plant model -- the one actually integrated by
 * \ref SingleTrackDynStateModel::xdot() below.
 *
 * \verbatim
   x, y     : position (world frame)
   yaw      : heading (world frame)
   vx, vy   : body-frame longitudinal/lateral velocity
   yaw_rate : yaw rate
   steer    : front steering angle (state -- integrated from steering rate,
              matching every other actuator interface in this codebase:
              see DynamicModel::packAccelSteerRate())
   acc      : commanded longitudinal acceleration (input)
   sv       : commanded steering rate (input)
   m        : mass
   Iz       : yaw inertia
   lf, lr   : distance from CG to front/rear axle
   cf, cr   : front/rear tire cornering stiffness (linear tire model)
  
      State & Input
   x = [x, y, yaw, vx, vy, yaw_rate, steer]^T
   u = [acc, sv]^T
  
      Nonlinear dynamics
   af           = steer - atan2(vy + lf*yaw_rate, vx)   (front slip angle)
   ar           = -atan2(vy - lr*yaw_rate, vx)           (rear slip angle)
   Fyf          = cf * af,  Fyr = cr * ar                (linear tire forces)
   vx_dot       = acc + vy*yaw_rate
   vy_dot       = (Fyf*cos(steer) + Fyr)/m - vx*yaw_rate
   yaw_rate_dot = (lf*Fyf*cos(steer) - lr*Fyr) / Iz
   x_dot        = vx*cos(yaw) - vy*sin(yaw)
   y_dot        = vx*sin(yaw) + vy*cos(yaw)
   yaw_dot      = yaw_rate
   steer_dot    = sv
  
   Reference : Jarrod M. Snider, "Automatic Steering Methods for Autonomous Automobile Path
   Tracking", Robotics Institute, Carnegie Mellon University, February 2009.
  
      A controller (e.g. LQR) that wants the reduced 4-state path-error form
      below derives it by linearizing the above around a reference speed
      vr and dropping the steer/sv actuator states -- that reduction is the
      controller's own design-model concern, not this plant's.
  
   e, de, th, dth : lateral error, its rate, heading error, its rate
   k              : curvature on reference trajectory point
  
   x = [e, de, th, dth]^T,  u = steer (commanded directly, no rate state)
  
            [0,                   1,                0,                        0]       [       0] [
   0] dx/dt = [0,
   -(cf+cr)/m/vr,        (cf+cr)/m,       (lr*cr-lf*cf)/m/vr] * x + [    cf/m] * u +
   [(lr*cr-lf*cf)/m/vr*k - vr*k] [0, 0,                0,                        1]       [       0]
   [                          0] [0, (lr*cr-lf*cf)/Iz/vr, (lf*cf-lr*cr)/Iz,
   -(lf^2*cf+lr^2*cr)/Iz/vr]       [lf*cf/Iz]       [   -(lf^2*cf+lr^2*cr)/Iz/vr]
 * \endverbatim
 */
/// \brief State variables of \ref SingleTrackDynStateModel -- see that class for the full model.
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

/// \brief Input variables of \ref SingleTrackDynStateModel -- commanded acceleration and steering rate.
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
	SingleTrackDynStateModel() = default;

	// SingleTrackDynStateModel(YAML::Node & simConfig, YAML::Node & vehConfig, const UniqueId &
	// id);
	~SingleTrackDynStateModel() = default;
	void initialze(YAML::Node & simConfig, YAML::Node & vehConfig, const UniqueId & id) override;
	void step() override;
	void updateCommandedControl(const InputVector & u) override;
	const StateVector & getState() const override;
	void setState(const StateVector &) override;
	void reset() override;
	/// \brief Get the current commanded input, packed as a \c StateVector-shaped vector.
	const StateVector & getInput() const;
	/// \brief Set the commanded input directly (bypassing \ref updateCommandedControl).
	void setInput(const InputVector &);
	stPose getStatePose() const override;
	/**
	 * \brief Continuous-time state derivative of the nonlinear single-track
	 * model (see class doc for the equations), integrated by \ref step.
	 * \param state Current state `[x, y, yaw, vx, vy, yaw_rate, steer]`.
	 * \param input Current input `[acc, sv]`.
	 * \return State derivative, same layout as \p state.
	 */
	StateVector xdot(const StateVector & state, const InputVector & input) const;
	void createIntegrator() override;
	void setupRos(const rclcpp::Node::SharedPtr & node, const std::string & ns,
	    const std::string & fixedFrame) override;
	void publishStates() const override;
	InputVector packAccelSteerRate(double acc, double steeringRate) const override;

  private:
	/// \brief Apply \ref mInputStruct as the currently-commanded input (overload of the public setter).
	void updateCommandedControl();
	/// \brief Build the numerical integrator and seed init* state/input from YAML config.
	void createIntegrator(YAML::Node & simConfig, YAML::Node & vehConfig);
	/// \brief Convert a \ref StateStruct to the generic \c StateVector layout.
	StateVector StateToVector(const StateStruct &) const;
	/// \brief Convert a generic \c StateVector back to a \ref StateStruct.
	StateStruct VectorToState(const StateVector &) const;
	/// \brief Convert an \ref InputStruct to the generic \c InputVector layout.
	InputVector InputToVector(const InputStruct &) const;
	/// \brief Convert a generic \c InputVector back to an \ref InputStruct.
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
