/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
// Adapated from autoware IV
#pragma once
#include "motion_model_base/dynamic_model/dynamic_model.hpp"
#include "project_utils/main.hpp"

#include <Eigen/Dense>

#include <nav_msgs/msg/odometry.hpp>
#include <project_utils_msgs/msg/steering_report.hpp>

#include <yaml-cpp/yaml.h>

#include <cmath>

//////////////////////////////////////////////////////////////////////////


/// \brief State variables of \ref BicycleKinematicModel -- pose plus longitudinal speed and steering angle.
struct StateStruct
{
	double x;
	double y;
	double z{0.0};
	double yaw;
	double vx;
	double sf;
};

//////////////////////////////////////////////////////////////////////////

/// \brief Input variables of \ref BicycleKinematicModel -- commanded acceleration and steering rate.
struct InputStruct
{
	double acc;
	double sv;
};

//////////////////////////////////////////////////////////////////////////

/**
 * \brief \ref DynamicModel plugin implementing the kinematic bicycle model:
 * state `[x, y, yaw, vx, sf]`, input `[acc, sv]` (steering angle `sf` is a
 * state, integrated from commanded steering rate `sv`, matching every other
 * actuator interface in this codebase -- see
 * `DynamicModel::packAccelSteerRate()`).
 *
 * \verbatim
   xdot   = vx * cos(yaw)
   ydot   = vx * sin(yaw)
   yawdot = vx * tan(sf) / wheelbase
   vxdot  = acc
   sfdot  = sv
   \endverbatim
 *
 * No tire/slip model -- see \ref SingleTrackDynStateModel for the dynamic
 * (slip-angle) alternative.
 */
class BicycleKinematicModel : public DynamicModel
{
	using BaseType = DynamicModel;

  public:
	BicycleKinematicModel() = default;

	// BicycleKinematicModel(YAML::Node & simConfig, YAML::Node & vehConfig, const UniqueId &
	// id);
	~BicycleKinematicModel() = default;
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
	 * \brief Continuous-time state derivative of the kinematic bicycle
	 * model (see class doc for the equations), integrated by \ref step.
	 * \param state Current state `[x, y, yaw, vx, sf]`.
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
	/// \brief Apply the currently-set input as the commanded control (overload of the public setter).
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
	double mInitVx;
	double mInitSf;
	double mInitSv;
	double mInitAcc;
	double mVehWheelBase;
	double mSterringTau;
	InputStruct mInputStruct;
	StateStruct mStateStruct;
	InputVector mInputVector;
	StateVector mStateVector;

	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mOdomPub;
	rclcpp::Publisher<project_utils_msgs::msg::SteeringReport>::SharedPtr mSteeringPub;
};

//////////////////////////////////////////////////////////////////////////
