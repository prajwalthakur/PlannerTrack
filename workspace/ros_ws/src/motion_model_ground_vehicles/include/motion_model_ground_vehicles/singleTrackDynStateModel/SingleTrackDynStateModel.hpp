/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */

#pragma once
#include "motion_model_base/dynamic_model/dynamic_model.hpp"
#include "project_utils/main.hpp"

#include <Eigen/Dense>

#include <nav_msgs/msg/odometry.hpp>
#include <project_utils_msgs/msg/steering_report.hpp>

#include <yaml-cpp/yaml.h>

#include <cmath>

//////////////////////////////////////////////////////////////////////////

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
	double mInitVx;
	double mInitSf;
	double mInitSv;
	double mInitAcc;
	double mVehWheelBase;
	InputStruct mInputStruct;
	StateStruct mStateStruct;
	InputVector mInputVector;
	StateVector mStateVector;

	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mOdomPub;
	rclcpp::Publisher<project_utils_msgs::msg::SteeringReport>::SharedPtr mSteeringPub;
};

//////////////////////////////////////////////////////////////////////////
