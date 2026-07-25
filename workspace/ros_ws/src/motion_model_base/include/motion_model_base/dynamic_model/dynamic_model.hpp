/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include "project_utils/integrator.hpp"
#include "project_utils/macros_expression.hpp"
#include "project_utils/pose_definition.hpp"
#include "project_utils/unique_id.hpp"

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <string>
////////////////////////////////////////////////////////////////////////////////

class DynamicModel
{
  public:
	// Constructor
	DynamicModel() = default;
	// Destructor
	virtual ~DynamicModel() = default;
	virtual void initialze(YAML::Node & simConfig, YAML::Node & vehConfig, const UniqueId & id) = 0;
	// DynamicModel(YAML::Node & simConfig, YAML::Node & vehConfig, const UniqueId & id)
	//     : mSimConfig{simConfig}, mVehConfig{vehConfig}, mId{id} {};
	// Update the control command
	virtual void updateCommandedControl(const InputVector & u) = 0;
	// Step forward through simulation
	virtual void step() = 0;
	// Get the vehicle's state.
	virtual const StateVector & getState() const = 0;
	// Get the pose of th vehicle.
	virtual stPose getStatePose() const = 0;
	// Set the state of the vehicle. (Usefull for setting the states through feedback, KF, etc)
	virtual void setState(const StateVector &) = 0;
	// Set the integrator for the state model.
	virtual void createIntegrator() = 0;
	// Wire up this model's own ROS I/O. Called once after initialze(), since
	// pluginlib default-constructs plugins (createSharedInstance takes no
	// args), so a node can't be threaded through the constructor. node/ns
	// let a concrete model create whatever publishers it needs -- not every
	// agent type publishes the same things -- namespaced consistently with
	// the rest of the sim (e.g. ns == "agent_3" -> topic "/agent_3/odom",
	// frame "agent_3_base_link"). Default is a no-op: models with nothing to
	// publish aren't forced to override it.
	virtual void setupRos(
	    const rclcpp::Node::SharedPtr & node, const std::string & ns, const std::string & fixedFrame)
	{
		mNode = node;
		mNamespace = ns;
		mFixedFrame = fixedFrame;
	}
	// pusblish the states (odom, steering feedback etc, whatever information we need to publish)
	virtual void publishStates() const = 0;
	// Pack a generic {acceleration, steering-rate} actuator command into this
	// vehicle's own InputVector layout. Lets a downstream controller (e.g.
	// pid_controller) go from a low-level PID output straight to the vector
	// AgentModel::updateCommandedControl()/agent_sim expect, without ever
	// knowing the concrete dynamics type -- avoids both duplicating each
	// vehicle's input ordering in every controller AND needing a
	// dynamic_cast (and therefore a direct link) to a concrete pluginlib
	// class from outside its own translation unit.
	virtual InputVector packAccelSteerRate(double acc, double steeringRate) const = 0;

  protected:
	ptSharedPtr<IntegratorClass> mIntegrator{nullptr};
	YAML::Node mSimConfig;
	YAML::Node mVehConfig;
	InputVector mCommandedControl;
	UniqueId mId;
	rclcpp::Node::SharedPtr mNode;
	std::string mNamespace;
	std::string mFixedFrame;
};