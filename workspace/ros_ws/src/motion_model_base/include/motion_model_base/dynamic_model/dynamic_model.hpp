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

/**
 * \brief Pluginlib base interface for a vehicle's state-space dynamics
 * model (e.g. \c SingleTrackDynStateModel, \c BicycleKinematicModel).
 *
 * Loaded by \c VehicleModelFactory / \c pluginlib, one instance per agent.
 * See \ref plugin_architecture for why concrete implementations must be
 * `SHARED` libraries and why consumers only ever depend on this interface.
 */
class DynamicModel
{
  public:
	DynamicModel() = default;
	virtual ~DynamicModel() = default;

	/// \brief Initialize from this vehicle's sim-wide and per-vehicle YAML config.
	/// \param simConfig Sim-wide config (e.g. timestep).
	/// \param vehConfig This vehicle's own config block from `agents.yaml`.
	/// \param id This agent's \ref UniqueId.
	virtual void initialze(YAML::Node & simConfig, YAML::Node & vehConfig, const UniqueId & id) = 0;
	// DynamicModel(YAML::Node & simConfig, YAML::Node & vehConfig, const UniqueId & id)
	//     : mSimConfig{simConfig}, mVehConfig{vehConfig}, mId{id} {};

	/// \brief Update the commanded control input for the next \ref step.
	virtual void updateCommandedControl(const InputVector & u) = 0;
	/// \brief Advance the vehicle's state by one simulation timestep.
	virtual void step() = 0;
	/// \brief Get the vehicle's current state vector.
	virtual const StateVector & getState() const = 0;
	/// \brief Get the vehicle's current pose.
	virtual stPose getStatePose() const = 0;
	/// \brief Set the vehicle's state directly (e.g. from external feedback/a Kalman filter).
	virtual void setState(const StateVector &) = 0;
	/**
	 * \brief Snap state/input back to the `init*` values from this
	 * vehicle's YAML config (the same values \ref createIntegrator seeds on
	 * startup).
	 *
	 * For respawning an agent that has driven off the scenario (e.g. past
	 * the lane extents), not for normal simulation.
	 */
	virtual void reset() = 0;
	/// \brief Construct and assign this model's numerical integrator.
	virtual void createIntegrator() = 0;
	/**
	 * \brief Wire up this model's own ROS I/O (called once, after \ref initialze).
	 *
	 * Split out from the constructor because `pluginlib` default-constructs
	 * plugins (`createSharedInstance` takes no args), so a node can't be
	 * threaded through the constructor. \p node / \p ns let a concrete model
	 * create whatever publishers it needs -- not every agent type publishes
	 * the same things -- namespaced consistently with the rest of the sim
	 * (e.g. `ns == "agent_3"` -> topic `/agent_3/odom`, frame
	 * `agent_3_base_link`). Default is a no-op: models with nothing to
	 * publish aren't forced to override it.
	 * \param node Owning ROS node, for creating publishers/subscriptions.
	 * \param ns This agent's topic/frame namespace.
	 * \param fixedFrame The simulation's fixed TF frame.
	 */
	virtual void setupRos(
	    const rclcpp::Node::SharedPtr & node, const std::string & ns, const std::string & fixedFrame)
	{
		mNode = node;
		mNamespace = ns;
		mFixedFrame = fixedFrame;
	}
	/// \brief Publish this model's ROS output (odom, steering feedback, etc.).
	virtual void publishStates() const = 0;
	/**
	 * \brief Pack a generic {acceleration, steering-rate} actuator command
	 * into this vehicle's own \c InputVector layout.
	 *
	 * Lets a downstream controller (e.g. `pid_controller`) go from a
	 * low-level PID output straight to the vector
	 * `AgentModel::updateCommandedControl()`/`agent_sim` expect, without
	 * ever knowing the concrete dynamics type -- avoids both duplicating
	 * each vehicle's input ordering in every controller AND needing a
	 * `dynamic_cast` (and therefore a direct link) to a concrete pluginlib
	 * class from outside its own translation unit.
	 * \param acc Commanded longitudinal acceleration.
	 * \param steeringRate Commanded steering rate.
	 * \return The command packed into this vehicle's \c InputVector layout.
	 */
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