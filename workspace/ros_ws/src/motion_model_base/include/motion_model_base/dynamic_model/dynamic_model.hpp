/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include "project_utils/macros_expression.hpp"
#include "project_utils/pose_definition.hpp"
#include "project_utils/integrator.hpp"
#include "project_utils/unique_id.hpp"

#include <yaml-cpp/yaml.h>
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

  protected:
	ptSharedPtr<IntegratorClass> mIntegrator{nullptr};
	YAML::Node mSimConfig;
	YAML::Node mVehConfig;
	InputVector mCommandedControl;
	UniqueId mId;
};