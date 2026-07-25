/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include "motion_model_base/collision_model/collision_footprint.hpp"
#include "motion_model_base/dynamic_model/dynamic_model.hpp"
#include "motion_model_base/geometric_model/geometric_model.hpp"
#include "motion_model_base/sensor_model/sensor_model.hpp"
#include "project_utils/logger.hpp"
#include "project_utils/main.hpp"
class AgentModel
{
  public:
	// Constructor
	AgentModel(mpl::rclcpp_utils::Logger & logger, const UniqueId & id,
	    ptSharedPtr<DynamicModel> dynamicModel, ptSharedPtr<GeometricModel> geometricModel,
	    ptSharedPtr<CollisionFootPrint> collisionModel, ptSharedPtr<SensorModel> sensorModel);
	// Destructor
	~AgentModel() = default;

	// Update the control command
	void updateCommandedControl(const InputVector & u);
	// Virtual functions
	void step(const WorldSnapshot & world);

	// Get the id
	const UniqueId & id() const;
	// Non-owning access to the true/rendered shape, for building a
	// WorldSnapshot -- nothing outside AgentModel should keep this alive
	// past AgentModel's own lifetime (same reasoning as constructor params
	// being shared_ptr).
	const GeometricModel & geometry() const;
	// Non-owning access to this agent's collision footprint -- same lifetime
	// caveat as geometry().
	const CollisionFootPrint & collisionFootprint() const;
	// Non-owning access to this agent's sensor, e.g. for publishing its last
	// computed reading (SensorModel::getReadings()) -- same lifetime caveat
	// as geometry().
	const SensorModel & sensor() const;
	// Non-owning access to the dynamics model -- e.g. for a downstream
	// controller that needs to dynamic_cast to the concrete type to reach
	// vehicle-specific helpers (InputStruct/InputToVector). Same lifetime
	// caveat as geometry().
	const DynamicModel & dynamicModel() const;
	// Current pose/state, delegated straight to the dynamics model -- for
	// publishing (odometry, TF), not for driving simulation logic elsewhere.
	stPose getStatePose() const;
	const StateVector & getState() const;
	// virtual void setState(const StateVector &) = 0;
	// virtual bool checkNumStatesInputs(
	//     const StateVector & stateVec, const InputVector & inputVec) = 0;
	// virtual std::pair<int, int> getNumStatesInputs() = 0;
	void publishStates() const;

  protected:
	// YAML::Node mSimConfig;
	// YAML::Node mVehConfig;
	// int mNumStates{0};
	// int mNumControlInputs{0};
	mpl::rclcpp_utils::Logger mLogger;
	UniqueId mId;
	ptSharedPtr<DynamicModel> mDynamicModel{nullptr};  // state space model of vehicle
	ptSharedPtr<GeometricModel> mGeomModel{nullptr};  // Geometric Model, contains geometric shape a
	ptSharedPtr<CollisionFootPrint> mCollisionFootPrint{nullptr};
	ptSharedPtr<SensorModel> mSensorModel{nullptr};

};  // AgentModel