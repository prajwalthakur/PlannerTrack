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
/**
 * \brief Composes one simulated agent's dynamics, geometry, collision
 * footprint, and sensor plugins behind a single non-virtual interface.
 *
 * `VehicleModelFactory` builds one `AgentModel` per agent listed in
 * `agents.yaml`, wiring in whichever concrete \ref DynamicModel,
 * \ref GeometricModel, \ref CollisionFootPrint, and \ref SensorModel plugins
 * that agent's config names. \ref step drives all four each simulation
 * tick; everything else forwards to the composed plugin or is a thin
 * accessor for `agent_sim` to publish state/markers from.
 */
class AgentModel
{
  public:
	/// \brief Construct from already-created plugin instances (see \ref VehicleModelFactory).
	AgentModel(mpl::rclcpp_utils::Logger & logger, const UniqueId & id,
	    ptSharedPtr<DynamicModel> dynamicModel, ptSharedPtr<GeometricModel> geometricModel,
	    ptSharedPtr<CollisionFootPrint> collisionModel, ptSharedPtr<SensorModel> sensorModel);
	~AgentModel() = default;

	/// \brief Forward a new commanded control input to the dynamics model.
	/// \param u Control input in this agent's own \c InputVector layout.
	void updateCommandedControl(const InputVector & u);

	/**
	 * \brief Advance dynamics, geometry, and collision footprint by one
	 * step, and optionally the sensor model.
	 * \param world This tick's shared \ref WorldSnapshot (other agents'
	 * geometry, static obstacles, map).
	 * \param stepSensorModel Set false to skip the (potentially expensive)
	 * sensor step, e.g. on ticks where no consumer needs a fresh reading.
	 */
	void step(const WorldSnapshot & world,bool stepSensorModel=true);

	/// \brief This agent's \ref UniqueId.
	const UniqueId & id() const;
	/**
	 * \brief Non-owning access to the true/rendered shape, for building a
	 * \ref WorldSnapshot.
	 * \warning Nothing outside \ref AgentModel should keep this reference
	 * alive past this \ref AgentModel's own lifetime (same reasoning as the
	 * constructor taking the plugin as a \c shared_ptr).
	 */
	const GeometricModel & geometry() const;
	/// \brief Non-owning access to this agent's collision footprint. Same lifetime caveat as \ref geometry.
	const CollisionFootPrint & collisionFootprint() const;
	/**
	 * \brief Non-owning access to this agent's sensor, e.g. for publishing
	 * its last computed reading (\c SensorModel::getReadings()). Same
	 * lifetime caveat as \ref geometry.
	 */
	const SensorModel & sensor() const;
	/**
	 * \brief Non-owning access to the dynamics model, e.g. for a downstream
	 * controller that needs to \c dynamic_cast to the concrete type to
	 * reach vehicle-specific helpers (\c InputStruct/\c InputToVector). Same
	 * lifetime caveat as \ref geometry.
	 */
	const DynamicModel & dynamicModel() const;
	/**
	 * \brief Current pose, delegated straight to the dynamics model.
	 * \return This agent's pose, for publishing (odometry, TF) -- not for
	 * driving simulation logic elsewhere.
	 */
	stPose getStatePose() const;

	/// \brief This agent's current state vector, delegated to the dynamics model.
	const StateVector & getState() const;
	/**
	 * \brief Respawn this agent at its configured init pose/velocity
	 * (`agents.yaml`'s `init*` fields).
	 *
	 * For an agent that has driven off the scenario, not for normal
	 * simulation.
	 */
	void resetToInitialState();
	// virtual void setState(const StateVector &) = 0;
	// virtual bool checkNumStatesInputs(
	//     const StateVector & stateVec, const InputVector & inputVec) = 0;
	// virtual std::pair<int, int> getNumStatesInputs() = 0;
	/// \brief Publish this agent's model-specific ROS output (odom, steering feedback, etc.).
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