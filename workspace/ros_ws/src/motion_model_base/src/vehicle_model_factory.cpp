/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "motion_model_base/vehicle_model_factory.hpp"

/** \file
 * \brief \ref VehicleModelFactory implementation -- the one place a
 * concrete plugin name (a YAML string) gets resolved via `pluginlib`. See
 * \ref plugin_architecture.
 */

//////////////////////////////////////////////////////////////////////////

VehicleModelFactory::VehicleModelFactory(mpl::rclcpp_utils::Logger & logger)
    : mLogger(logger),
      mDynamicsLoader("motion_model_base", "DynamicModel"),
      mGeometryLoader("motion_model_base", "GeometricModel"),
      mCollisionLoader("motion_model_base", "CollisionFootPrint"),
      mSensorLoader("motion_model_base", "SensorModel")
{
}

//////////////////////////////////////////////////////////////////////////

ptSharedPtr<AgentModel> VehicleModelFactory::create(
    YAML::Node simConfig, const YAML::Node & agentConfig, const UniqueId & id,
    const rclcpp::Node::SharedPtr & node, const std::string & fixedFrame, bool createStPublisher)
{
	// Each *_plugin value is data read from YAML -- e.g. "SingleTrackDynStateModel",
	// which physically lives in libmotion_model_ground_vehicles.so. This line is the
	// only place that string is ever interpreted: ClassLoader looks it up against the
	// ament plugin index, dlopen()s whichever .so exports it, and hands back a
	// shared_ptr<DynamicModel> -- the concrete type is never named here.
	const std::string dynType = agentConfig["dynamics_plugin"].as<std::string>();
	auto dyn = mDynamicsLoader.createSharedInstance(dynType);
	// Clone: the vehicle_limits block is attached for the plugin without
	// modifying the caller's agentConfig (YAML::Node copies share data).
	// vehicle limits are seperate, and we are not passing it downstream.
	// Now dynParams gets the vehicle limits also.
	YAML::Node dynParams = YAML::Clone(agentConfig["dynamics_params"]);
	if (agentConfig["vehicle_limits"] && !dynParams["vehicle_limits"]) {
		// One source for the limits: plugins that enforce actuator limits in
		// simulation (e.g. SingleTrackAugDynModel, clip_control_in_sim) read
		// the agent's vehicle_limits as dynamics_params.vehicle_limits.
		dynParams["vehicle_limits"] = YAML::Clone(agentConfig["vehicle_limits"]);
	}
	dyn->initialze(simConfig, dynParams, id);
	if(createStPublisher){
		const std::string ns = "agent_" + std::to_string(id.value());
		dyn->setupRos(node, ns, fixedFrame);
	}
	dyn->createIntegrator();

	const std::string geomType = agentConfig["geometry_plugin"].as<std::string>();
	auto geom = mGeometryLoader.createSharedInstance(geomType);
	geom->initialize(agentConfig["geometry_params"], id);

	const std::string collType = agentConfig["collision_plugin"].as<std::string>();
	auto coll = mCollisionLoader.createSharedInstance(collType);
	coll->initialize(agentConfig["collision_params"]);

	const std::string sensorType = agentConfig["sensor_plugin"].as<std::string>();
	auto sensor = mSensorLoader.createSharedInstance(sensorType);
	sensor->initialize(agentConfig["sensor_params"]);

	return std::make_shared<AgentModel>(mLogger, id, dyn, geom, coll, sensor);
}
