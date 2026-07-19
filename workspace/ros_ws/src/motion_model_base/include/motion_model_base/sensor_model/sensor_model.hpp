/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include "motion_model_base/world_snapshot.hpp"
#include "project_utils/pose_definition.hpp"
#include "project_utils/unique_id.hpp"
#include <vector>
#include <yaml-cpp/yaml.h>

// Sensor model. Composed inside AgentModel (one sensor per agent, loaded by
// VehicleModelFactory alongside the dynamics/geometry/collision plugins).
// step() takes a WorldSnapshot -- built once per tick by the simulator and
// shared across every agent -- rather than depending on any concrete scene
// type, so this stays swappable/testable independent of the simulator.
class SensorModel
{
  public:
	SensorModel() = default;
	virtual ~SensorModel() = default;
	virtual void initialize(const YAML::Node & params) = 0;
	// Computes a fresh reading from `pose` against `world` (see
	// WorldSnapshot -- built once per tick by the simulator, shared across
	// every agent's sensors, not rebuilt per sensor). `id` is the owning
	// agent's own id, so the sensor can skip its own shape in `world.agents`.
	virtual void step(const UniqueId & id, const stPose & pose, const WorldSnapshot & world) = 0;
	// Last reading computed by step(). Meaning is sensor-specific (e.g. a
	// lidar returns one range per beam) -- consult the concrete plugin.
	virtual const std::vector<float> & getReadings() const = 0;
};
