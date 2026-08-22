/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include "motion_model_base/world_snapshot.hpp"
#include "project_utils/pose_definition.hpp"
#include "project_utils/unique_id.hpp"
#include <vector>
#include <yaml-cpp/yaml.h>

/**
 * \brief Pluginlib base interface for an agent's sensor (e.g. \c LidarSensorModel).
 *
 * Composed inside \ref AgentModel (one sensor per agent, loaded by
 * `VehicleModelFactory` alongside the dynamics/geometry/collision plugins).
 * \ref step takes a \ref WorldSnapshot -- built once per tick by the
 * simulator and shared across every agent -- rather than depending on any
 * concrete scene type, so this stays swappable/testable independent of the
 * simulator.
 */
class SensorModel
{
  public:
	SensorModel() = default;
	virtual ~SensorModel() = default;
	/// \brief Initialize from this sensor's YAML config block.
	virtual void initialize(const YAML::Node & params) = 0;
	/**
	 * \brief Compute a fresh reading from \p pose against \p world.
	 * \param id The owning agent's own \ref UniqueId, so the sensor can
	 * skip its own shape in `world.agents`.
	 * \param pose The owning agent's current pose.
	 * \param world This tick's shared \ref WorldSnapshot, built once by the
	 * simulator and shared across every agent's sensors, not rebuilt per
	 * sensor.
	 */
	virtual void step(const UniqueId & id, const stPose & pose, const WorldSnapshot & world) = 0;
	/**
	 * \brief Last reading computed by \ref step.
	 * \return Meaning is sensor-specific (e.g. a lidar returns one range
	 * per beam) -- consult the concrete plugin.
	 */
	virtual const std::vector<float> & getReadings() const = 0;
};
