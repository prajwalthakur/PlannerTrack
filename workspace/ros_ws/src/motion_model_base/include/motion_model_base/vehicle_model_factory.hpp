/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once

#include <string>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "motion_model_base/agent_model.hpp"
#include "motion_model_base/collision_model/collision_footprint.hpp"
#include "motion_model_base/dynamic_model/dynamic_model.hpp"
#include "motion_model_base/geometric_model/geometric_model.hpp"
#include "motion_model_base/sensor_model/sensor_model.hpp"
#include "project_utils/logger.hpp"
#include "project_utils/unique_id.hpp"

/**
 * \brief Builds one \ref AgentModel per agent from YAML config, by loading
 * four pluginlib-registered classes **by name** -- never by `#include`.
 *
 * This class has zero compile-time knowledge of "ellipse" or "ackermann"; it
 * only knows the four base interfaces it already owns
 * (\ref DynamicModel, \ref GeometricModel, \ref CollisionFootPrint,
 * \ref SensorModel). Concrete plugin names arrive at runtime as plain
 * strings, read out of each agent's YAML block.
 *
 * Proof this doesn't create a dependency on `motion_model_shapes` /
 * `motion_model_ground_vehicles` / `motion_model_sensors`: this file, and
 * `motion_model_base`'s `package.xml`/`CMakeLists.txt`, never mention any of
 * them. The four `ClassLoader`s resolve concrete classes at runtime via the
 * ament plugin index (built from every installed package's `plugins.xml`),
 * not at compile time -- see `vehicle_model_factory.cpp` for exactly where
 * that lookup happens, and \ref plugin_architecture for the full pattern.
 */
class VehicleModelFactory
{
    public:
        explicit VehicleModelFactory(mpl::rclcpp_utils::Logger & logger);
        ~VehicleModelFactory() = default;

        VehicleModelFactory(const VehicleModelFactory &) = delete;
        VehicleModelFactory & operator=(const VehicleModelFactory &) = delete;

        /**
         * \brief Construct one \ref AgentModel from an agent's YAML block,
         * loading its dynamics/geometry/collision/sensor plugins by name.
         * \param simConfig Sim-wide config; taken by value (`YAML::Node` is
         * a cheap handle type) so it binds to
         * `DynamicModel::initialze()`'s non-const `YAML::Node&` parameter.
         * \param agentConfig This agent's own config block, naming which
         * concrete plugin to load for each of the four interfaces.
         * \param id This agent's \ref UniqueId.
         * \param node Owning ROS node, forwarded to the dynamics plugin's
         * `setupRos()` so it can create its own namespaced publishers --
         * see \ref DynamicModel::setupRos for why this can't happen in a
         * constructor.
         * \param fixedFrame The simulation's fixed TF frame, forwarded the
         * same way.
         * \param createStPublisher Whether the dynamics plugin should set
         * up its state publishers (false for agents that don't need one).
         * \return A fully-wired \ref AgentModel for this agent.
         */
        ptSharedPtr<AgentModel> create(
            YAML::Node simConfig, const YAML::Node & agentConfig, const UniqueId & id,
            const rclcpp::Node::SharedPtr & node, const std::string & fixedFrame,bool createStPublisher=false);

    private:
        mpl::rclcpp_utils::Logger mLogger;
        pluginlib::ClassLoader<DynamicModel> mDynamicsLoader;
        pluginlib::ClassLoader<GeometricModel> mGeometryLoader;
        pluginlib::ClassLoader<CollisionFootPrint> mCollisionLoader;
        pluginlib::ClassLoader<SensorModel> mSensorLoader;
};
