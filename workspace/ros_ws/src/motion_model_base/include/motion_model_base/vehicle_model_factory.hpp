/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once

#include <string>

#include <pluginlib/class_loader.hpp>
#include <yaml-cpp/yaml.h>

#include "motion_model_base/agent_model.hpp"
#include "motion_model_base/collision_model/collision_footprint.hpp"
#include "motion_model_base/dynamic_model/dynamic_model.hpp"
#include "motion_model_base/geometric_model/geometric_model.hpp"
#include "motion_model_base/sensor_model/sensor_model.hpp"
#include "project_utils/logger.hpp"
#include "project_utils/unique_id.hpp"

// Builds one AgentModel per agent from YAML config, by loading four
// pluginlib-registered classes BY NAME -- never by #include. This class has
// zero compile-time knowledge of "ellipse" or "ackermann"; it only knows the
// four base interfaces it already owns. Concrete plugin names arrive at
// runtime as plain strings, read out of each agent's YAML block.
//
// Proof this doesn't create a dependency on motion_model_shapes /
// motion_model_ground_vehicles / motion_model_sensors: this file, and
// motion_model_base's package.xml/CMakeLists.txt, never mention any of
// them. The four ClassLoaders resolve concrete classes at runtime via the
// ament plugin index (built from every installed package's plugins.xml),
// not at compile time. See vehicle_model_factory.cpp for exactly where
// that lookup happens.
class VehicleModelFactory
{
    public:
        explicit VehicleModelFactory(mpl::rclcpp_utils::Logger & logger);
        ~VehicleModelFactory() = default;

        VehicleModelFactory(const VehicleModelFactory &) = delete;
        VehicleModelFactory & operator=(const VehicleModelFactory &) = delete;

        // simConfig taken by value (YAML::Node is a cheap handle type) so it
        // binds to DynamicModel::initialze()'s non-const YAML::Node& params.
        ptSharedPtr<AgentModel> create(
            YAML::Node simConfig, const YAML::Node & agentConfig, const UniqueId & id);

    private:
        mpl::rclcpp_utils::Logger mLogger;
        pluginlib::ClassLoader<DynamicModel> mDynamicsLoader;
        pluginlib::ClassLoader<GeometricModel> mGeometryLoader;
        pluginlib::ClassLoader<CollisionFootPrint> mCollisionLoader;
        pluginlib::ClassLoader<SensorModel> mSensorLoader;
};
