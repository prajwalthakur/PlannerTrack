/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include "motion_model_base/geometric_model/shape_descriptor.hpp"
#include "project_utils/macros_expression.hpp"
#include "project_utils/pose_definition.hpp"

#include <yaml-cpp/yaml.h>
/**
 * \brief Pluginlib base interface for an agent's collision/safety-margin
 * footprint (e.g. \c EllipseCollisionFootprint), kept distinct from its
 * true/rendered \ref GeometricModel shape.
 */
class CollisionFootPrint
{
  public:
	CollisionFootPrint() = default;
	virtual ~CollisionFootPrint() = default;
	/// \brief Initialize from this agent's YAML footprint config.
	virtual void initialize(const YAML::Node & params) = 0;
	/// \brief Update the footprint's placement for this agent's current pose.
	virtual void step(const stPose & pose) = 0;
	/**
	 * \brief Describe the footprint shape.
	 * \return An exact \ref ShapeDescriptor, for consumers (e.g.
	 * visualization) that must not depend on any concrete
	 * collision-footprint package -- see `shape_descriptor.hpp`.
	 */
	virtual ShapeDescriptor describe() const = 0;
};
