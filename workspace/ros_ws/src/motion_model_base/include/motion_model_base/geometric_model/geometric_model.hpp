/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include "motion_model_base/geometric_model/shape_descriptor.hpp"
#include "project_utils/macros_expression.hpp"
#include "project_utils/pose_definition.hpp"
#include "project_utils/unique_id.hpp"

#include <yaml-cpp/yaml.h>
/**
 * \brief Pluginlib base interface for an agent's true/rendered geometric
 * shape (e.g. \c RectangularGeometricModel).
 */
class GeometricModel
{
  public:
	GeometricModel() = default;
	virtual ~GeometricModel() = default;
	/// \brief Initialize from this agent's YAML shape config.
	virtual void initialize(const YAML::Node & params, const UniqueId & id) = 0;
	// virtual void setCollisionFootPrint(const ptSharedPtr<CollisionFootPrint>
	// collisionFootPrint)=0; virtual ptwkPtr<CollisionFootPrint> getCollisionFootPrint()=0;
	/// \brief Update the shape's placement for this agent's current pose.
	virtual void step(const stPose & pose) = 0;
	/**
	 * \brief Describe the true/rendered shape.
	 * \return An exact \ref ShapeDescriptor, for consumers (e.g. sensors)
	 * that must not depend on any concrete shape package -- see
	 * `shape_descriptor.hpp`.
	 */
	virtual ShapeDescriptor describe() const = 0;
  protected:
	UniqueId mId;

};