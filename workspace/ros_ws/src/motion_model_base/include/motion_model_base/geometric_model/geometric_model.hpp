/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include "motion_model_base/geometric_model/shape_descriptor.hpp"
#include "project_utils/macros_expression.hpp"
#include "project_utils/pose_definition.hpp"
#include "project_utils/unique_id.hpp"

#include <yaml-cpp/yaml.h>
// Geometric Model
class GeometricModel
{
  public:
	GeometricModel() = default;
	virtual ~GeometricModel() = default;
	virtual void initialize(const YAML::Node & params, const UniqueId & id) = 0;
	// virtual void setCollisionFootPrint(const ptSharedPtr<CollisionFootPrint>
	// collisionFootPrint)=0; virtual ptwkPtr<CollisionFootPrint> getCollisionFootPrint()=0;
	virtual void step(const stPose & pose) = 0;
	// True/rendered shape, exact -- for consumers (sensors) that must not
	// depend on any concrete shape package. See shape_descriptor.hpp.
	virtual ShapeDescriptor describe() const = 0;
  protected:
	UniqueId mId;

};