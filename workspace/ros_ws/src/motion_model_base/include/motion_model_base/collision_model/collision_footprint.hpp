/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include "motion_model_base/geometric_model/shape_descriptor.hpp"
#include "project_utils/macros_expression.hpp"
#include "project_utils/pose_definition.hpp"

#include <yaml-cpp/yaml.h>
// Collision foot print model
class CollisionFootPrint
{
  public:
	CollisionFootPrint() = default;
	virtual ~CollisionFootPrint() = default;
	virtual void initialize(const YAML::Node & params) = 0;
	virtual void step(const stPose & pose) = 0;
	// Exact shape, for consumers (e.g. visualization) that must not depend
	// on any concrete collision-footprint package. See shape_descriptor.hpp.
	virtual ShapeDescriptor describe() const = 0;
};
