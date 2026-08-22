/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include "project_utils/pose_definition.hpp"
#include "project_utils/unique_id.hpp"
#include <vector>

/**
 * \brief Exact shape data a \ref GeometricModel exposes to consumers
 * (sensors, visualization, ...) that must not depend on any concrete shape
 * package.
 *
 * A generic vertex list would be lossy for circles, and `dynamic_cast` to a
 * concrete type would force a real build dependency from the consumer onto
 * every shape package -- this tagged variant (see \ref ShapeDescriptor)
 * avoids both.
 */
struct RectangleData
{
	double cx{0.0}, cy{0.0};  // center
	double length{0.0}, width{0.0};
	double phi{0.0};  // heading (radians)
};

struct CircleData
{
	double cx{0.0}, cy{0.0};
	double radius{0.0};
};

struct EllipseData
{
	double cx{0.0}, cy{0.0};  // center
	double majorAxisLength{0.0}, minorAxisLength{0.0};  // full lengths, not semi-axes
	double phi{0.0};  // heading (radians)
};

/// \brief Tagged-union shape description; only the member matching \ref kind is valid.
struct ShapeDescriptor
{
	enum class Kind { Rectangle, Circle, Polygon, Ellipse } kind{Kind::Polygon};
	RectangleData rect;  // valid iff kind == Rectangle
	CircleData circle;  // valid iff kind == Circle
	EllipseData ellipse;  // valid iff kind == Ellipse
	/// Fallback for anything else. Also used for line-segment obstacles (a
	/// 2-vertex "polygon" is treated as an open segment, not a closed loop).
	std::vector<stPose> polygon;
	UniqueId id;
};
