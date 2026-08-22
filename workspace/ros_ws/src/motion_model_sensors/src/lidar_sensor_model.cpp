/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "motion_model_sensors/lidar_sensor_model.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

/** \file
 * \brief \ref LidarSensorModel implementation: per-beam ray-shape/ray-grid
 * intersection tests and the main scan loop. `PLUGINLIB_EXPORT_CLASS` at
 * the bottom registers the class with pluginlib at runtime -- see
 * \ref plugin_architecture.
 */

namespace
{
constexpr float kInf = std::numeric_limits<float>::infinity();
}

//////////////////////////////////////////////////////////////////////////

void LidarSensorModel::initialize(const YAML::Node & params)
{
	if (params["num_beams"]) mNumBeams = params["num_beams"].as<int>();
	if (params["angle_min"]) mAngleMin = params["angle_min"].as<float>();
	if (params["angle_max"]) mAngleMax = params["angle_max"].as<float>();
	if (params["range_min"]) mRangeMin = params["range_min"].as<float>();
	if (params["range_max"]) mRangeMax = params["range_max"].as<float>();
	mRanges.assign(mNumBeams, mRangeMax);
}

//////////////////////////////////////////////////////////////////////////
// Ray-Circle intersection

//////////////////////////////////////////////////////////////////////////

float LidarSensorModel::rayCircle(
    float ox, float oy, float dx, float dy, const CircleData & circle, float range_min) const
{
	const float cx = static_cast<float>(circle.cx);
	const float cy = static_cast<float>(circle.cy);
	const float r = static_cast<float>(circle.radius);

	const float vx = ox - cx;
	const float vy = oy - cy;
	const float b = 2.0f * (vx * dx + vy * dy);
	const float c = vx * vx + vy * vy - r * r;
	const float disc = b * b - 4.0f * c;
	if (disc < 0.0f) {
		return kInf;
	}
	const float t = (-b - std::sqrt(disc)) * 0.5f;
	return (t > range_min) ? t : kInf;
}

//////////////////////////////////////////////////////////////////////////
// Ray-Line Segment intersection (Cramer's rule)

//////////////////////////////////////////////////////////////////////////

float LidarSensorModel::raySegment(float ox, float oy, float dx, float dy, float x1, float y1,
    float x2, float y2, float range_min) const
{
	const float ex = x2 - x1;
	const float ey = y2 - y1;
	const float fx = ox - x1;
	const float fy = oy - y1;

	// denom = (Q-P) x d  (2D cross product); sign gives ray/segment orientation
	const float denom = dy * ex - dx * ey;
	if (std::fabs(denom) < 1e-9f) {
		return kInf;  // ray parallel to segment
	}

	const float t = (fx * ey - fy * ex) / denom;
	const float s = (fx * dy - fy * dx) / denom;

	if (t > range_min && s >= 0.0f && s <= 1.0f) {
		return t;
	}
	return kInf;
}

//////////////////////////////////////////////////////////////////////////
// Ray-Rectangle intersection (slab method in local frame)

//////////////////////////////////////////////////////////////////////////

float LidarSensorModel::rayRectangle(
    float ox, float oy, float dx, float dy, const RectangleData & rect, float range_min) const
{
	const float cos_phi = std::cos(static_cast<float>(rect.phi));
	const float sin_phi = std::sin(static_cast<float>(rect.phi));
	const float hl = static_cast<float>(rect.length) * 0.5f;
	const float hw = static_cast<float>(rect.width) * 0.5f;

	const float tx = ox - static_cast<float>(rect.cx);
	const float ty = oy - static_cast<float>(rect.cy);

	const float lox = cos_phi * tx + sin_phi * ty;
	const float loy = -sin_phi * tx + cos_phi * ty;
	const float ldx = cos_phi * dx + sin_phi * dy;
	const float ldy = -sin_phi * dx + cos_phi * dy;

	// Slab test on axis 0 (half-length hl, along local-x)
	float t_near, t_far;
	if (std::fabs(ldx) < 1e-9f) {
		if (lox < -hl || lox > hl) {
			return kInf;
		}
		t_near = -kInf;
		t_far = kInf;
	} else {
		float t0 = (-hl - lox) / ldx;
		float t1 = (hl - lox) / ldx;
		if (t0 > t1) {
			std::swap(t0, t1);
		}
		t_near = t0;
		t_far = t1;
	}

	// Slab test on axis 1 (half-width hw, along local-y)
	if (std::fabs(ldy) < 1e-9f) {
		if (loy < -hw || loy > hw) {
			return kInf;
		}
	} else {
		float t0 = (-hw - loy) / ldy;
		float t1 = (hw - loy) / ldy;
		if (t0 > t1) {
			std::swap(t0, t1);
		}
		t_near = std::max(t_near, t0);
		t_far = std::min(t_far, t1);
	}

	if (t_far < t_near || t_near < range_min) {
		return kInf;
	}
	return t_near;
}

//////////////////////////////////////////////////////////////////////////
// Ray-Polygon intersection: min over edges. A 2-vertex polygon is treated
// as a single open segment (e.g. a wall), not a closed loop.

//////////////////////////////////////////////////////////////////////////

float LidarSensorModel::rayPolygon(float ox, float oy, float dx, float dy,
    const std::vector<stPose> & polygon, float range_min) const
{
	const size_t n = polygon.size();
	if (n < 2) {
		return kInf;
	}
	const size_t edges = (n == 2) ? 1 : n;

	float t_min = kInf;
	for (size_t i = 0; i < edges; ++i) {
		const auto & a = polygon[i];
		const auto & b = polygon[(i + 1) % n];
		const float t =
		    raySegment(ox, oy, dx, dy, static_cast<float>(a.xCoord), static_cast<float>(a.yCoord),
		        static_cast<float>(b.xCoord), static_cast<float>(b.yCoord), range_min);
		if (t < t_min) {
			t_min = t;
		}
	}
	return t_min;
}

//////////////////////////////////////////////////////////////////////////

float LidarSensorModel::rayGrid(float ox, float oy, float dx, float dy,
    const OccupancyGridMap & grid, float range_min, float range_max) const
{
	const float res = grid.resolution;
	const float originX = static_cast<float>(grid.originX);
	const float originY = static_cast<float>(grid.originY);

	int cellX = static_cast<int>(std::floor((ox - originX) / res));
	int cellY = static_cast<int>(std::floor((oy - originY) / res));

	const int stepX = (dx > 0.0f) ? 1 : (dx < 0.0f ? -1 : 0);
	const int stepY = (dy > 0.0f) ? 1 : (dy < 0.0f ? -1 : 0);

	// t-distance (world units) to cross one full cell along each axis.
	const float tDeltaX = (dx != 0.0f) ? std::fabs(res / dx) : kInf;
	const float tDeltaY = (dy != 0.0f) ? std::fabs(res / dy) : kInf;

	// t-distance to the first cell boundary crossed along each axis.
	const float nextBoundaryX = originX + static_cast<float>(cellX + (stepX > 0 ? 1 : 0)) * res;
	const float nextBoundaryY = originY + static_cast<float>(cellY + (stepY > 0 ? 1 : 0)) * res;
	float tMaxX = (dx != 0.0f) ? (nextBoundaryX - ox) / dx : kInf;
	float tMaxY = (dy != 0.0f) ? (nextBoundaryY - oy) / dy : kInf;

	float t = 0.0f;
	while (t <= range_max) {
		if (t > range_min && grid.isOccupied(cellX, cellY)) {
			return t;
		}
		if (tMaxX < tMaxY) {
			t = tMaxX;
			cellX += stepX;
			tMaxX += tDeltaX;
		} else {
			t = tMaxY;
			cellY += stepY;
			tMaxY += tDeltaY;
		}
	}
	return kInf;
}

//////////////////////////////////////////////////////////////////////////

float LidarSensorModel::rayShape(
    float ox, float oy, float dx, float dy, const ShapeDescriptor & shape, float range_min) const
{
	switch (shape.kind) {
		case ShapeDescriptor::Kind::Rectangle:
			return rayRectangle(ox, oy, dx, dy, shape.rect, range_min);
		case ShapeDescriptor::Kind::Circle:
			return rayCircle(ox, oy, dx, dy, shape.circle, range_min);
		case ShapeDescriptor::Kind::Polygon:
			return rayPolygon(ox, oy, dx, dy, shape.polygon, range_min);
		case ShapeDescriptor::Kind::Ellipse:
			// CollisionFootPrint::describe() can return this, but only
			// GeometricModel::describe() ever feeds a WorldSnapshot -- a
			// lidar never actually sees this case today.
			break;
	}
	return kInf;
}

//////////////////////////////////////////////////////////////////////////
// Main scan computation

//////////////////////////////////////////////////////////////////////////

void LidarSensorModel::step(const UniqueId & id, const stPose & pose, const WorldSnapshot & world)
{
	const float ox = static_cast<float>(pose.xCoord);
	const float oy = static_cast<float>(pose.yCoord);
	const float phi = static_cast<float>(pose.yaw);

	const float angle_inc =
	    (mNumBeams > 1) ? (mAngleMax - mAngleMin) / static_cast<float>(mNumBeams) : 0.0f;

	// describe() is a real virtual call with per-shape work behind it -- call
	// it once per agent per tick here, not once per beam per agent below.
	std::vector<ShapeDescriptor> shapes;
	shapes.reserve(world.agents.size() + world.staticObstacles.size());
	for (const auto * agentShape : world.agents) {
		if (agentShape != nullptr) {
			if (id == agentShape->describe().id) continue;
			shapes.push_back(agentShape->describe());
		}
	}
	shapes.insert(shapes.end(), world.staticObstacles.begin(), world.staticObstacles.end());

	mRanges.assign(mNumBeams, mRangeMax);

	for (int k = 0; k < mNumBeams; ++k) {
		const float world_angle = phi + mAngleMin + static_cast<float>(k) * angle_inc;
		const float dx = std::cos(world_angle);
		const float dy = std::sin(world_angle);

		float t_min = mRangeMax;
		for (const auto & shape : shapes) {
			const float t = rayShape(ox, oy, dx, dy, shape, mRangeMin);
			if (t < t_min) {
				t_min = t;
			}
		}
		if (world.mapGrid != nullptr && world.mapGrid->valid()) {
			const float t = rayGrid(ox, oy, dx, dy, *world.mapGrid, mRangeMin, t_min);
			if (t < t_min) {
				t_min = t;
			}
		}
		mRanges[k] = std::min(t_min, mRangeMax);
	}
}

//////////////////////////////////////////////////////////////////////////

const std::vector<float> & LidarSensorModel::getReadings() const
{
	return mRanges;
}

//////////////////////////////////////////////////////////////////////////

PLUGINLIB_EXPORT_CLASS(LidarSensorModel, SensorModel)
