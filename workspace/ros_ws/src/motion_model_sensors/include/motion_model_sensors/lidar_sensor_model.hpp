/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include "motion_model_base/sensor_model/sensor_model.hpp"
#include "project_utils/unique_id.hpp"
/**
 * \brief Brute-force 2D lidar \ref SensorModel plugin: every beam tested
 * against every shape in the \ref WorldSnapshot (agents + static
 * obstacles), no spatial acceleration structure.
 *
 * Ported from `agent_sim`'s original `LidarSensor`, dispatching on
 * `ShapeDescriptor::Kind` instead of concrete `WorldOBB`/`WorldCircle`
 * types so this package never depends on `motion_model_shapes` or any
 * other concrete geometry plugin.
 */
class LidarSensorModel : public SensorModel
{
  public:
	LidarSensorModel() = default;
	~LidarSensorModel() override = default;

	void initialize(const YAML::Node & params) override;
	void step(const UniqueId & id, const stPose & pose, const WorldSnapshot & world) override;
	const std::vector<float> & getReadings() const override;

  private:
	/// \brief Ray-rectangle intersection distance (slab method in the rectangle's local frame), or infinity if no hit.
	float rayRectangle(
	    float ox, float oy, float dx, float dy, const RectangleData & rect, float range_min) const;
	/// \brief Ray-circle intersection distance, or infinity if no hit.
	float rayCircle(
	    float ox, float oy, float dx, float dy, const CircleData & circle, float range_min) const;
	/**
	 * \brief Ray-polygon intersection distance: minimum over edges, or
	 * infinity if no hit. A 2-vertex polygon is treated as a single open
	 * segment (e.g. a wall), not a closed loop.
	 */
	float rayPolygon(float ox, float oy, float dx, float dy, const std::vector<stPose> & polygon,
	    float range_min) const;
	/// \brief Ray-line-segment intersection distance (Cramer's rule), or infinity if no hit.
	float raySegment(float ox, float oy, float dx, float dy, float x1, float y1, float x2, float y2,
	    float range_min) const;
	/// \brief Dispatch to the ray-intersection test matching \p shape's \c ShapeDescriptor::Kind.
	float rayShape(
	    float ox, float oy, float dx, float dy, const ShapeDescriptor & shape, float range_min) const;
	/**
	 * \brief Grid-cell DDA (Amanatides-Woo) march against a map-derived
	 * occupancy grid -- O(cells crossed) per beam, unlike \ref rayShape
	 * which is O(1) per shape but would need one shape per occupied pixel
	 * if the map were converted to rectangles instead.
	 */
	float rayGrid(float ox, float oy, float dx, float dy, const OccupancyGridMap & grid,
	    float range_min, float range_max) const;

  private:
	int mNumBeams{360};
	float mAngleMin{-3.14159265f};
	float mAngleMax{3.14159265f};
	float mRangeMin{0.05f};
	float mRangeMax{10.0f};
	std::vector<float> mRanges;
};
