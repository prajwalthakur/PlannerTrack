/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include "motion_model_base/sensor_model/sensor_model.hpp"
#include "project_utils/unique_id.hpp"
// Brute-force 2D lidar: every beam tested against every shape in the
// WorldSnapshot (agents + static obstacles), no spatial acceleration
// structure. Ported from agent_sim's original LidarSensor, dispatching on
// ShapeDescriptor::Kind instead of concrete WorldOBB/WorldCircle types so
// this package never depends on motion_model_shapes or any other concrete
// geometry plugin.
class LidarSensorModel : public SensorModel
{
  public:
	LidarSensorModel() = default;
	~LidarSensorModel() override = default;

	void initialize(const YAML::Node & params) override;
	void step(const UniqueId & id, const stPose & pose, const WorldSnapshot & world) override;
	const std::vector<float> & getReadings() const override;

  private:
	float rayRectangle(
	    float ox, float oy, float dx, float dy, const RectangleData & rect, float range_min) const;
	float rayCircle(
	    float ox, float oy, float dx, float dy, const CircleData & circle, float range_min) const;
	float rayPolygon(float ox, float oy, float dx, float dy, const std::vector<stPose> & polygon,
	    float range_min) const;
	float raySegment(float ox, float oy, float dx, float dy, float x1, float y1, float x2, float y2,
	    float range_min) const;
	float rayShape(
	    float ox, float oy, float dx, float dy, const ShapeDescriptor & shape, float range_min) const;

  private:
	int mNumBeams{360};
	float mAngleMin{-3.14159265f};
	float mAngleMax{3.14159265f};
	float mRangeMin{0.05f};
	float mRangeMax{10.0f};
	std::vector<float> mRanges;
};
