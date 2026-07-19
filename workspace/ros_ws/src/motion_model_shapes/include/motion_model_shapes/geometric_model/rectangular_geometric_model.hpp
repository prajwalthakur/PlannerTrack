/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include "motion_model_base/geometric_model/geometric_model.hpp"
#include "project_utils/main.hpp"

struct stVertices
{
	stPose frontLeft;
	stPose frontRight;
	stPose rearRight;
	stPose rearLeft;
};

class RectangularGeometry : public GeometricModel
{
  public:
	// Constructor.
	RectangularGeometry() = default;
	RectangularGeometry(const double length, const double width);
	void initialize(const YAML::Node & params, const UniqueId & id) override;
	// Destructor.
	virtual ~RectangularGeometry() = default;
	// Get the vertices of the rectangular geometric model.
	const std::shared_ptr<stVertices> getVertices() const;
	// Print the vertices.
	void printVertices() const;
	// // Set the collision Footprint
	// void setCollisionFootPrint(ptSharedPtr<CollisionFootPrint> collisionFootPrint) override;
	// // Get the collision footprint
	// std::weak_ptr<CollisionFootPrint> getCollisionFootPrint() override;
	// Update the geometric Model.
	void step(const stPose & pose) override;
	ShapeDescriptor describe() const override;

  private:
	// Calculate the vertices.
	void calcVertices();

  private:
	double mLength{0.0};
	double mWidth{0.0};
	ptSharedPtr<stVertices> mVertices{nullptr};
	// ptSharedPtr<CollisionFootPrint> mCollisionFootprint{nullptr};
	ptSharedPtr<stPose> mPose{nullptr};
};
