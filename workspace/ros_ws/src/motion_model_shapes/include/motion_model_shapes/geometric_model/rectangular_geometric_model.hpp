/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include "motion_model_base/geometric_model/geometric_model.hpp"
#include "project_utils/main.hpp"

/// \brief The four corners of a \ref RectangularGeometry, in world frame.
struct stVertices
{
	stPose frontLeft;
	stPose frontRight;
	stPose rearRight;
	stPose rearLeft;
};

/**
 * \brief \ref GeometricModel plugin representing an agent's true/rendered
 * shape as an oriented rectangle (length x width).
 */
class RectangularGeometry : public GeometricModel
{
  public:
	RectangularGeometry() = default;
	RectangularGeometry(const double length, const double width);
	void initialize(const YAML::Node & params, const UniqueId & id) override;
	virtual ~RectangularGeometry() = default;
	/// \brief Get the current world-frame corner vertices.
	const std::shared_ptr<stVertices> getVertices() const;
	/// \brief Print the current corner vertices (debugging).
	void printVertices() const;
	// // Set the collision Footprint
	// void setCollisionFootPrint(ptSharedPtr<CollisionFootPrint> collisionFootPrint) override;
	// // Get the collision footprint
	// std::weak_ptr<CollisionFootPrint> getCollisionFootPrint() override;
	void step(const stPose & pose) override;
	ShapeDescriptor describe() const override;

  private:
	/// \brief Recompute \ref mVertices from \ref mPose and the rectangle's length/width.
	void calcVertices();

  private:
	double mLength{0.0};
	double mWidth{0.0};
	ptSharedPtr<stVertices> mVertices{nullptr};
	// ptSharedPtr<CollisionFootPrint> mCollisionFootprint{nullptr};
	ptSharedPtr<stPose> mPose{nullptr};
};
