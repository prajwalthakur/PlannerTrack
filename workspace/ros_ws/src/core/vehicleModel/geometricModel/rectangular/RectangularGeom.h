
#pragma once
#include "EllipseCollisionFootPrint.h"
#include "core/CoreCollection.h"
#include "utils/UtilsCollection.h"
#include "vehicleModel/geometricModel/GeometricModel.h"

//////////////////////////////////////////////////////////////////////////

struct stVertices
{
	stPose frontLeft;
	stPose frontRight;
	stPose rearRight;
	stPose rearLeft;
};

//////////////////////////////////////////////////////////////////////////

class RectangularGeometry : public GeometricModel
{
  public:
	// Constructor.
	RectangularGeometry(const double length, const double width);
	// Destructor.
	virtual ~RectangularGeometry() = default;
	// Get the vertices of the rectangular geometric model.
	const std::shared_ptr<stVertices> getVertices() const;
	// Print the vertices.
	void printVertices() const;
	// Set the collision Footprint
	void setCollisionFootPrint(ptSharedPtr<CollisionFootPrint> collisionFootPrint) override;
	// Get the collision footprint
	std::weak_ptr<CollisionFootPrint> getCollisionFootPrint() override;
	// Update the geometric Model.
	void step(const ptSharedPtr<stPose> & pose) override;

  private:
	// Calculate the vertices.
	void calcVertices();

  private:
	double mLength{0.0};
	double mWidth{0.0};
	ptSharedPtr<stVertices> mVertices{nullptr};
	ptSharedPtr<CollisionFootPrint> mCollisionFootprint{nullptr};
	ptSharedPtr<stPose> mPose{nullptr};
};
