/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */

#include "motion_model_shapes/geometric_model/rectangular_geometric_model.hpp"

#include <pluginlib/class_list_macros.hpp>

/** \file
 * \brief \ref RectangularGeometry implementation. `PLUGINLIB_EXPORT_CLASS`
 * at the bottom of this file registers the class with pluginlib at runtime
 * -- see \ref plugin_architecture.
 */

//////////////////////////////////////////////////////////////////////////

RectangularGeometry::RectangularGeometry(const double length, const double width)
{
	mLength = length;
	mWidth = width;
	mVertices = std::make_shared<stVertices>();
}

//////////////////////////////////////////////////////////////////////////

void RectangularGeometry::initialize(const YAML::Node & params, const UniqueId & id)
{
	mLength = params["length"].as<double>();
	mWidth = params["width"].as<double>();
	mId = id; // base
	mVertices = std::make_shared<stVertices>();
	// Must be non-null immediately, not just after the first step(): other
	// agents' sensors can call describe() on this object before this
	// agent's own step() has run for the first time this tick (WorldSnapshot
	// is built from every agent's current geometry before any of them step).
	mPose = std::make_shared<stPose>();
}
//////////////////////////////////////////////////////////////////////////

// void RectangularGeometry::setCollisionFootPrint(ptSharedPtr<CollisionFootPrint>
// collisionFootPrint)
// {
// 	if (collisionFootPrint == nullptr)
// 		std::cerr << " collisiont footprint is nullptr " << std::endl;
// 	mCollisionFootprint = collisionFootPrint;
// }

//////////////////////////////////////////////////////////////////////////

// std::weak_ptr<CollisionFootPrint> RectangularGeometry::getCollisionFootPrint()
// {
// 	return std::weak_ptr<CollisionFootPrint>(mCollisionFootprint);
// }

//////////////////////////////////////////////////////////////////////////

void RectangularGeometry::step(const stPose & pose)
{
	mPose = std::make_shared<stPose>(pose);
	// if (mCollisionFootprint == nullptr)
	// 	std::cerr << "[ Rectangular Geometric Model ]: Collision Footprint is nullptr, Please set "
	// 	             "the pointer.";
	// mCollisionFootprint->step(mPose);
	calcVertices();
	// printVertices();
}

//////////////////////////////////////////////////////////////////////////

void RectangularGeometry::calcVertices()
{
	// front left
	double xFl =
	    mPose->xCoord + (mLength / 2.0) * cos(mPose->yaw) - (mWidth / 2.0) * sin(mPose->yaw);
	double yFl =
	    mPose->yCoord + (mWidth / 2.0) * cos(mPose->yaw) + (mLength / 2.0) * sin(mPose->yaw);
	mVertices->frontLeft.setCoord(xFl, yFl, 0.0, mPose->yaw);
	// front right
	double xFr =
	    mPose->xCoord + (mLength / 2.0) * cos(mPose->yaw) + (mWidth / 2.0) * sin(mPose->yaw);
	double yFr =
	    mPose->yCoord - (mWidth / 2.0) * cos(mPose->yaw) + (mLength / 2.0) * sin(mPose->yaw);
	mVertices->frontRight.setCoord(xFr, yFr, 0.0, mPose->yaw);
	// rear right
	double xRr =
	    mPose->xCoord - (mLength / 2.0) * cos(mPose->yaw) + (mWidth / 2.0) * sin(mPose->yaw);
	double yRr =
	    mPose->yCoord - (mWidth / 2.0) * cos(mPose->yaw) - (mLength / 2.0) * sin(mPose->yaw);
	mVertices->rearRight.setCoord(xRr, yRr, 0.0, mPose->yaw);
	// rear left
	double xRl =
	    mPose->xCoord - (mLength / 2.0) * cos(mPose->yaw) - (mWidth / 2.0) * sin(mPose->yaw);
	double yRl =
	    mPose->yCoord + (mWidth / 2.0) * cos(mPose->yaw) - (mLength / 2.0) * sin(mPose->yaw);
	mVertices->rearLeft.setCoord(xRl, yRl, 0.0, mPose->yaw);
}

//////////////////////////////////////////////////////////////////////////

const std::shared_ptr<stVertices> RectangularGeometry::getVertices() const
{
	return mVertices;
}

//////////////////////////////////////////////////////////////////////////

void RectangularGeometry::printVertices() const
{

	const auto & fl = mVertices->frontLeft;
	const auto & fr = mVertices->frontRight;
	const auto & rl = mVertices->rearLeft;
	const auto & rr = mVertices->rearRight;
	std::cerr << "FL: (" << fl.xCoord << ", " << fl.yCoord << ")\n";
	std::cerr << "FR: (" << fr.xCoord << ", " << fr.yCoord << ")\n";
	std::cerr << "RL: (" << rl.xCoord << ", " << rl.yCoord << ")\n";
	std::cerr << "RR: (" << rr.xCoord << ", " << rr.yCoord << ")\n";
}

//////////////////////////////////////////////////////////////////////////

ShapeDescriptor RectangularGeometry::describe() const
{
	ShapeDescriptor desc;
	desc.kind = ShapeDescriptor::Kind::Rectangle;
	desc.id = mId;
	desc.rect.cx = mPose->xCoord;
	desc.rect.cy = mPose->yCoord;
	desc.rect.length = mLength;
	desc.rect.width = mWidth;
	desc.rect.phi = mPose->yaw;
	return desc;
}

//////////////////////////////////////////////////////////////////////////

PLUGINLIB_EXPORT_CLASS(RectangularGeometry, GeometricModel)
