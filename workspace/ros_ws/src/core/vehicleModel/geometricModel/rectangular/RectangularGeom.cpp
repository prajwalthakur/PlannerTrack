
#include "RectangularGeom.h"

//////////////////////////////////////////////////////////////////////////

RectangularGeomClass::RectangularGeomClass(const double length, const double width)
{
    mLength = length;
    mWidth = width;
    mVertices = std::make_shared<stVertices>();
}

//////////////////////////////////////////////////////////////////////////

void RectangularGeomClass::setCollisionFootPrint( ptSharedPtr<CollisionFootPrint> collisionFootPrint)
{
    if(collisionFootPrint==nullptr)
        std::cerr << " collisiont footprint is nullptr " << std::endl;
    mCollisionFootprint = collisionFootPrint;
}

//////////////////////////////////////////////////////////////////////////

std::weak_ptr<CollisionFootPrint> RectangularGeomClass::getCollisionFootPrint()
{
    return std::weak_ptr<CollisionFootPrint>(mCollisionFootprint);
}

//////////////////////////////////////////////////////////////////////////

void RectangularGeomClass::step(const ptSharedPtr<stPose>& pose)
{

    mPose = pose;
    if(mCollisionFootprint==nullptr)
        std::cerr << "[ Rectangular Geometric Model ]: Collision Footprint is nullptr, Please set the pointer.";
    mCollisionFootprint->step(mPose);
    calcVertices();
    printVertices();
}

//////////////////////////////////////////////////////////////////////////

void RectangularGeomClass::calcVertices()
{
    // front left
    double xFl = mPose->xCoord + (mLength/2.0)*cos(mPose->yaw) - (mWidth/2.0)*sin(mPose->yaw); 
    double yFl = mPose->yCoord + (mWidth/2.0)*cos(mPose->yaw) + (mLength/2.0)*sin(mPose->yaw);
    mVertices->frontLeft.setCoord(xFl,yFl,0.0,mPose->yaw);
    // front right
    double xFr = mPose->xCoord + (mLength/2.0)*cos(mPose->yaw) + (mWidth/2.0)*sin(mPose->yaw); 
    double yFr = mPose->yCoord - (mWidth/2.0)*cos(mPose->yaw) + (mLength/2.0)*sin(mPose->yaw);
    mVertices->frontRight.setCoord(xFr,yFr,0.0,mPose->yaw);
    // rear right
    double xRr = mPose->xCoord - (mLength/2.0)*cos(mPose->yaw) + (mWidth/2.0)*sin(mPose->yaw); 
    double yRr = mPose->yCoord - (mWidth/2.0)*cos(mPose->yaw) - (mLength/2.0)*sin(mPose->yaw);
    mVertices->rearRight.setCoord(xRr,yRr,0.0,mPose->yaw);
    // rear left
    double xRl = mPose->xCoord - (mLength/2.0)*cos(mPose->yaw) - (mWidth/2.0)*sin(mPose->yaw); 
    double yRl = mPose->yCoord + (mWidth/2.0)*cos(mPose->yaw) - (mLength/2.0)*sin(mPose->yaw);
    mVertices->rearLeft.setCoord(xRl,yRl,0.0,mPose->yaw);
     
}

//////////////////////////////////////////////////////////////////////////

const std::shared_ptr<stVertices> RectangularGeomClass::getVertices() const
{
    return mVertices;
}

//////////////////////////////////////////////////////////////////////////

void RectangularGeomClass::printVertices() const 
{
    auto const& fl = mVertices->frontLeft;
    auto const& fr = mVertices->frontRight;
    auto const& rl = mVertices->rearLeft;
    auto const& rr = mVertices->rearRight;

    std::cerr << "front left :  "
              << "x " << fl.xCoord << ", y " << fl.yCoord << ", yaw " << fl.yaw << '\n';

    std::cerr << "front right: "
              << "x " << fr.xCoord << ", y " << fr.yCoord << ", yaw " << fr.yaw << '\n';

    std::cerr << "rear left  : "
              << "x " << rl.xCoord << ", y " << rl.yCoord << ", yaw " << rl.yaw << '\n';

    std::cerr << "rear right : "
              << "x " << rr.xCoord << ", y " << rr.yCoord << ", yaw " << rr.yaw << '\n';
}
