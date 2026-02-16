#pragma once
#include "core/CoreCollection.h"

// Collision foot print model
class CollisionFootPrint
{
    public:
    CollisionFootPrint()=default;
    virtual ~CollisionFootPrint()=default;
    virtual void step(const ptSharedPtr<stPose>& pose)=0;
};

////////////////////////////////////////////////////////////////////////////////

// Geometric Model
// Collision Model is member of Geometric Model
class GeometricModel
{
    public:
        GeometricModel()=default;
        virtual ~GeometricModel()=default;  
        virtual void setCollisionFootPrint(const ptSharedPtr<CollisionFootPrint> collisionFootPrint)=0;
        virtual ptwkPtr<CollisionFootPrint> getCollisionFootPrint()=0;
        virtual void step(const ptSharedPtr<stPose>& pose)=0 ;
};