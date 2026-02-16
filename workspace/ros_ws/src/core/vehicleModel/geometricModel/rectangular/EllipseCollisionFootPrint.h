#pragma once
#include "vehicleModel/VehicleModel.h"

//////////////////////////////////////////////////////////////////////////

class EllipseCollisionFootPrint : public CollisionFootPrint
{
    public:
        // Constructor
        EllipseCollisionFootPrint(const double majorAxisLength, const double minorAxisLength);
        // Destructor
        ~EllipseCollisionFootPrint()=default;
        // Get the ellipise collision foot print model.
        Eigen::Matrix2f getEllipseMatrix() const ;
        // Get the ceneter of the ellipse.
        Eigen::Vector2f getCenter();
        // Detect the collision between two ellipse shaped collision foot print model.
        std::pair<double,bool> detectCollision(const std::shared_ptr<CollisionFootPrint> object1, const std::shared_ptr<CollisionFootPrint> object2);
        // Check if the pose is within the ellipse.
        bool contains(const std::shared_ptr<stPose>& pose) const ;
        // Update collision foot print model
        void step(const ptSharedPtr<stPose>& pose) override;
    private:
        std::shared_ptr<stPose> mPose{nullptr};
        double mMajorAxisLength{0.0};
        double mMinorAxisLength{0.0};
        double mSemiMajorAxisLength{0.0};
        double mSemiMinorAxisLength{0.0};

};