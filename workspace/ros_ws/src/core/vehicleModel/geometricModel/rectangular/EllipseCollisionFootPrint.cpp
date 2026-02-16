
#include "EllipseCollisionFootPrint.h"

//////////////////////////////////////////////////////////////////////////

EllipseCollisionFootPrint::EllipseCollisionFootPrint(const double majorAxisLength, const double minorAxisLength)
    : mMajorAxisLength(majorAxisLength),
      mMinorAxisLength(minorAxisLength),
      mSemiMajorAxisLength(majorAxisLength/2.0),
      mSemiMinorAxisLength(minorAxisLength/2.0)
      {
        mPose  = std::make_shared<stPose>();
      }

//////////////////////////////////////////////////////////////////////////

void EllipseCollisionFootPrint::step(const ptSharedPtr<stPose>& pose)
{
    mPose = pose;
}

//////////////////////////////////////////////////////////////////////////

bool EllipseCollisionFootPrint::contains(const std::shared_ptr<stPose>& pose) const {
    // For an ellipse E and the corresponding ellipse matrix A,
    // x is inside in E if (x-x0)^T * A * (x-x0) <= 1, where x0 is the center of E.

    auto offset = pose->toEigenVector(pose) - pose->toEigenVector(mPose) ; // x-x0

    return offset.transpose() * getEllipseMatrix() * offset <= 1.f;
}

//////////////////////////////////////////////////////////////////////////

Eigen::Matrix2f EllipseCollisionFootPrint::getEllipseMatrix() const
{
    Eigen::Matrix2f D;
    D << 1.0f / (mSemiMajorAxisLength * mSemiMajorAxisLength), 0,
         0, 1.0f / (mSemiMinorAxisLength * mSemiMinorAxisLength);

    float c = std::cos(mPose->yaw);
    float s = std::sin(mPose->yaw);

    Eigen::Matrix2f R;
    R << c, -s,
         s,  c;

    return R * D * R.transpose();
}

//////////////////////////////////////////////////////////////////////////

Eigen::Vector2f EllipseCollisionFootPrint::getCenter()
{
    Eigen::Vector2f vec;
    vec << mPose->xCoord , mPose->yCoord;
    return vec;
}

//////////////////////////////////////////////////////////////////////////

std::pair<double,bool> EllipseCollisionFootPrint::detectCollision(const std::shared_ptr<CollisionFootPrint> object1, const std::shared_ptr<CollisionFootPrint> object2)
{
    auto obj1FootPrint = std::dynamic_pointer_cast<EllipseCollisionFootPrint>(object1);
    auto obj2FootPrint = std::dynamic_pointer_cast<EllipseCollisionFootPrint>(object2);

    if(!obj1FootPrint || !obj2FootPrint)
    {
        std::cerr << "ERROR: Invalid footprint type";
        return std::make_pair(-1.0,false);
    }

    Eigen::Vector2f a = obj1FootPrint->getCenter();
    Eigen::Vector2f b = obj2FootPrint->getCenter();

    Eigen::Matrix2f A = obj1FootPrint->getEllipseMatrix();
    Eigen::Matrix2f B = obj2FootPrint->getEllipseMatrix();

    Eigen::Matrix2f Ainv = A.inverse();
    Eigen::Matrix2f Binv = B.inverse();

    Eigen::Vector2f d = b - a;
    auto k_function = [&](float s)
    {
        // avoid division by zero
        s = std::clamp(s, 1e-4f, 1.0f - 1e-4f);

        Eigen::Matrix2f M =
            (1.0f / (1.0f - s)) * Ainv +
            (1.0f / s) * Binv;

        Eigen::Matrix2f Minv = M.inverse();

        float quad = d.transpose() * Minv * d;
        return 1.0f - quad;
    };

    auto [_, min_k] = optimize(0.5f, k_function);
    return std::make_pair(min_k,min_k > 0.f);
}