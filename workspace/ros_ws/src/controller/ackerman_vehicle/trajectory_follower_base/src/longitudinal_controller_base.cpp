
#include "trajectory_follower_base/longitudinal_controller_base.h"

namespace mpl::control::trajectory_follower
{

    void LongitudinalControllerBase::sync(LateralSyncData const& lateralSyncData)
    {
        mLateralSyncData = lateralSyncData;
    }
    void LongitudinalControllerBase::reset()
    {
        mLateralSyncData.mIsSteerConverged = false;
    }
}//namespace mpl::control::trajectory_follower