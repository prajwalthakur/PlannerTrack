#include "trajectory_follower_base/lateral_controller_base.h"
namespace mpl::control::trajectory_follower
{

    void LateralControllerBase::sync(LongitudinalSyncData const& longitudinalSyncData)
    {
        mLongitudinalSyncData = longitudinalSyncData;
    }
}//namespace mpl::control::trajectory_follower