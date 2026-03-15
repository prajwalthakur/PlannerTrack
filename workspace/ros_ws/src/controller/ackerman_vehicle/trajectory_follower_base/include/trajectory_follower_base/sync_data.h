#pragma once

namespace mpl::control::trajectory_follower
{

    struct LateralSyncData
    {
        bool mIsSteerConverged{false};

    };
    struct LongitudinalSyncData
    {
        // Not used
        //bool mIsVelocityConverged{false};
    };
}//namespace mpl::control::trajectory_follower