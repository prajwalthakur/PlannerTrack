#pragma once
#include "trajectory_follower_base/controller_base.h"

namespace mpl::control::trajectory_follower
{
    using project_utils::msg::Longitudinal;
    struct LongitudinalOutput
    {
        Longitudinal mControlCmd;
        LongitudinalHorizon mControlCmdHorizon;
        LongitudinalSyncData mSyncData;
    };

    class LongitudinalControllerBase : public ControllerBase
    {
        public:
            virtual bool isReady(const InputData& inputData)=0;
            virtual LongitudinalOutput run(InputData const& inputData)=0;
            void sync(LateralSyncData const & lateralSyncData);
            // NOTE: This reset function should be called when the trajectory is replanned by changing ego
            // pose or goal pose.
            void reset();
        protected:
            LateralSyncData mLateralSyncData;
    };
}//namespace mpl::control::trajectory_follower