#pragma once
#include "trajectory_follower_base/controller_base.h"

namespace mpl::control::trajectory_follower
{
    using project_utils::msg::Lateral;
    struct LateralOutput
    {
        Lateral mControlCmd;
        LateralHorizon mControlCmdHorizon;
        LateralSyncData mSyncData;
    };

    class LateralControllerBase: public ControllerBase
    {
        public:
            LateralControllerBase()=default;
            ~LateralControllerBase() override;
            
            virtual bool isReady(const InputData& inputData)=0;
            virtual LateralOutput run(InputData const& inputData)=0;
            void sync(LongitudinalSyncData const & longitudinalSyncData);
        protected:
            LongitudinalSyncData mLongitudinalSyncData;
    };
}//namespace mpl::control::trajectory_follower