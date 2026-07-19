// Copyright 2026 Prajwal Thakur
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#pragma once
#include "trajectory_follower_base/controller_base.h"

namespace mpl::control::trajectory_follower
{
    using project_utils_msgs::msg::Lateral;
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
            ~LateralControllerBase() override=default;
            
            virtual bool isReady(const InputData& inputData)=0;
            virtual LateralOutput run(InputData const& inputData)=0;
            void sync(LongitudinalSyncData const & longitudinalSyncData);
        protected:
            LongitudinalSyncData mLongitudinalSyncData;
    };
}//namespace mpl::control::trajectory_follower
namespace trajectory_follower = mpl::control::trajectory_follower;