#pragma once
#include "mppi_controller/critics/critic_function.hpp"
#include "mppi_controller/models/trajectory.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/models/path.hpp"
namespace controller::mppi_controller::critic
{
    /**
    * @brief Penalizes each rollout for falling short of the furthest
    * collision-free ("valid") point reached on the local path, promoting
    * progress toward that short-term goal rather than exact path alignment
    * -- this is what lets rollouts deviate around dynamic obstacles while
    * still making forward progress.
    */
    class PathFollowCritic: public CriticFunction
    {
        public:
            PathFollowCritic()=default;
            ~PathFollowCritic() override =default;
            void initialize() override;
            /// \brief Accumulate a distance-to-short-term-goal penalty into `data.costs`.
            void score(CriticData& data) override;
        protected:
            unsigned int mPower{0};
            float mWeight{0};
            float mThresholdToConsider{0};
            size_t mOffsetFromFurthest{0};
    };

}//namespace controller::mppi_controller

