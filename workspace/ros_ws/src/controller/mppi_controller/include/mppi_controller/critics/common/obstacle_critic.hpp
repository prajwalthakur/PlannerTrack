#pragma once
#include "mppi_controller/critics/critic_function.hpp"
#include "mppi_controller/models/trajectory.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/models/path.hpp"
namespace controller::mppi_controller::critic
{
    /**
    * @brief Intended as an obstacle-proximity critic (name/member layout
    * mirrors \ref PathAlignCritic), but not currently wired up:
    * `obstacle_critic.cpp` is not listed in `CMakeLists.txt`'s
    * `LIB_SOURCES`, and `"ObstacleCritic"` is not one of the names \ref
    * mppi_critics_utils::getCritic "critics_utils::getCritic" recognizes,
    * so this critic is never loaded. \ref CostCritic is what actually
    * penalizes costmap obstacles today.
    */
    class ObstacleCritic: public CriticFunction
    {
        public:
            ObstacleCritic()=default;
            ~ObstacleCritic() override =default;
            void initialize() override;
            /**
            * @brief Evaluate cost related to trajectories path alignment
            *
            * @param costs [out] add reference cost values to this tensor
            */
            void score(CriticData& data) override;
        protected:
            unsigned int mPower{0};
            float mWeight{0};
            float mThresholdToConsider{0};
            size_t mOffsetFromFurthest{0};
            int mTrajectoriesPointStep{0};
            float mMaxPathOccupancyRatio{0};
            bool mUsePathOrientation{false};
    };

}//namespace controller::mppi_controller

