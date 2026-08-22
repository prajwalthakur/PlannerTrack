#pragma once
#include "mppi_controller/critics/critic_function.hpp"
#include "mppi_controller/models/trajectory.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/models/path.hpp"
namespace controller::mppi_controller::critic
{
    /**
    * @brief Penalizes each rollout's lateral distance from the reference
    * path near the furthest reached path point. High settings follow the
    * path more precisely, but also make it difficult (or impossible) to
    * deviate in the presence of dynamic obstacles. Skipped when too large
    * a fraction of the local path is occupied (`mMaxPathOccupancyRatio`),
    * so it doesn't fight the obstacle-avoidance critics.
    */
    class PathAlignCritic: public CriticFunction
    {
        public:
            PathAlignCritic()=default;
            ~PathAlignCritic() override =default;
            void initialize() override;
            /// \brief Accumulate a path-lateral-deviation penalty into `data.costs`.
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

