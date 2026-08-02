#pragma once
#include "mppi_controller/critics/critic_function.hpp"
#include "mppi_controller/models/trajectory.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/models/path.hpp"
namespace controller::mppi_controller::critic
{
    /**
    * @class mppi::critics::ObstacleCritic
    * @brief Critic objective function for aligning to the path. Note:
    * High settings of this will follow the path more precisely, but also makes it
    * difficult (or impossible) to deviate in the presence of dynamic obstacles.
    * This is an important critic to tune and consider in tandem with Obstacle.
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

