#pragma once
#include "mppi_controller/critics/critic_function.hpp"
#include "mppi_controller/models/trajectory.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/models/path.hpp"
namespace controller::mppi_controller::critic
{
    /**
    * @class mppi::critics::ConstraintCritic
    * @brief Critic objective function for following the path approximately
    * To allow for deviation from path in case of dynamic obstacles. Path follow critic
    * will promote the trajectories to follow short term goals 
    */
    class PreferForwardCritic: public CriticFunction
    {
        public:
            PreferForwardCritic()=default;
            ~PreferForwardCritic() override =default;
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
    };

}//namespace controller::mppi_controller

