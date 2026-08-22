#pragma once
#include "mppi_controller/critics/critic_function.hpp"
#include "mppi_controller/models/trajectory.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/models/path.hpp"
namespace controller::mppi_controller::critic
{
    /**
    * @brief Penalizes negative (reverse) longitudinal velocity, so
    * rollouts are biased toward driving forward rather than backward when
    * both are otherwise similarly costed.
    */
    class PreferForwardCritic: public CriticFunction
    {
        public:
            PreferForwardCritic()=default;
            ~PreferForwardCritic() override =default;
            void initialize() override;
            /// \brief Accumulate a time-integrated reverse-velocity penalty into `data.costs`.
            void score(CriticData& data) override;
        protected:
            unsigned int mPower{0};
            float mWeight{0};
            float mThresholdToConsider{0};
    };

}//namespace controller::mppi_controller

