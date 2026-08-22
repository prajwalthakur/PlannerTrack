#pragma once
#include "mppi_controller/critics/critic_function.hpp"
#include "mppi_controller/models/trajectory.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/models/path.hpp"
namespace controller::mppi_controller::critic
{
    /**
    * @brief Penalizes yaw-rate magnitude, discouraging rollouts that spin
    * in place rather than making net positional progress.
    */
    class TwirlingCritic: public CriticFunction
    {
        public:
            TwirlingCritic()=default;
            ~TwirlingCritic() override =default;
            void initialize() override;
            /// \brief Accumulate a mean-|yaw-rate| penalty into `data.costs`.
            void score(CriticData& data) override;
        protected:
            unsigned int mPower{0};
            float mWeight{0};
            float mPositionXTol{0};
    };

}//namespace controller::mppi_controller

