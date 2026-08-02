#pragma once
#include "mppi_controller/critics/critic_function.hpp"
#include "mppi_controller/models/trajectory.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/models/path.hpp"
#include "mppi_controller/models/ackermann/ackermann.hpp"
namespace controller::mppi_controller::critic
{
    /**
    * @class mppi::critics::VelocityDeadbandCritic
    * @brief Critic objective function for enforcing feasible constraints
    */
    class VelocityDeadbandCritic : public CriticFunction
    {
        public:
            VelocityDeadbandCritic()=default;
            ~VelocityDeadbandCritic() override =default;
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
            std::vector<float> mDeadbandVelocities{0.0f, 0.0f, 0.0f};
    };

}//namespace controller::mppi_controller

