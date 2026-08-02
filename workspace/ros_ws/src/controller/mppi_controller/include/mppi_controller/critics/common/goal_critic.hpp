#pragma once
#include "mppi_controller/critics/critic_function.hpp"
#include "mppi_controller/models/trajectory.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/models/path.hpp"
namespace controller::mppi_controller::critic
{

    class GoalCritic: public CriticFunction
    {
        public:
            GoalCritic()=default;
            ~GoalCritic() override =default;
            void initialize() override;
            void score(CriticData& data) override;
        protected:
            bool mPathFollow{true};
            unsigned int mPower{0};
            float mWeight{0};
            float mThresholdToConsider{0};
    };

}//namespace controller::mppi_controller