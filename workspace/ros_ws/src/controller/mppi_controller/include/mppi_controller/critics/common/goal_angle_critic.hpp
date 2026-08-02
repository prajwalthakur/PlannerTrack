#pragma once
#include "mppi_controller/critics/critic_function.hpp"
#include "mppi_controller/models/trajectory.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/models/path.hpp"
namespace controller::mppi_controller::critic
{

    class GoalAngleCritic: public CriticFunction
    {
        public:
            GoalAngleCritic()=default;
            ~GoalAngleCritic() override =default;
            
            void initialize() override;
              /**
             * @brief Evaluate cost related to robot orientation at goal pose
             * (considered only if robot near last goal in current plan)
             *
             * @param costs [out] add goal angle cost values to this tensor
             */
            void score(CriticData& data) override;
        protected:
            unsigned int mPower{0};
            float mWeight{0};
            float mThresholdToConsider{0};
            bool mSymmetricYawTolerance{false};

    };

}//namespace controller::mppi_controller