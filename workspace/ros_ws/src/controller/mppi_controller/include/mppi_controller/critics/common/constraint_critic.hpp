#pragma once
#include "mppi_controller/critics/critic_function.hpp"
#include "mppi_controller/models/trajectory.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/models/path.hpp"
#include "mppi_controller/models/ackermann/ackermann.hpp"

namespace controller::mppi_controller::critic
{
    /**
    * @brief Penalizes rollouts that violate this vehicle's kinodynamic
    * constraints: longitudinal velocity outside `[mMinVelX, mMaxVelX]`, and
    * a turning radius (`|vx|/|wz|`) tighter than the model's minimum
    * turning radius.
    */
    class ConstraintCritic: public CriticFunction
    {
        public:
            ConstraintCritic()=default;
            ~ConstraintCritic() override =default;

            void initialize() override;
            /**
            * @brief Accumulate a time-integrated penalty for velocity-bound
            * and minimum-turning-radius violations over each rollout.
            *
            * @param data [in,out] critic data; violation cost is added to `data.costs`
            */
            void score(CriticData& data) override;

            // float getMaxVelConstraint() {return mMaxVel;}

            // float getMinVelConstraint() {return mMinVel;}
        protected:
            unsigned int mPower{0};
            float mWeight{0};
            float mMaxVelX{0}; 
            float mMinVelX{0};
            float mMaxVelY{0};
    };

}//namespace controller::mppi_controller

