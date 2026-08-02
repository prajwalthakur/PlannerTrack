#pragma once
#include "mppi_controller/critics/critic_function.hpp"
#include "mppi_controller/models/trajectory.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/models/path.hpp"
#include "mppi_controller/models/ackermann/ackermann.hpp"

namespace controller::mppi_controller::critic
{
    /**
    * @class mppi::critics::ConstraintCritic
    * @brief Critic objective function for following the path approximately
    * To allow for deviation from path in case of dynamic obstacles. Path follow critic
    * will promote the trajectories to follow short term goals 
    */
    class ConstraintCritic: public CriticFunction
    {
        public:
            ConstraintCritic()=default;
            ~ConstraintCritic() override =default;
            
            void initialize() override;
            /**
            * @brief Evaluate cost related to trajectories path alignment
            *
            * @param costs [out] add reference cost values to this tensor
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

