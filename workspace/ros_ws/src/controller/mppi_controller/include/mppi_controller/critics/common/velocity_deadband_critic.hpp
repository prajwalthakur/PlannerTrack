#pragma once
#include "mppi_controller/critics/critic_function.hpp"
#include "mppi_controller/models/trajectory.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/models/path.hpp"
#include "mppi_controller/models/ackermann/ackermann.hpp"
namespace controller::mppi_controller::critic
{
    /**
    * @brief Penalizes commanding a linear/angular velocity below
    * `mDeadbandVelocities`, i.e. inside a "dead zone" too small to
    * physically move the vehicle (e.g. under motor stiction).
    */
    class VelocityDeadbandCritic : public CriticFunction
    {
        public:
            VelocityDeadbandCritic()=default;
            ~VelocityDeadbandCritic() override =default;
            void initialize() override;
            /// \brief Accumulate a time-integrated deadband-violation penalty into `data.costs`.
            void score(CriticData& data) override;
        protected:
            unsigned int mPower{0};
            float mWeight{0};
            std::vector<float> mDeadbandVelocities{0.0f, 0.0f, 0.0f};
    };

}//namespace controller::mppi_controller

