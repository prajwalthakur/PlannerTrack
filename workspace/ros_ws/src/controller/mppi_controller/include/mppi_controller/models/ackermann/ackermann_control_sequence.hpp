#pragma once
#include "mppi_controller/models/control_sequence.hpp"
namespace controller::mppi_controller::models
{
    /// \brief \ref Control for \ref AckermannModel -- a single-timestep `{vx, wz}` command.
    struct AckermannControl : public Control
    {
        AckermannControl() = default;
        AckermannControl(float vx_, float wz_) : vx(vx_), wz(wz_) {}
        ~AckermannControl() override = default;
        float vx;
        float wz;
    };

    /// \brief \ref ControlSequence for \ref AckermannModel -- `{vx, wz}` over the optimization horizon.
    struct AckermannControlSequence : public ControlSequence
    {


        ~AckermannControlSequence() override = default;

        /// \brief Resize \ref vx / \ref wz to \p time_steps and zero them.
        void reset(unsigned int time_steps) override;
        mppi_mt::ArrayX vx;
        mppi_mt::ArrayX wz;
    };

} // namespadce constraints