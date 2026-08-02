#pragma once
#include "mppi_controller/models/control_sequence.hpp"
namespace controller::mppi_controller::models
{
    struct AckermannControl : public Control
    {
        AckermannControl() = default;
        AckermannControl(float vx_, float wz_) : vx(vx_), wz(wz_) {}
        ~AckermannControl() override = default;
        float vx;
        float wz;
    };

    struct AckermannControlSequence : public ControlSequence
    {


        ~AckermannControlSequence() override = default;
        
        void reset(unsigned int time_steps) override;
        mppi_mt::ArrayX vx;
        mppi_mt::ArrayX wz;
    };

} // namespadce constraints