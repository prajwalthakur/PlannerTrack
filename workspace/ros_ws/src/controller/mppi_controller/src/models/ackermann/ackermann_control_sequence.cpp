#include "mppi_controller/models/ackermann/ackermann_control_sequence.hpp"



void  models::AckermannControlSequence::reset(unsigned int time_steps)
{
    vx.setZero(time_steps);
    wz.setZero(time_steps);
}