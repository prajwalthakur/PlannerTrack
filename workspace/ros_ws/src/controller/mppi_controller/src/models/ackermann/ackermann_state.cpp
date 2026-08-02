#include "mppi_controller/models/ackermann/ackermann_state.hpp"



void model::AckermannState::reset(unsigned int batch_size, unsigned int time_steps)
{
    vx.setZero(batch_size, time_steps);
    
    wz.setZero(batch_size, time_steps);

    cvx.setZero(batch_size, time_steps);
    
    cwz.setZero(batch_size, time_steps);
}