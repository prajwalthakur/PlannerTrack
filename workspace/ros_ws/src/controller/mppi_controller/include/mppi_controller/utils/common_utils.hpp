#pragma once

namespace controller::mppi_controller::utils
{

    /**
    * @brief Clamps the input between the given lower and upper bounds.
    * @param lower_bound Lower bound.
    * @param upper_bound Upper bound.
    * @return Clamped output.
    */
    inline float clamp(
    const float lower_bound, const float upper_bound, const float input)
    {
    return std::min(upper_bound, std::max(input, lower_bound));
    }

}//namespace controller::mppi_controller::utils

namespace mppi_utils = controller::mppi_controller::utils;