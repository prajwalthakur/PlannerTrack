#pragma once
#include "project_utils/msg/lateral.hpp"
#include "project_utils/msg/longitudinal.hpp"
#include "project_utils/msg/float64_stamped.hpp"
#include <vector>
namespace mpl::control::trajectory_follower
{
    using  project_utils::msg::Lateral;
    using  project_utils::msg::Longitudinal;
    using  project_utils::msg::Float64Stamped;

    struct LateralHorizon
    {
        // time step to send control commands.
        double mTimeStepMs;
        // Lateral Control commands.
        std::vector<Lateral> mControls;
    };

    struct LongitudinalHorizon
    {
        // time step to send control commands.
        double mTimeStepMs;
        // Longitudinal Control commands.
        std::vector<Longitudinal> mControls;
    };

}// namespace mpl::control::trajectory_follower
