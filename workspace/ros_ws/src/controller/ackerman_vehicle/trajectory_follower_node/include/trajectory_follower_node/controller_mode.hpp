#pragma once

#include <string>
#include <algorithm>
#include <stdexcept>

enum class LateralControllerMode
{
    INVALID = 0,
    DUMMY_LATERAL,
    PURE_PURSUIT,
    MPC,
    MPCC
};

enum class LongitudinalControllerMode
{
    INVALID = 0,
    DUMMY_LONGITUDINAL,
    PID,
    MPCC
};

inline LateralControllerMode getLateralControllerMode(std::string mode)
{
    std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);

    if (mode == "mpc") return LateralControllerMode::MPC;
    if (mode == "pure_pursuit") return LateralControllerMode::PURE_PURSUIT;
    if (mode == "mpcc") return LateralControllerMode::MPCC;
    if (mode == "dummy_lateral") return LateralControllerMode::DUMMY_LATERAL;

    return LateralControllerMode::INVALID;
}

inline LongitudinalControllerMode getLongitudinalControllerMode(std::string mode)
{
    std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);

    if (mode == "pid") return LongitudinalControllerMode::PID;
    if (mode == "mpcc") return LongitudinalControllerMode::MPCC;
    if (mode == "dummy_longitudinal") return LongitudinalControllerMode::DUMMY_LONGITUDINAL;

    return LongitudinalControllerMode::INVALID;
}