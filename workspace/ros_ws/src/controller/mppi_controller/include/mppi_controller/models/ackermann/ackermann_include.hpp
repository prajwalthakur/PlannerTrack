#pragma once

/**
 * \file
 * \brief Umbrella header pulling in every \ref AckermannModel building
 * block (state, trajectories, path, control sequence, constraints, noise
 * generator) in one `#include`.
 */
#include "mppi_controller/models/model.hpp"
#include "mppi_controller/models/ackermann/ackermann_constraints.hpp"
#include "mppi_controller/models/ackermann/ackermann_control_sequence.hpp"
#include "mppi_controller/models/ackermann/ackermann_noise_generator.hpp"
#include "mppi_controller/models/ackermann/ackermann_path.hpp"
#include "mppi_controller/models/ackermann/ackermann_state.hpp"
#include "mppi_controller/models/ackermann/ackermann_trajectory.hpp"