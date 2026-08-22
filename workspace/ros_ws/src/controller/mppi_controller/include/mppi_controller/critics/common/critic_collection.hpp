#pragma once
/**
 * \file
 * \brief Umbrella header pulling in every critic \ref
 * mppi_critics_utils::getCritic "critics_utils::getCritic" can construct
 * by name. (`obstacle_critic.hpp` is deliberately not included here --
 * see that critic's doc comment.)
 */
#include "mppi_controller/critics/common/goal_critic.hpp"
#include "mppi_controller/critics/common/goal_angle_critic.hpp"
#include "mppi_controller/critics/common/path_align_critic.hpp"
#include "mppi_controller/critics/common/path_follow_critic.hpp"
#include "mppi_controller/critics/common/prefer_forward_critic.hpp"
#include "mppi_controller/critics/common/path_angle_critic.hpp"
#include "mppi_controller/critics/common/constraint_critic.hpp"
#include "mppi_controller/critics/common/twirling_critic.hpp"
#include "mppi_controller/critics/common/velocity_deadband_critic.hpp"
#include "mppi_controller/critics/common/cost_critic.hpp"
