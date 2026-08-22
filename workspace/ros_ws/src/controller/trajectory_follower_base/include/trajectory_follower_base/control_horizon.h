// Copyright 2026 Prajwal Thakur
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#pragma once
#include "project_utils_msgs/msg/float64_stamped.hpp"
#include "project_utils_msgs/msg/lateral.hpp"
#include "project_utils_msgs/msg/longitudinal.hpp"

#include <vector>

/** \file
 * \brief Multi-step-ahead control command sequences, alongside each
 * controller's single-step output -- for consumers that want a short
 * horizon of upcoming commands rather than just the next one.
 */
namespace mpl::control::trajectory_follower
{
using project_utils_msgs::msg::Float64Stamped;
using project_utils_msgs::msg::Lateral;
using project_utils_msgs::msg::Longitudinal;

/// \brief A sequence of upcoming lateral control commands, spaced \ref mTimeStepMs apart.
struct LateralHorizon
{
	double mTimeStepMs;
	std::vector<Lateral> mControls;
};

/// \brief A sequence of upcoming longitudinal control commands, spaced \ref mTimeStepMs apart.
struct LongitudinalHorizon
{
	double mTimeStepMs;
	std::vector<Longitudinal> mControls;
};

/// \brief Combined lateral + longitudinal command horizon.
struct ControlHorizon
{
	LateralHorizon mLateralHorizon;
	LongitudinalHorizon mLongitudinalHorizon;
};

}  // namespace mpl::control::trajectory_follower
