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
#include "trajectory_follower_base/controller_base.h"

#include "project_utils_msgs/msg/control.hpp"

#include <string>
namespace mpl::control::trajectory_follower
{
using project_utils_msgs::msg::Control;
/// \brief Combined lateral+longitudinal output of a \ref HybridControllerBase controller.
struct HybridOutput
{
  Control mControlCmd;
  ControlHorizon mControlCmdHorizon;
};

/**
 * \brief Base for controllers that compute both steering and
 * velocity/acceleration commands together in a single \c run (as opposed
 * to the separate \ref LateralControllerBase / \ref LongitudinalControllerBase
 * split), e.g. for a coupled controller like `mppi_controller`.
 */
class HybridControllerBase : public ControllerBase
{
public:
  HybridControllerBase() = default;
  ~HybridControllerBase() override = default;
  virtual bool isReady(const InputData & inputData) = 0;
  /// \brief Compute this cycle's combined steering + velocity/acceleration command from \p inputData.
  virtual HybridOutput run(InputData const & inputData) = 0;

protected:
  std::string mName;
};
}  // namespace mpl::control::trajectory_follower
namespace trajectory_follower = mpl::control::trajectory_follower;