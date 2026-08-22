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

namespace mpl::control::trajectory_follower
{
using project_utils_msgs::msg::Longitudinal;
/// \brief One longitudinal controller cycle's output: the command itself, its horizon, and sync data for the lateral side.
struct LongitudinalOutput
{
	Longitudinal mControlCmd;
	LongitudinalHorizon mControlCmdHorizon;
	LongitudinalSyncData mSyncData;
};

/**
 * \brief Base for controllers that compute velocity/acceleration commands
 * from an \ref InputData snapshot (e.g. \c DummyLongitudinalController,
 * `pid_controller`).
 */
class LongitudinalControllerBase : public ControllerBase
{
  public:
	virtual bool isReady(const InputData & inputData) = 0;
	/// \brief Compute this cycle's velocity/acceleration command from \p inputData.
	virtual LongitudinalOutput run(InputData const & inputData) = 0;
	/// \brief Receive the lateral controller's sync data (e.g. steering-converged state) for this cycle.
	void sync(LateralSyncData const & lateralSyncData);
	/// \brief Reset internal controller state; call when the trajectory is replanned (ego/goal pose changed).
	void reset();

  protected:
	LateralSyncData mLateralSyncData;
};
}  // namespace mpl::control::trajectory_follower