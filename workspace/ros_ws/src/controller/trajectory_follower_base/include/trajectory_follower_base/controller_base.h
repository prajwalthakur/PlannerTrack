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
#include "motion_model_base/agent_model.hpp"
#include "motion_model_base/vehicle_model_factory.hpp"
#include "project_utils/logger.hpp"
#include "trajectory_follower_base/control_horizon.h"
#include "trajectory_follower_base/input_data.h"
#include "trajectory_follower_base/sync_data.h"

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace mpl::control::trajectory_follower
{
/**
 * \brief Common base for lateral/longitudinal/hybrid trajectory-follower
 * controllers: readiness check plus an optional internal \ref AgentModel
 * a controller can use for e.g. simulating/predicting its own vehicle.
 */
class ControllerBase
{
  public:
	ControllerBase() = default;
	virtual ~ControllerBase() = default;
	/// \brief Whether the controller has enough data/state to safely \c run this cycle.
	virtual bool isReady(const InputData & inputData) = 0;

	/**
	 * \brief Lazily create this controller's own \ref AgentModel from
	 * YAML config, e.g. for internal simulation/prediction.
	 *
	 * The agent is named `"controller_agent" + agentNumber` to
	 * distinguish it from `agent_sim`'s simulated agents.
	 * \return Whether the agent was constructed successfully.
	 */
	bool createAgent(YAML::Node simConfig, const YAML::Node & agentConfig, int agentNumber)
	{
		if (!mVehicleFactory) {
			mVehicleFactory = std::make_unique<VehicleModelFactory>(mLogger);
		}
		const UniqueId id = UniqueId("controller_agent", agentNumber);
		mAgent = mVehicleFactory->create(simConfig, agentConfig, id, nullptr, "", false);
		return mAgent != nullptr;
	}

  protected:
	mpl::rclcpp_utils::Logger mLogger;
	std::unique_ptr<VehicleModelFactory> mVehicleFactory;
	ptSharedPtr<AgentModel> mAgent;
};
}  // namespace mpl::control::trajectory_follower
