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
class ControllerBase
{
  public:
	ControllerBase() = default;
	virtual ~ControllerBase() = default;
	virtual bool isReady(const InputData & inputData) = 0;

	// creates agent model for the controller , notice
	// the name of the agent is "controller_agent" + agentNumber, to distinguish
	// against sim agents
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
