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
#include "dummy_lateral_controller/dummy_lateral_controller.hpp"
#include "dummy_longitudinal_controller/dummy_longitudinal_controller.hpp"
#include "mppi_controller/mppi_controller.hpp"
#include "project_utils/common_utils.hpp"
#include "regulated_pure_pursuit/regulated_pure_pursuit.hpp"
#include "trajectory_follower_base/trajectory_follower_base_collection.h"

#include <algorithm>
#include <stdexcept>
#include <string>
enum class LateralControllerMode {
	INVALID = 0,
	DUMMY_LATERAL,
	PURE_PURSUIT,
	MPC,
};

enum class LongitudinalControllerMode { INVALID = 0, DUMMY_LONGITUDINAL, PID, MPC };

enum class HybridControllerMode { INVALID = 0, MPC, MPCC, REGULATED_PURE_PURSUIT, MPPI };

//////////////////////////////////////////////////////////////////////////

inline LateralControllerMode getLateralControllerMode(const std::string & mode)
{
	if (isStringEqual(mode, "mpc")) {
		return LateralControllerMode::MPC;
	}
	if (isStringEqual(mode, "pure_pursuit")) {
		return LateralControllerMode::PURE_PURSUIT;
	}
	if (isStringEqual(mode, "dummy_lateral")) {
		return LateralControllerMode::DUMMY_LATERAL;
	}

	return LateralControllerMode::INVALID;
}

//////////////////////////////////////////////////////////////////////////

inline std::unique_ptr<trajectory_follower::LateralControllerBase> getLateralController(
    const std::string & mode, rclcpp::Node & node)
{
	const auto lateralControllerMode = getLateralControllerMode(mode);
	switch (lateralControllerMode) {
		case LateralControllerMode::MPC: {
			// mLateralController =
			// std::make_shared<mpc_lateral_controller::MpcLateralController>(*this);
			break;
		}
		case LateralControllerMode::PURE_PURSUIT: {
			// mLateralController =  std::make_shared<pure_pursuit::PurePursuitController>(*this);
			// mLogger.info("[LateralController] pure-pursuit as lateral controller is selected");
			break;
		}
		case LateralControllerMode::DUMMY_LATERAL: {
			auto mLateralController =
			    std::make_unique<dummy_lateral_controller::DummyLateralController>(node);
			return mLateralController;
		}
		default: {
			return nullptr;
		}
	}
	return nullptr;
}

//////////////////////////////////////////////////////////////////////////

inline LongitudinalControllerMode getLongitudinalControllerMode(const std::string & mode)
{
	if (isStringEqual(mode, "pid")) {
		return LongitudinalControllerMode::PID;
	}
	if (isStringEqual(mode, "dummy_longitudinal")) {
		return LongitudinalControllerMode::DUMMY_LONGITUDINAL;
	}

	return LongitudinalControllerMode::INVALID;
}

//////////////////////////////////////////////////////////////////////////

inline std::unique_ptr<trajectory_follower::LongitudinalControllerBase> getLongitudinalController(
    const std::string & mode, rclcpp::Node & node)
{
	const auto longitudinalControllerMode = getLongitudinalControllerMode(mode);
	switch (longitudinalControllerMode) {
		case LongitudinalControllerMode::PID: {
			// mLongitudinalController =
			// std::make_shared<pid_longitudinal_controller::PIDLongController>(*this);
			// mLogger.info("[Longitudinal Controller] pid as longitudinal controller is selected");
			break;
		}
		case LongitudinalControllerMode::DUMMY_LONGITUDINAL: {
			auto mLongitudinalController =
			    std::make_unique<dummy_longitudinal_controller::DummyLongitudinalController>(node);
			return mLongitudinalController;
		}
		default: {
			return nullptr;
		}
	}
	return nullptr;
}

//////////////////////////////////////////////////////////////////////////

inline HybridControllerMode getHybridControllerMode(const std::string & mode)
{
	if (isStringEqual(mode, "mpc")) {
		return HybridControllerMode::MPC;
	} else if (isStringEqual(mode, "mpcc")) {
		return HybridControllerMode::MPCC;
	} else if (isStringEqual(mode, "regulated_pp")) {
		return HybridControllerMode::REGULATED_PURE_PURSUIT;
	} else if (isStringEqual(mode, "mppi_controller")) {
		return HybridControllerMode::MPPI;
	} else {
		return HybridControllerMode::INVALID;
	}
}

//////////////////////////////////////////////////////////////////////////

inline std::unique_ptr<trajectory_follower::HybridControllerBase> getHybridController(
    const std::string & mode, rclcpp::Node & node)
{
	const auto longitudinalControllerMode = getHybridControllerMode(mode);
	switch (longitudinalControllerMode) {
		case HybridControllerMode::MPC: {
			// mLongitudinalController =
			// std::make_shared<pid_longitudinal_controller::PIDLongController>(*this);
			// mLogger.info("[Longitudinal Controller] pid as longitudinal controller is selected");
			break;
		}
		case HybridControllerMode::MPCC: {
			// auto mHybridController = std::make_unique<mpcc_controller::MPCCController>(node);
			return nullptr;
		}
		case HybridControllerMode::REGULATED_PURE_PURSUIT: {
			auto hydridController = std::make_unique<regulatedpp_controller::RegulatedPP>(node);
			return hydridController;
		}
		case HybridControllerMode::MPPI: {
			auto hydridController = std::make_unique<controller::mppi_controller::MPPIController>(node);
			return hydridController;
		}
		default: {
			return nullptr;
		}
	}
	return nullptr;
}
