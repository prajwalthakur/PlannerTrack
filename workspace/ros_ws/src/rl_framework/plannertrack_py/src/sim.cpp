/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */

#include "plannertrack_py/sim.hpp"

#include "project_utils/unique_id.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

/** \file
 * \brief \ref plannertrack_py::Sim implementation.
 */

namespace plannertrack_py
{

namespace
{

YAML::Node requireKey(const YAML::Node & node, const std::string & key, const std::string & where)
{
	if (!node[key]) {
		throw std::invalid_argument("plannertrack_py.Sim: missing '" + key + "' in " + where);
	}
	return node[key];
}

void checkSize(const Eigen::VectorXd & v, int expected, const std::string & what)
{
	if (v.size() != expected) {
		throw std::invalid_argument("plannertrack_py.Sim: " + what + " has size " +
		                            std::to_string(v.size()) + ", expected " +
		                            std::to_string(expected));
	}
}

}  // namespace

//////////////////////////////////////////////////////////////////////////

Sim::Sim(const std::string & simYaml, const std::string & agentYaml)
    : mLogger("plannertrack_py")
{
	YAML::Node simConfig = YAML::Load(simYaml);
	const YAML::Node agentConfig = YAML::Load(agentYaml);

	mDt = requireKey(simConfig, "simTimeStep", "sim config").as<double>();
	mDynamicsPlugin = requireKey(agentConfig, "dynamics_plugin", "agent config").as<std::string>();
	const YAML::Node dynParams = requireKey(agentConfig, "dynamics_params", "agent config");
	mNu = requireKey(dynParams, "numControlInputs", "dynamics_params").as<int>();

	mFactory = std::make_unique<VehicleModelFactory>(mLogger);
	mAgent = mFactory->create(simConfig, agentConfig, UniqueId("agent", 1), nullptr, "", false);
	if (!mAgent) {
		throw std::runtime_error("plannertrack_py.Sim: VehicleModelFactory returned no agent");
	}
	mNx = static_cast<int>(mAgent->getState().size());
}

//////////////////////////////////////////////////////////////////////////

void Sim::reset()
{
	mAgent->resetToInitialState();
	// resetToInitialState() only resets the dynamics; resync geometry/footprint.
	mAgent->setState(mAgent->getState());
}

//////////////////////////////////////////////////////////////////////////

void Sim::setState(const Eigen::VectorXd & state)
{
	checkSize(state, mNx, "state");
	mAgent->setState(state);
}

//////////////////////////////////////////////////////////////////////////

Eigen::VectorXd Sim::getState() const
{
	return mAgent->getState();
}

//////////////////////////////////////////////////////////////////////////

Eigen::VectorXd Sim::step(const Eigen::VectorXd & u, int n)
{
	checkSize(u, mNu, "input");
	if (n < 1) {
		throw std::invalid_argument("plannertrack_py.Sim: n must be >= 1");
	}
	mAgent->updateCommandedControl(u);
	for (int i = 0; i < n; ++i) {
		mAgent->step(mWorld, /*stepSensorModel=*/false);
	}
	return mAgent->getState();
}

//////////////////////////////////////////////////////////////////////////

Eigen::VectorXd Sim::xdot(const Eigen::VectorXd & state, const Eigen::VectorXd & u) const
{
	checkSize(state, mNx, "state");
	checkSize(u, mNu, "input");
	return mAgent->dynamicModel().xdot(state, u);
}

}  // namespace plannertrack_py
