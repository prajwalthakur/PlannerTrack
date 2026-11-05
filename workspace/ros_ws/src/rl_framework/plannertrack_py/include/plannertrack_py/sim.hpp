/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */

#pragma once

#include "motion_model_base/agent_model.hpp"
#include "motion_model_base/vehicle_model_factory.hpp"
#include "motion_model_base/world_snapshot.hpp"
#include "project_utils/logger.hpp"

#include <Eigen/Dense>

#include <memory>
#include <string>

namespace plannertrack_py
{

/**
 * \brief Single-agent, ROS-free driver around one \ref AgentModel, for use
 * from Python.
 *
 * Builds the agent through \ref VehicleModelFactory exactly as \c agent_sim
 * does (same plugins, same YAML), but without a node: nothing is published
 * and time only advances when \ref step is called, so it runs as fast as the
 * dynamics allow and is fully deterministic.
 *
 * Member order matters: \c mFactory owns the pluginlib \c ClassLoaders and
 * must outlive the plugin instances held by \c mAgent, so it is declared
 * first (members are destroyed in reverse declaration order).
 */
class Sim
{
  public:
	/**
	 * \param simYaml   Contents of a \c sim.yaml (needs \c simTimeStep).
	 * \param agentYaml One agent's node from \c agents.yaml (the mapping under
	 *                  e.g. \c agent_1: \c dynamics_plugin, \c dynamics_params,
	 *                  \c geometry_plugin, ...).
	 */
	Sim(const std::string & simYaml, const std::string & agentYaml);
	~Sim() = default;

	Sim(const Sim &) = delete;
	Sim & operator=(const Sim &) = delete;

	/// \brief Return to the initial state configured in \c dynamics_params.
	void reset();

	/// \brief Overwrite the state; size must equal \ref nx.
	void setState(const Eigen::VectorXd & state);

	/// \brief Copy of the current state vector.
	Eigen::VectorXd getState() const;

	/**
	 * \brief Hold input \p u for \p n simulator steps of \ref dt each and
	 * return the resulting state. Size of \p u must equal \ref nu.
	 */
	Eigen::VectorXd step(const Eigen::VectorXd & u, int n = 1);

	/**
	 * \brief Continuous-time derivative dx/dt = f(x, u) of the loaded dynamics
	 * plugin (DynamicModel::xdot), at any state/input. Does not change the
	 * simulator's state. Sizes must equal \ref nx and \ref nu.
	 */
	Eigen::VectorXd xdot(const Eigen::VectorXd & state, const Eigen::VectorXd & u) const;

	/// \brief Simulator step [s] (\c simTimeStep).
	double dt() const { return mDt; }

	/// \brief State dimension of the loaded dynamics plugin.
	int nx() const { return mNx; }

	/// \brief Input dimension of the loaded dynamics plugin.
	int nu() const { return mNu; }

	/// \brief Name of the loaded dynamics plugin (e.g. \c SingleTrackDynStateModel).
	const std::string & dynamicsPlugin() const { return mDynamicsPlugin; }

  private:
	mpl::rclcpp_utils::Logger mLogger;
	std::unique_ptr<VehicleModelFactory> mFactory;  // must outlive mAgent
	ptSharedPtr<AgentModel> mAgent;
	WorldSnapshot mWorld;  // empty: single agent, sensors are not stepped

	double mDt{0.0};
	int mNx{0};
	int mNu{0};
	std::string mDynamicsPlugin;
};

}  // namespace plannertrack_py
