// Copyright 2021 The Autoware Foundation
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
#include "mpl_osqp_interface/csc_matrix_conv.hpp"
#include "osqp/osqp.h"

#include <Eigen/Core>

#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <vector>
#include <rclcpp/rclcpp.hpp>
namespace mpl::osqp_interface
{
constexpr c_float INF = 1e30;

struct OSQPResult
{
	std::vector<double> primal_solution;
	std::vector<double> lagrange_multipliers;
	int polish_status;
	int solution_status;
	int iteration_status;
	int exit_flag;
};

class OSQPInterface
{
  private:
	std::unique_ptr<OSQPWorkspace, std::function<void(OSQPWorkspace *)>> m_work;
	std::unique_ptr<OSQPSettings> m_settings;
	std::unique_ptr<OSQPData> m_data;
	OSQPInfo m_latest_work_info;
	int64_t m_param_n;
	bool m_work_initialized = false;
	int64_t m_exitflag;

	OSQPResult solve();
	static void OSQPWorkspaceDeleter(OSQPWorkspace * ptr) noexcept;

  public:
	explicit OSQPInterface(
	    const c_float eps_abs = std::numeric_limits<c_float>::epsilon(), const bool polish = true);
	OSQPInterface(const Eigen::MatrixXd & P, const Eigen::MatrixXd & A,
	    const std::vector<double> & q, const std::vector<double> & l, const std::vector<double> & u,
	    const c_float eps_abs);
	OSQPInterface(const CSC_Matrix & P, const CSC_Matrix & A, const std::vector<double> & q,
	    const std::vector<double> & l, const std::vector<double> & u, const c_float eps_abs);
	~OSQPInterface();

	OSQPResult optimize();
	OSQPResult optimize(const Eigen::MatrixXd & P, const Eigen::MatrixXd & A,
	    const std::vector<double> & q, const std::vector<double> & l,
	    const std::vector<double> & u);

	int64_t initializeProblem(const Eigen::MatrixXd & P, const Eigen::MatrixXd & A,
	    const std::vector<double> & q, const std::vector<double> & l,
	    const std::vector<double> & u);
	int64_t initializeProblem(CSC_Matrix P, CSC_Matrix A, const std::vector<double> & q,
	    const std::vector<double> & l, const std::vector<double> & u);

	bool setWarmStart(
	    const std::vector<double> & primal_variables, const std::vector<double> & dual_variables);
	bool setPrimalVariables(const std::vector<double> & primal_variables);
	bool setDualVariables(const std::vector<double> & dual_variables);

	void updateP(const Eigen::MatrixXd & P_new);
	void updateCscP(const CSC_Matrix & P_csc);
	void updateA(const Eigen::MatrixXd & A_new);
	void updateCscA(const CSC_Matrix & A_csc);
	void updateQ(const std::vector<double> & q_new);
	void updateL(const std::vector<double> & l_new);
	void updateU(const std::vector<double> & u_new);
	void updateBounds(const std::vector<double> & l_new, const std::vector<double> & u_new);
	void updateEpsAbs(const double eps_abs);
	void updateEpsRel(const double eps_rel);
	void updateMaxIter(const int iter);
	void updateVerbose(const bool verbose);
	void updateRhoInterval(const int rho_interval);
	void updateRho(const double rho);
	void updateAlpha(const double alpha);
	void updateScaling(const int scaling);
	void updatePolish(const bool polish);
	void updatePolishRefinementIteration(const int polish_refine_iter);
	void updateCheckTermination(const int check_termination);

	inline int64_t getTakenIter() const { return static_cast<int64_t>(m_latest_work_info.iter); }
	inline std::string getStatusMessage() const
	{
		return static_cast<std::string>(m_latest_work_info.status);
	}
	inline int64_t getStatus() const { return static_cast<int64_t>(m_latest_work_info.status_val); }
	inline int64_t getStatusPolish() const
	{
		return static_cast<int64_t>(m_latest_work_info.status_polish);
	}
	inline double getRunTime() const { return m_latest_work_info.run_time; }
	inline double getObjVal() const { return m_latest_work_info.obj_val; }
	inline int64_t getExitFlag() const { return m_exitflag; }

	void logUnsolvedStatus(const std::string & prefix_message = "") const;
};

}  // namespace mpl::osqp_interface