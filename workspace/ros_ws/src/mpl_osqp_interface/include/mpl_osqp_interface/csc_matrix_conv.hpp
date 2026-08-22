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
#include "osqp/glob_opts.h"

#include <Eigen/Core>

#include <ostream>
#include <vector>

namespace mpl::osqp_interface
{

struct CSC_Matrix
{
	std::vector<c_float> m_vals;
	std::vector<c_int> m_row_idxs;
	std::vector<c_int> m_col_idxs;

	friend std::ostream & operator<<(std::ostream & os, const CSC_Matrix & matrix)
	{
		os << "CSC_Matrix: {\n";
		os << "\tm_vals: [";
		for (auto it = std::begin(matrix.m_vals); it != std::end(matrix.m_vals); ++it) {
			os << *it;
			if (std::next(it) != std::end(matrix.m_vals)) os << ", ";
		}
		os << "],\n\tm_row_idxs: [";
		for (auto it = std::begin(matrix.m_row_idxs); it != std::end(matrix.m_row_idxs); ++it) {
			os << *it;
			if (std::next(it) != std::end(matrix.m_row_idxs)) os << ", ";
		}
		os << "],\n\tm_col_idxs: [";
		for (auto it = std::begin(matrix.m_col_idxs); it != std::end(matrix.m_col_idxs); ++it) {
			os << *it;
			if (std::next(it) != std::end(matrix.m_col_idxs)) os << ", ";
		}
		os << "]\n}\n";
		return os;
	}
};

CSC_Matrix calCSCMatrix(const Eigen::MatrixXd & mat);
CSC_Matrix calCSCMatrixTrapezoidal(const Eigen::MatrixXd & mat);
void printCSCMatrix(const CSC_Matrix & csc_mat);

}  // namespace mpl::osqp_interface