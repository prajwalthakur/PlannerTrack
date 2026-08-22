/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
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
#include "rclcpp/rclcpp.hpp"
#include "pid_controller/pid_controller.hpp"

/** \file
 * \brief Not part of the build -- `CMakeLists.txt` only compiles
 * `pid_controller.cpp`/`simple_pid.cpp` into `pid_controller_lib`, and the
 * actual runnable executable (`pid_controller_node_exe`) is auto-generated
 * by `rclcpp_components_register_node` (see \ref plugin_architecture,
 * section 3), not by this `main()`. Also stale: \ref PidControllerNode's
 * constructor now requires a `rclcpp::NodeOptions` argument this call
 * doesn't pass.
 */
int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<mpl::control::pid_controller::PidControllerNode>();

	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
