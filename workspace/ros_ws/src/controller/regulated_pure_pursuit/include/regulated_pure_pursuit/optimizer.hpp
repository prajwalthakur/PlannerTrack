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
#include "interpolation_utils/spline_interpolation_points_2d.hpp"
#include "project_utils/parameter.hpp"
#include "project_utils/planning_utils.hpp"
#include "project_utils/resampling_utils.hpp"
#include "trajectory_follower_base/hybrid_controller_base.h"
#include "trajectory_follower_base/input_data.h"

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "f110_msgs/msg/wpnt_array.hpp"
#include "project_utils_msgs/msg/float32_multi_array_stamped.hpp"
#include "project_utils_msgs/msg/trajectory.hpp"
#include <geometry_msgs/msg/pose.hpp>

#include <optional>
// #include <tf2/utils.h>

namespace
{
enum TYPE {
	VEL_LD = 0,
	CURVATURE_LD = 1,
	LATERAL_ERROR_LD = 2,
	TOTAL_LD = 3,
	CURVATURE = 4,
	LATERAL_ERROR = 5,
	VELOCITY = 6,
	SIZE  // this is the number of enum elements
};
}
using mpl::control::trajectory_follower::HybridControllerBase;
using mpl::control::trajectory_follower::HybridOutput;
using mpl::control::trajectory_follower::InputData;
using project_utils_msgs::msg::Control;
using project_utils_msgs::msg::Lateral;
using project_utils_msgs::msg::Trajectory;
using project_utils_msgs::msg::TrajectoryPoint;

namespace regulatedpp_controller::optimizer
{

struct DebugData
{
	geometry_msgs::msg::Point next_target;
};

struct stAuxInput
{
};

struct stOptimParam
{
	float ld_velocity_ratio{1.2};
	float ld_lateral_error_ratio{2.0};
	float ld_curvature_ratio{40.0};
	float long_ld_lateral_error_threshold{0.5};
	float min_lookahead_distance{1.0};
	float max_lookahead_distance{6.0};
	float converged_steer_rad{0.1};
	float reverse_min_lookahead_distance{1.5};
	float prediction_ds{0.2};
	float prediction_distance_length{10.0};
	float resampling_ds{0.1};
	float curvature_calculation_distance{1.5};

	bool enable_path_smoothing{false};

	float path_filter_moving_ave_num{15.0};

	float wheel_base{1.4};

	float closest_thr_dist{3.0};

	float closest_thr_ang{M_PI / 4};

	float speed_lookahead{0.2};

	float lat_err_coeff{1.0};

	float max_curv_est{0.8};

	float min_curv_est{0.01};

	float max_steering_angle{0.3};

	bool set_external_target_speed{false};
	
	float external_target_speed{0.5};
};

struct PpOutput
{
	double curvature;
	double velocity;
};

class Optimizer
{
  public:
	// Constructor
	Optimizer() = delete;
	Optimizer(mpl::rclcpp_utils::Parameters * parameters, mpl::rclcpp_utils::Logger logger,
	    const std::string & parentName, const std::string & name, rclcpp::Node & node);
	// Destructor
	~Optimizer() = default;

	bool optimize(const trajectory_follower::InputData & input_data,
	    trajectory_follower::HybridOutput & output_data);
	void onConfigure();
	void reset();
	// void getInitialGuess(mpcc_controller::model::stState & xInit);

  private:
	void convertWptsToPoints(const f110_msgs::msg::WpntArray & wpts,
	    std::vector<geometry_msgs::msg::Point> & geomPoints);
	// void updateInitialGuess(mpcc_controller::model::stState & xInit);
	// void generateInitialGuess(mpcc_controller::model::stState & xInit);
	// void unwrapInitialGuess();
	// void setResampledTrajectory();
	Lateral generateLatControlCmd(const trajectory_follower::InputData & inputData);

	std::optional<PpOutput> calcTargetCurvature(bool, const trajectory_follower::InputData &);

	Lateral generateCtrlCmdMsg(const double target_curvature);

	double calcLookaheadDistance(const double lateral_error, const double curvature,
	    const double velocity, const double min_ld, const bool is_control_cmd);

	std::pair<bool, double> runPP(const geometry_msgs::msg::Pose & pose, double lookAheadDistance,
	    project_utils_msgs::msg::Trajectory & trajToFollow);

	int32_t findNextPointIdx(const geometry_msgs::msg::Pose & currPose,
	    std::vector<geometry_msgs::msg::Pose> & waypoints, double lookAheadDistance,
	    int32_t search_start_idx);

	std::pair<bool, geometry_msgs::msg::Point> lerpNextTarget(
	    const geometry_msgs::msg::Pose & currPose,
	    std::vector<geometry_msgs::msg::Pose> & waypoints, double lookAheadDistance,
	    int32_t next_wp_idx);

	// long speed command
	float generateSpeedCommand(const trajectory_follower::InputData &);
	float speedAdjustLatError(float kappa, float target_speed, float lateral_error);

  private:
	// mpcc_controller::model::stState mXo;
	// mpcc_controller::model::stInput mInput;
	stOptimParam mOptimParam;
	rclcpp::Node & mNode;
	mpl::rclcpp_utils::Parameters * mParameters{nullptr};
	mpl::rclcpp_utils::Logger mLogger;
	std::string mParentName;
	std::string mName;
	mutable DebugData mDebugData;

	project_utils_msgs::msg::Trajectory mLocalTrajectoryToFollow;
	project_utils_msgs::msg::Trajectory mSampledLocalTrajToFollow;
	std::vector<project_utils_msgs::msg::TrajectoryPoint> mSampledTrajwayPoints;

	std::optional<project_utils_msgs::msg::Lateral> mPrevCommand;
	geometry_msgs::msg::Point mNextTgtPosition;
	// class constructions
	// std::unique_ptr<mpcc_controller::model::SingleTrackModelExt> mModel{nullptr};
	// std::unique_ptr<mpcc_controller::model::Integrator> mIntegrator{nullptr};
	std::unique_ptr<mpl::interpolation::SplineInterpolationPoints2d> mInterpolator{nullptr};

	// publisher
	rclcpp::Publisher<project_utils_msgs::msg::Float32MultiArrayStamped>::SharedPtr
	    pub_debug_values_;
	static constexpr double loggerThrottleInterval = 5000;  // in ms -> 5 sec
};

}  // namespace regulatedpp_controller::optimizer
