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

#include "project_utils_msgs/msg/float32_multi_array_stamped.hpp"
#include "project_utils_msgs/msg/trajectory.hpp"
#include "project_utils_msgs/msg/wpnt_array.hpp"
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

/// \brief Debug-visualization state (e.g. the current pure-pursuit lookahead target).
struct DebugData
{
	geometry_msgs::msg::Point next_target;
};

struct stAuxInput
{
};

/// \brief Tunable parameters for \c Optimizer, loaded from ROS params under `<parentName>.optimizer`.
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

/// \brief One pure-pursuit solve's result: target curvature and the velocity it was computed against.
struct PpOutput
{
	double curvature;
	double velocity;
};

/**
 * \brief Lateral (regulated pure-pursuit) + heuristic longitudinal speed
 * command generator used by \ref regulatedpp_controller::RegulatedPP
 * "RegulatedPP".
 */
class Optimizer
{
  public:
	Optimizer() = delete;
	Optimizer(mpl::rclcpp_utils::Parameters * parameters, mpl::rclcpp_utils::Logger logger,
	    const std::string & parentName, const std::string & name, rclcpp::Node & node);
	~Optimizer() = default;

	/// \brief Compute this cycle's combined lateral + longitudinal command into \p output_data.
	bool optimize(const trajectory_follower::InputData & input_data,
	    trajectory_follower::HybridOutput & output_data);
	/// \brief Load \ref mOptimParam from ROS params and construct helper objects (spline interpolator, publishers).
	void onConfigure();
	/// \brief Reset internal state (previous command, interpolator).
	void reset();
	// void getInitialGuess(mpcc_controller::model::stState & xInit);

  private:
	/// \brief Convert a waypoint array to plain geometry points.
	void convertWptsToPoints(const project_utils_msgs::msg::WpntArray & wpts,
	    std::vector<geometry_msgs::msg::Point> & geomPoints);
	// void updateInitialGuess(mpcc_controller::model::stState & xInit);
	// void generateInitialGuess(mpcc_controller::model::stState & xInit);
	// void unwrapInitialGuess();
	// void setResampledTrajectory();
	/// \brief Run the pure-pursuit solve and pack the result into a \c Lateral command message.
	Lateral generateLatControlCmd(const trajectory_follower::InputData & inputData);

	/// \brief Pure-pursuit target curvature for the current pose/trajectory, or \c nullopt if no valid target was found.
	std::optional<PpOutput> calcTargetCurvature(bool, const trajectory_follower::InputData &);

	/// \brief Convert a target curvature into a \c Lateral steering command message.
	Lateral generateCtrlCmdMsg(const double target_curvature);

	/// \brief Speed/curvature/lateral-error-scaled lookahead distance for the pure-pursuit search.
	double calcLookaheadDistance(const double lateral_error, const double curvature,
	    const double velocity, const double min_ld, const bool is_control_cmd);

	/// \brief Core pure-pursuit geometry solve: find the lookahead target on \p trajToFollow and its curvature.
	/// \return `{success, curvature}`.
	std::pair<bool, double> runPP(const geometry_msgs::msg::Pose & pose, double lookAheadDistance,
	    project_utils_msgs::msg::Trajectory & trajToFollow);

	/// \brief Index of the first waypoint at least \p lookAheadDistance ahead of \p currPose, searching from \p search_start_idx.
	int32_t findNextPointIdx(const geometry_msgs::msg::Pose & currPose,
	    std::vector<geometry_msgs::msg::Pose> & waypoints, double lookAheadDistance,
	    int32_t search_start_idx);

	/// \brief Linearly interpolate the exact lookahead-distance target point between waypoints.
	/// \return `{success, target point}`.
	std::pair<bool, geometry_msgs::msg::Point> lerpNextTarget(
	    const geometry_msgs::msg::Pose & currPose,
	    std::vector<geometry_msgs::msg::Pose> & waypoints, double lookAheadDistance,
	    int32_t next_wp_idx);

	/**
	 * \brief Target speed for this cycle: either \ref stOptimParam::external_target_speed
	 * "mOptimParam.external_target_speed" if externally overridden, or the
	 * trajectory's speed at the closest waypoint, scaled down by
	 * \ref speedAdjustLatError for curvature/lateral-error.
	 */
	float generateSpeedCommand(const trajectory_follower::InputData &);
	/// \brief Scale down \p target_speed when curvature and lateral error are both high (cuts speed into corners while off-track).
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
