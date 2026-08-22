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

///////////////////////////////////////////////////////////////////////////

#include "project_utils/planning_utils.hpp"
#include "regulated_pure_pursuit/optimizer.hpp"

/** \file
 * \brief \ref regulatedpp_controller::optimizer::Optimizer "Optimizer"
 * implementation: per-cycle trajectory resampling, the pure-pursuit
 * lookahead-target search, and the curvature/lateral-error-regulated
 * speed command.
 */

namespace regulatedpp_controller::optimizer
{

Optimizer::Optimizer(mpl::rclcpp_utils::Parameters * parameters, mpl::rclcpp_utils::Logger logger,
    const std::string & parentName, const std::string & name, rclcpp::Node & node)
    : mNode{node}, mParameters(parameters), mLogger(logger), mParentName(parentName), mName(name)
{
	mLogger.info("configuring optimizer.");
	onConfigure();
	mLogger.info("configured.");
}

void Optimizer::onConfigure()
{
	// model setup
	mpl::rclcpp_utils::Logger logger(mLogger, "model");
	// mModel = std::make_unique<mpcc_controller::model::SingleTrackModelExt>(
	//     mParameters, logger, mParentName, mParentName + ".vehicle");

	auto parentParamGetter = mParameters->getParamGetter(mParentName);
	double lf = 0.0, lr = 0.0;
	parentParamGetter(lf, "vehicle.lf", 0.0);
	parentParamGetter(lr, "vehicle.lr", 0.0);
	mOptimParam.wheel_base = lf + lr;
	// helper class constructions

	mInterpolator = std::make_unique<mpl::interpolation::SplineInterpolationPoints2d>();
	// auto x_next_rk4_vec = integrator.step(model.stateToVector(x), model.inputToVector(u));

	// publisher
	pub_debug_values_ = mNode.create_publisher<project_utils_msgs::msg::Float32MultiArrayStamped>(
	    "~/debug/ld_outputs", rclcpp::QoS{1});

	// parameters
	auto paramGetter = mParameters->getParamGetter(mParentName + ".optimizer");
	paramGetter(mOptimParam.ld_velocity_ratio, "ld_velocity_ratio", 1.4);
	paramGetter(mOptimParam.ld_lateral_error_ratio, "ld_lateral_error_ratio", 2.5);
	paramGetter(mOptimParam.ld_curvature_ratio, "ld_curvature_ratio", 60.0);

	paramGetter(
	    mOptimParam.long_ld_lateral_error_threshold, "long_ld_lateral_error_threshold", 0.5);

	paramGetter(mOptimParam.min_lookahead_distance, "min_lookahead_distance", 1.2);
	paramGetter(mOptimParam.max_lookahead_distance, "max_lookahead_distance", 8.0);

	paramGetter(mOptimParam.converged_steer_rad, "converged_steer_rad", 0.1);

	paramGetter(mOptimParam.reverse_min_lookahead_distance, "reverse_min_lookahead_distance", 1.5);

	paramGetter(mOptimParam.prediction_ds, "prediction_ds", 0.2);
	paramGetter(mOptimParam.prediction_distance_length, "prediction_distance_length", 10.0);

	paramGetter(mOptimParam.resampling_ds, "resampling_ds", 0.1);

	paramGetter(mOptimParam.curvature_calculation_distance, "curvature_calculation_distance", 1.5);

	paramGetter(mOptimParam.enable_path_smoothing, "enable_path_smoothing", false);

	paramGetter(mOptimParam.path_filter_moving_ave_num, "path_filter_moving_ave_num", 10.0);

	paramGetter(mOptimParam.closest_thr_dist, "closest_thr_dist", 3.0);

	paramGetter(mOptimParam.closest_thr_ang, "closest_thr_ang", M_PI / 4);

	paramGetter(mOptimParam.speed_lookahead, "speed_lookahead", 0.2);

	paramGetter(mOptimParam.lat_err_coeff, "lat_err_coeff", 1.0);

	paramGetter(mOptimParam.min_curv_est, "min_curv_est", 0.0);

	paramGetter(mOptimParam.max_curv_est, "max_curv_est", 0.8);

	paramGetter(mOptimParam.max_steering_angle, "max_steering_angle", 0.3);

	paramGetter(mOptimParam.set_external_target_speed, "set_external_target_speed", false);

	paramGetter(mOptimParam.external_target_speed, "external_target_speed", 0.3);

	mLogger.info("Optimzer initialized with min ld %.3f max ld %.3f , set_external_target_speed %d , external_target_speed %.3f", mOptimParam.min_lookahead_distance,
	    mOptimParam.max_lookahead_distance, mOptimParam.set_external_target_speed, mOptimParam.external_target_speed);
	reset();
}

void Optimizer::reset()
{
}

float Optimizer::generateSpeedCommand(const trajectory_follower::InputData & inputData)
{
	float speed_command = 0.0;
	if (mOptimParam.set_external_target_speed) {
		speed_command = mOptimParam.external_target_speed;
	} else {
		// speed calculation based on some heuristic
		// lookahead for speed
		auto & curr_pose = inputData.mCurrentOdometry.pose.pose;
		auto & curr_speed = inputData.mCurrentOdometry.twist.twist;
		auto adv_ts_speed = mOptimParam.speed_lookahead;
		float x_pred = curr_pose.position.x + curr_speed.linear.x * adv_ts_speed;
		float y_pred = curr_pose.position.y + curr_speed.linear.y * adv_ts_speed;
		auto s_ec_pair = mInterpolator->projectPointOntoSpline(x_pred, y_pred);

		[[maybe_unused]] float s = static_cast<float>(s_ec_pair.first);
		float lateral_error = static_cast<float>(s_ec_pair.second);

		const auto closest_idx_result =
		    mpl::extra_utils::findNearestIndex(mSampledTrajwayPoints, curr_pose, 2.0, M_PI_4);
		if (!closest_idx_result) {
			mLogger.error("cannot find closest waypoint");
			return {};
		}
		const double target_speed = static_cast<float>(
		    mSampledTrajwayPoints.at(*closest_idx_result).longitudinal_velocity_mps);

		float kappa =
		    static_cast<float>(mSampledTrajwayPoints.at(*closest_idx_result)
		                           .track_kappa_radpm);  // hack : track curvature appended in traj

		speed_command = speedAdjustLatError(kappa, target_speed, lateral_error);
	}

	return speed_command;
}

float Optimizer::speedAdjustLatError(float kappa, float target_speed, float lateral_error)
{
	if (mOptimParam.lat_err_coeff <= 0.0f) {
		return target_speed;
	}
	float min_curv = mOptimParam.min_curv_est;
	float max_curv = mOptimParam.max_curv_est;

	const float denom = std::max(max_curv - min_curv, 1e-6f);

	float norm_curv = (kappa - min_curv) / denom;

	norm_curv = std::clamp(norm_curv, 0.0f, 1.0f);

	const float scale = (1.0f - mOptimParam.lat_err_coeff) +
	    mOptimParam.lat_err_coeff * std::exp(-std::abs(lateral_error) * norm_curv);

	return target_speed * scale;
}

bool Optimizer::optimize(const trajectory_follower::InputData & inputData,
    trajectory_follower::HybridOutput & outputData)
{
	[[maybe_unused]] auto i_ = inputData;
	[[maybe_unused]] auto o_ = outputData;
	// mLogger.info("in optimizer");

	[[maybe_unused]] auto & currentOdom = inputData.mCurrentOdometry;
	[[maybe_unused]] auto & currentSteering = inputData.mCurrentSteering.steering_tire_angle;
	// mLogger.info("currentOdom referenced");

	//mLogger.info("local wp size = %d", inputData.mLocalWpArray.wpnts.size());

	// mpl::extra_utils::convertPathWptsToTrajectory(
	//     inputData.mLocalWpArray, mLocalTrajectoryToFollow);
	mLocalTrajectoryToFollow = inputData.mCurrentTrajectory;
	// mLogger.info("convertPathWptsToTrajectory called");

	// project_utils_msgs::msg::Trajectory
	if (mInterpolator == nullptr) mLogger.error("mInterpolator is nullptr!");
	//mLogger.info("traj size = %d", mLocalTrajectoryToFollow.points.size());
	mInterpolator->initPoints(mLocalTrajectoryToFollow.points, mLogger);
	// mLogger.info("initPoints called");
	mSampledLocalTrajToFollow = mLocalTrajectoryToFollow;
	mSampledTrajwayPoints =
	    mpl::extra_utils::convertToTrajectoryPointArray(mSampledLocalTrajToFollow);
	// mpl::extra_utils::setResampledTrajectory(mInterpolator.get(), mLocalTrajectoryToFollow,
	//     mSampledLocalTrajToFollow, mSampledTrajwayPoints, mOptimParam.resampling_ds);

	// mLogger.info("going to calculate lateral command");
	// Lateral command
	const auto lat_cmd_msg = generateLatControlCmd(inputData);

	// Longitudinal command
	float cmd_speed = generateSpeedCommand(inputData);

	project_utils_msgs::msg::Control cmd_;
	cmd_.stamp = mNode.get_clock()->now();

	cmd_.longitudinal.velocity = cmd_speed;
	cmd_.longitudinal.acceleration = 0.0;

	cmd_.lateral.steering_tire_angle = lat_cmd_msg.steering_tire_angle;
	cmd_.lateral.steering_tire_rotation_rate = 0.0;
	outputData.mControlCmd = cmd_;
	mLogger.info_throttle(*mNode.get_clock(), loggerThrottleInterval, "calculated the commanded value in pure pursuit, returning now");
	return true;
}

Lateral Optimizer::generateLatControlCmd(const trajectory_follower::InputData & inputData)
{
	// Generate the control command
	const auto pp_output = calcTargetCurvature(true, inputData);
	Lateral output_cmd;

	if (pp_output) {
		output_cmd = generateCtrlCmdMsg(pp_output->curvature);
		mPrevCommand = std::optional<Lateral>(output_cmd);
		// publishDebugMarker();
	} else {
		mLogger.warn("failed to solve pure_pursuit for control command calculation");
		if (mPrevCommand) {
			output_cmd = *mPrevCommand;
		} else {
			output_cmd = generateCtrlCmdMsg(0.0);
		}
	}
	return output_cmd;
}

std::optional<PpOutput> Optimizer::calcTargetCurvature(
    bool is_control_output, const trajectory_follower::InputData & inputData)
{
	auto & odom = inputData.mCurrentOdometry;
	auto & pose = odom.pose.pose;

	// Ignore invalid trajectory
	if (mSampledLocalTrajToFollow.points.size() < 3) {
		mLogger.warn("received path size is < 3, ignored");
		return {};
	}

	// Calculate target point for velocity/acceleration

	const auto closest_idx_result =
	    mpl::extra_utils::findNearestIndex(mSampledTrajwayPoints, pose, 2.0, M_PI_4);
	if (!closest_idx_result) {
		mLogger.error("cannot find closest waypoint");
		return {};
	}

	double target_vel = 0.0;
	if (mOptimParam.set_external_target_speed) {
		target_vel = mOptimParam.external_target_speed;
	} else {
		target_vel= mSampledTrajwayPoints.at(*closest_idx_result).longitudinal_velocity_mps;
	}
	// calculate the lateral error
	auto pp = mInterpolator->projectPointOntoSpline(pose.position.x, pose.position.y);
	float lateral_error = pp.second;
	[[maybe_unused]] float closest_s = pp.first;
	// const double current_curvature = mInterpolator->getSplineInterpolatedCurvature(0.0,
	// closest_s);
	const double current_curvature =
	    static_cast<float>(mSampledTrajwayPoints.at(*closest_idx_result)
	                           .track_kappa_radpm);  // hack : track curvature appended in traj

	// calculate the current curvature calculation using three points ?
	// Disable since we are not storing the previous past trajectories point
	// const double current_curvature = calcCurvature(*closest_idx_result);

	// Calculate lookahead distance

	const bool is_reverse = (target_vel < 0);
	const double min_lookahead_distance = is_reverse ? mOptimParam.reverse_min_lookahead_distance
	                                                 : mOptimParam.min_lookahead_distance;
	double lookahead_distance = is_control_output
	    ? calcLookaheadDistance(lateral_error, current_curvature, odom.twist.twist.linear.x,
	          min_lookahead_distance, is_control_output)
	    : calcLookaheadDistance(lateral_error, current_curvature, target_vel,
	          min_lookahead_distance, is_control_output);

	// Run PurePursuit
	const auto pure_pursuit_result = runPP(pose, lookahead_distance, mSampledLocalTrajToFollow);
	if (!pure_pursuit_result.first) {
		return {};
	}

	const auto kappa = pure_pursuit_result.second;

	// Set debug data
	if (is_control_output) {
		mDebugData.next_target = mNextTgtPosition;
	}
	PpOutput output{};
	output.curvature = kappa;
	if (!is_control_output) {
		output.velocity = odom.twist.twist.linear.x;
	} else {
		output.velocity = target_vel;
	}

	return output;
}

double Optimizer::calcLookaheadDistance(const double lateral_error, const double curvature,
    const double velocity, const double min_ld, const bool is_control_cmd)
{
	const double vel_ld = abs(mOptimParam.ld_velocity_ratio * velocity);
	const double curvature_ld = -abs(mOptimParam.ld_curvature_ratio * curvature);
	double lateral_error_ld = 0.0;

	if (abs(lateral_error) >= mOptimParam.long_ld_lateral_error_threshold) {
		// If lateral error is higher than threshold, we should make ld larger to prevent entering
		// the road with high heading error.
		lateral_error_ld = abs(mOptimParam.ld_lateral_error_ratio * lateral_error);
	}

	const double total_ld = std::clamp(vel_ld + curvature_ld + lateral_error_ld, min_ld,
	    static_cast<double>(mOptimParam.max_lookahead_distance));

	auto pubDebugValues = [&]() {
		project_utils_msgs::msg::Float32MultiArrayStamped debug_msg{};
		debug_msg.data.resize(TYPE::SIZE);
		debug_msg.data.at(TYPE::VEL_LD) = static_cast<float>(vel_ld);
		debug_msg.data.at(TYPE::CURVATURE_LD) = static_cast<float>(curvature_ld);
		debug_msg.data.at(TYPE::LATERAL_ERROR_LD) = static_cast<float>(lateral_error_ld);
		debug_msg.data.at(TYPE::TOTAL_LD) = static_cast<float>(total_ld);
		debug_msg.data.at(TYPE::VELOCITY) = static_cast<float>(velocity);
		debug_msg.data.at(TYPE::CURVATURE) = static_cast<float>(curvature);
		debug_msg.data.at(TYPE::LATERAL_ERROR) = static_cast<float>(lateral_error);
		debug_msg.stamp = mNode.get_clock()->now();
		pub_debug_values_->publish(debug_msg);
	};

	if (is_control_cmd) {
		pubDebugValues();
	}

	return total_ld;
}

Lateral Optimizer::generateCtrlCmdMsg(const double target_curvature)
{
	const float tmp_steering =
	    static_cast<float>(mpl::planning_utils::convertCurvatureToSteeringAngle(
	        mOptimParam.wheel_base, target_curvature));
	Lateral cmd;
	cmd.stamp = mNode.get_clock()->now();
	cmd.steering_tire_angle = static_cast<float>(std::min(
	    std::max(tmp_steering, -mOptimParam.max_steering_angle), mOptimParam.max_steering_angle));

	// pub_ctrl_cmd_->publish(cmd);
	return cmd;
}

std::pair<bool, double> Optimizer::runPP(const geometry_msgs::msg::Pose & currPose,
    double lookAheadDistance, project_utils_msgs::msg::Trajectory & trajToFollow)
{
	std::vector<geometry_msgs::msg::Pose> waypoints =
	    mpl::geometry_utils::extractPoses(trajToFollow);

	// skip small cusp
	auto closest_pair = mpl::planning_utils::findClosestIdxWithDistAngThr(
	    waypoints, currPose, mOptimParam.closest_thr_dist, mOptimParam.closest_thr_ang);

	if (!closest_pair.first) {
		mLogger.warn(
		    "cannot find, curr_bool: %d, closest_idx: %d", closest_pair.first, closest_pair.second);
		return std::make_pair(false, std::numeric_limits<double>::quiet_NaN());
	}

	int32_t next_wp_idx =
	    findNextPointIdx(currPose, waypoints, lookAheadDistance, closest_pair.second);
	if (next_wp_idx == -1) {
		mLogger.warn("lost next waypoint");
		return std::make_pair(false, std::numeric_limits<double>::quiet_NaN());
	}

	[[maybe_unused]] geometry_msgs::msg::Point loc_next_wp_ = waypoints.at(next_wp_idx).position;
	if (next_wp_idx == 0) {
		mNextTgtPosition = waypoints.at(next_wp_idx).position;
	} else {
		// linear interpolation to find exact position
		std::pair<bool, geometry_msgs::msg::Point> lerp_pair =
		    lerpNextTarget(currPose, waypoints, lookAheadDistance, next_wp_idx);
		if (!lerp_pair.first) {
			mLogger.warn(" lost target! ");
			return std::make_pair(false, std::numeric_limits<double>::quiet_NaN());
		}
		mNextTgtPosition = lerp_pair.second;
	}
	double kappa = mpl::planning_utils::calcCurvature(mNextTgtPosition, currPose);
	return std::make_pair(true, kappa);
}

int32_t Optimizer::findNextPointIdx(const geometry_msgs::msg::Pose & currPose,
    std::vector<geometry_msgs::msg::Pose> & waypoints, double lookAheadDistance,
    int32_t search_start_idx)
{
	// if waypoints are not given, do nothing.
	if (waypoints.empty() || search_start_idx == -1) {
		return -1;
	}

	// check if the waypoint direction is forward
	const int8_t wpDir =
	    mpl::planning_utils::getPathDirection(waypoints, mOptimParam.resampling_ds / 2.0);
	// look for the next waypoint
	for (int32_t i = search_start_idx; i < static_cast<int32_t>(waypoints.size()); ++i) {
		// if the first waypoint is the last
		if (i == static_cast<int32_t>(waypoints.size()) - 1) {
			return i;
		}
		if (wpDir == 0) {
			// wp is in forward direction (x_prev < x_next) normal forward driving
			// if wp is not in front of ego ? prev_wp ---> car --> next_wp
			auto ret = mpl::planning_utils::transformToRelativeCoordinate2D(
			    waypoints.at(i).position, currPose);
			if (ret.x < 0) {
				// wp is increasing in x direction (forward motion expected)
				// but the car pose is in different direction
				continue;
			}
		} else if (wpDir == 1) {
			// waypoint direction is backward
			// if waypoint is in front of ego, skip
			auto ret = mpl::planning_utils::transformToRelativeCoordinate2D(
			    waypoints.at(i).position, currPose);
			if (ret.x > 0) {
				continue;
			}
		} else {
			return -1;
		}
		const geometry_msgs::msg::Point & curr_motion_point = waypoints.at(i).position;
		const geometry_msgs::msg::Point & curr_pose_point = currPose.position;
		// if there exists an effective waypoint
		const double ds =
		    mpl::geometry_utils::calcDistance2dSquare(curr_motion_point, curr_pose_point);
		if (ds > std::pow(lookAheadDistance, 2)) {
			return i;
		}
	}
	return -1;
}

// linear interpolation of next target
std::pair<bool, geometry_msgs::msg::Point> Optimizer::lerpNextTarget(
    const geometry_msgs::msg::Pose & currPose, std::vector<geometry_msgs::msg::Pose> & waypoints,
    double lookAheadDistance, int32_t next_wp_idx)
{
	constexpr double ERROR2 = 1e-5;  // 0.00001
	const geometry_msgs::msg::Point & vec_end = waypoints.at(next_wp_idx).position;
	const geometry_msgs::msg::Point & vec_start = waypoints.at(next_wp_idx - 1).position;
	const geometry_msgs::msg::Pose & curr_pose = currPose;

	Eigen::Vector3d vec_a(
	    (vec_end.x - vec_start.x), (vec_end.y - vec_start.y), (vec_end.z - vec_start.z));

	if (vec_a.norm() < ERROR2) {
		mLogger.error("waypoint interval is almost 0");
		return std::make_pair(false, geometry_msgs::msg::Point());
	}

	const double lateral_error =
	    mpl::planning_utils::calcLateralError2D(vec_start, vec_end, curr_pose.position);

	if (fabs(lateral_error) > lookAheadDistance) {
		mLogger.error("lateral error is larger than lookahead distance");
		mLogger.error(
		    "lateral error: %lf, lookahead distance: %lf", lateral_error, lookAheadDistance);
		return std::make_pair(false, geometry_msgs::msg::Point());
	}

	/* calculate the position of the foot of a perpendicular line */
	Eigen::Vector2d uva2d(vec_a.x(), vec_a.y());
	uva2d.normalize();
	Eigen::Rotation2Dd rot =
	    (lateral_error > 0) ? Eigen::Rotation2Dd(-M_PI / 2.0) : Eigen::Rotation2Dd(M_PI / 2.0);
	Eigen::Vector2d uva2d_rot = rot * uva2d;

	geometry_msgs::msg::Point h;
	h.x = curr_pose.position.x + fabs(lateral_error) * uva2d_rot.x();
	h.y = curr_pose.position.y + fabs(lateral_error) * uva2d_rot.y();
	h.z = curr_pose.position.z;

	// if h touch the line segement, and length of lateral_error (i.e length of h is same as
	// lookAheadDistance)
	if (fabs(fabs(lateral_error) - lookAheadDistance) < ERROR2) {
		return std::make_pair(true, h);
	} else {
		// if there are two intersection
		// get intersection in front of vehicle
		const double s = sqrt(pow(lookAheadDistance, 2) - pow(lateral_error, 2));
		geometry_msgs::msg::Point res;
		res.x = h.x + s * uva2d.x();
		res.y = h.y + s * uva2d.y();
		res.z = curr_pose.position.z;
		return std::make_pair(true, res);
	}
}

// double  Optimizer::calcCurvature(const size_t closest_idx)
// {
// 	// Calculate current curvature
// 	const size_t idx_dist = static_cast<size_t>(std::max(
// 	    static_cast<int>((mOptimParam.curvature_calculation_distance) / mOptimParam.resampling_ds),
// 1));

// 	// Find the points in trajectory to calculate curvature
// 	size_t next_idx = mSampledLocalTrajToFollow->points.size() - 1;
// 	size_t prev_idx = 0;

// 	if (static_cast<size_t>(closest_idx) >= idx_dist) {
// 		prev_idx = closest_idx - idx_dist;
// 	} else {
// 		// return zero curvature when backward distance is not long enough in the trajectory
// 		return 0.0;
// 	}

// 	if (mSampledLocalTrajToFollow->points.size() - 1 >= closest_idx + idx_dist) {
// 		next_idx = closest_idx + idx_dist;
// 	} else {
// 		// return zero curvature when forward distance is not long enough in the trajectory
// 		return 0.0;
// 	}
// 	// TODO(k.sugahara): shift the center point of the curvature calculation to allow sufficient
// 	// distance, because if sufficient distance cannot be obtained in front or behind, the curvature
// 	// will be zero in the current implementation.

// 	// Calculate curvature assuming the trajectory points interval is constant
// 	double current_curvature = 0.0;

// 	try {
// 		current_curvature = mpl::geometry_utils::calcCurvaturewithPoints(
// 		    mpl::geometry_utils::get_point(mSampledLocalTrajToFollow->points.at(prev_idx)),
// 		    mpl::geometry_utils::get_point(mSampledLocalTrajToFollow->points.at(closest_idx)),
// 		    mpl::geometry_utils::get_point(mSampledLocalTrajToFollow->points.at(next_idx)));
// 	} catch (std::exception const & e) {
// 		// ...code that handles the error...
// 		mLogger.warn(rclcpp::get_logger("pure_pursuit"), "%s", e.what());
// 		current_curvature = 0.0;
// 	}
// 	return current_curvature;
// }

}  // namespace regulatedpp_controller::optimizer
