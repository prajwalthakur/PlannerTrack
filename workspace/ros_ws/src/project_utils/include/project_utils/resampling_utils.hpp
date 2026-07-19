#pragma once
#include "project_utils/geometry_utils.hpp"
#include "project_utils/validation_utils.hpp"

#include "f110_msgs/msg/wpnt_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "project_utils_msgs/msg/trajectory.hpp"
#include "project_utils_msgs/msg/trajectory_point.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mpl::extra_utils
{

inline void convertPathWptsToTrajectory(const f110_msgs::msg::WpntArray & waypoints,
    project_utils_msgs::msg::Trajectory & trajectory,
    const std_msgs::msg::Header & header = std_msgs::msg::Header{})
{
	const std::size_t num_wp = waypoints.wpnts.size();

	trajectory.points.clear();
	trajectory.points.reserve(num_wp);
	trajectory.header = header;

	double t_prev = 0.0;
	geometry_msgs::msg::Point prev_point;
	bool is_first_point = true;

	for (const auto & wpnt : waypoints.wpnts) {
		project_utils_msgs::msg::TrajectoryPoint traj_point;

		// Position
		traj_point.pose.position.x = wpnt.x_m;
		traj_point.pose.position.y = wpnt.y_m;
		traj_point.pose.position.z = 0.0;

		// Orientation from yaw
		// Assumation : That the trajectory is same as path yaw, which is not
		// True in general
		// only when the car is following closely the path
		traj_point.pose.orientation = mpl::geometry_utils::createQuaternionFromYaw(wpnt.psi_rad);

		// Velocity
		traj_point.longitudinal_velocity_mps = wpnt.vx_mps;
		traj_point.lateral_velocity_mps = 0.0;

		// Acceleration
		traj_point.acceleration_mps2 = wpnt.ax_mps2;

		// Steering (not directly available from waypoint)
		traj_point.front_wheel_angle_rad = 0.0;
		traj_point.rear_wheel_angle_rad = 0.0;

		// Time from start
		if (is_first_point) {
			traj_point.time_from_start.sec = 0;
			traj_point.time_from_start.nanosec = 0;
			is_first_point = false;
		} else {
			const double ds =
			    mpl::geometry_utils::calcDistance2d(prev_point, traj_point.pose.position);

			const double v = std::max(static_cast<double>(wpnt.vx_mps), 0.1);
			const double dt = ds / v;

			t_prev += dt;

			traj_point.time_from_start.sec = static_cast<int32_t>(t_prev);

			traj_point.time_from_start.nanosec =
			    static_cast<uint32_t>((t_prev - std::floor(t_prev)) * 1e9);
		}

		traj_point.track_kappa_radpm = wpnt.kappa_radpm;
		trajectory.points.push_back(traj_point);

		prev_point = traj_point.pose.position;
	}
}

/**
 * @brief A resampling function for a trajectory. Note that in a default setting, position xy are
 *        resampled by spline interpolation, position z are resampled by linear interpolation, twist
 *        informaiton(velocity and acceleration) are resampled by zero_order_hold, and heading rate
 *        is resampled by linear interpolation. The rest of the category is resampled by linear
 *        interpolation. Orientation of the resampled path are calculated by a forward difference
 *        method based on the interpolated position x and y.
 * @param input_trajectory input trajectory to resample
 * @param resampled_arclength arclength that contains length of each resampling points from initial
 *        point
 * @param use_akima_spline_for_xy If true, it uses linear interpolation to resample position x and
 *        y. Otherwise, it uses spline interpolation
 * @param use_lerp_for_z If true, it uses linear interpolation to resample position z.
 *        Otherwise, it uses spline interpolation
 * @param use_zero_order_hold_for_twist If true, it uses zero_order_hold to resample
 *        longitudinal, lateral velocity and acceleration. Otherwise, it uses linear interpolation
 * @return resampled trajectory
 */
// inline void resampleTrajectory(mpl::interpolation::SplineInterpolationPoints2d * interpolator,
//     const project_utils_msgs::msg::Trajectory & input_trajectory,
//     project_utils::msg::Trajectory & output_trajectory,
//     const std::vector<double> & resampled_arclength, const bool use_akima_spline_for_xy = false,
//     const bool use_lerp_for_z = true, const bool use_zero_order_hold_for_twist = true)
// {
// 	if (!validate_arguments(input_trajectory.points, resampled_arclength)) {
// 		return input_trajectory;
// 	}

// 	// Input trajectory information
// 	std::vector<double> input_arclength;
// 	std::vector<geometry_msgs::msg::Pose> input_pose;
// 	std::vector<double> v_lon;
// 	std::vector<double> v_lat;
// 	std::vector<double> heading_rate;
// 	std::vector<double> acceleration;
// 	std::vector<double> front_wheel_angle;
// 	std::vector<double> rear_wheel_angle;
// 	std::vector<double> time_from_start;
// 	input_arclength.reserve(input_trajectory.points.size());
// 	input_pose.reserve(input_trajectory.points.size());
// 	v_lon.reserve(input_trajectory.points.size());
// 	v_lat.reserve(input_trajectory.points.size());
// 	heading_rate.reserve(input_trajectory.points.size());
// 	acceleration.reserve(input_trajectory.points.size());
// 	front_wheel_angle.reserve(input_trajectory.points.size());
// 	rear_wheel_angle.reserve(input_trajectory.points.size());
// 	time_from_start.reserve(input_trajectory.points.size());

// 	input_arclength.push_back(0.0);
// 	input_pose.push_back(input_trajectory.points.front().pose);
// 	v_lon.push_back(input_trajectory.points.front().longitudinal_velocity_mps);
// 	v_lat.push_back(input_trajectory.points.front().lateral_velocity_mps);
// 	heading_rate.push_back(input_trajectory.points.front().heading_rate_rps);
// 	acceleration.push_back(input_trajectory.points.front().acceleration_mps2);
// 	front_wheel_angle.push_back(input_trajectory.points.front().front_wheel_angle_rad);
// 	rear_wheel_angle.push_back(input_trajectory.points.front().rear_wheel_angle_rad);
// 	time_from_start.push_back(
// 	    rclcpp::Duration(input_trajectory.points.front().time_from_start).seconds());

// 	for (size_t i = 1; i < input_trajectory.points.size(); ++i) {
// 		const auto & prev_pt = input_trajectory.points.at(i - 1);
// 		const auto & curr_pt = input_trajectory.points.at(i);
// 		const double ds =
// 		    mpl::geometry_utils::calcDistance2d(prev_pt.pose.position, curr_pt.pose.position);

// 		input_arclength.push_back(ds + input_arclength.back());
// 		input_pose.push_back(curr_pt.pose);
// 		v_lon.push_back(curr_pt.longitudinal_velocity_mps);
// 		v_lat.push_back(curr_pt.lateral_velocity_mps);
// 		heading_rate.push_back(curr_pt.heading_rate_rps);
// 		acceleration.push_back(curr_pt.acceleration_mps2);
// 		front_wheel_angle.push_back(curr_pt.front_wheel_angle_rad);
// 		rear_wheel_angle.push_back(curr_pt.rear_wheel_angle_rad);
// 		time_from_start.push_back(rclcpp::Duration(curr_pt.time_from_start).seconds());
// 	}

// 	// Set Zero Velocity After Stop Point
// 	// If the longitudinal velocity is zero, set the velocity to zero after that point.
// 	bool stop_point_found_in_v_lon = false;
// 	bool seen_nonzero_v_lon = false;
// 	constexpr double epsilon = 1e-4;
// 	for (size_t i = 0; i < v_lon.size(); ++i) {
// 		const bool is_zero = std::abs(v_lon.at(i)) < epsilon;
// 		if (!is_zero) {
// 			seen_nonzero_v_lon = true;
// 		}
// 		if (seen_nonzero_v_lon && is_zero) {
// 			stop_point_found_in_v_lon = true;
// 		}
// 		if (stop_point_found_in_v_lon) {
// 			v_lon.at(i) = 0.0;
// 		}
// 	}

// 	const auto interpolated_pose =
// 	    interpolator->getSplineInterpolatedPoses(input_pose, resampled_arclength);
// 	const bool is_driving_forward =
// 	    isDrivingForward(interpolated_pose.at(0), interpolated_pose.at(1));

// 	insertOrientation(interpolated_pose, is_driving_forward);

// 	// Initial orientation is depend on the initial value of the resampled_arclength
// 	// when backward driving
// 	if (!is_driving_forward && resampled_arclength.front() < 1e-3) {
// 		interpolated_pose.at(0).orientation = input_pose.at(0).orientation;
// 	}

// 	return interpolated_pose;
// }

// inline bool isDrivingForward(
//     const geometry_msgs::msg::Pose & src_pose, const geometry_msgs::msg::Pose & dst_pose)
// {
// 	// check the first point direction
// 	const double src_yaw = tf2::getYaw(src_pose.orientation);
// 	const double pose_direction_yaw =
// mpl::geometry_utils::calcAzimuthAngle(src_pose.orientation,dst_pose.orientation);

// 	return std::fabs(mpl::geometry_utils::normalizeAngles(src_yaw - pose_direction_yaw)) <
// 	    M_PI / 2.0;
// }

/**
 * @brief Insert orientation to each point in points container (trajectory, path, ...)
 * @param points points of trajectory, path, ... (input / output)
 * @param is_driving_forward  flag indicating the order of points is forward or backward
 */
// template <class T>
// inline void insertOrientation(T & points, const bool is_driving_forward)
// {
// 	if (is_driving_forward) {
// 		for (size_t i = 0; i < points.size() - 1; ++i) {
// 			const auto & src_point = mpl::geometry_utils::get_point(points.at(i));
// 			const auto & dst_point = mpl::geometry_utils::get_point(points.at(i + 1));
// 			// TODO(Yuki TAKAGI): The current implementation sets a value with the sign inverted.
// 			// Fix in https://github.com/autowarefoundation/autoware_core/issues/571
// 			const double yaw = mpl::geometry_utils::calcAzimuthAngle(src_point, dst_point);
// 			points.at(i).orientation =
// 			    mpl::geometry_utils::createQuaternionFromYaw(yaw) if (i == points.size() - 2)
// 			{
// 				// Terminal orientation is same as the point before it

// 				// TODO:
// 				mpl::geometry_utils::set_orientation(
// 				    mpl::geometry_utils::get_pose(points.at(i)).orientation, points.at(i + 1));
// 			}
// 		}
// 	} else {
// 		for (size_t i = points.size() - 1; i >= 1; --i) {
// 			const auto & src_point = mpl::geometry_utils::get_point(points.at(i));
// 			const auto & dst_point = mpl::geometry_utils::get_point(points.at(i - 1));
// 			const double yaw = mpl::geometry_utils::calc_azimuth_angle(src_point, dst_point);
// 			const double yaw = mpl::geometry_utils::calcAzimuthAngle(src_point, dst_point);
// 			points.at(i).orientation = mpl::geometry_utils::createQuaternionFromYaw(yaw)
// 		}
// 		// Initial orientation is same as the point after it
// 		mpl::geometry_utils::set_orientation(
// 		    mpl::geometry_utils::get_pose(points.at(1)).orientation, points.at(0));
// 	}
// }

/**
 * @brief Convert project_utils_msgs::msg::Trajectory to
 * std::vector<project_utils_msgs::msg::TrajectoryPoint>.
 */

//////////////////////////////////////////////////////////////////////////

inline std::vector<project_utils_msgs::msg::TrajectoryPoint> convertToTrajectoryPointArray(
    const project_utils_msgs::msg::Trajectory & trajectory)
{
	std::vector<project_utils_msgs::msg::TrajectoryPoint> output(trajectory.points.size());
	std::copy(trajectory.points.begin(), trajectory.points.end(), output.begin());
	return output;
}

// inline void setResampledTrajectory(mpl::interpolation::SplineInterpolationPoints2d *
// interpolator,
//     project_utils_msgs::msg::Trajectory & trajectory,
//     project_utils_msgs::msg::Trajectory & trajectorySampled,
//     std::vector<project_utils_msgs::msg::TrajectoryPoint> & trajSampledWayPoints, double
//     resampling_ds)
// {
// 	interpolator->initPoints(trajectory.points);
// 	trajectorySampled  = trajectory;
// 	trajSampledWayPoints = convertToTrajectoryPointArray(trajectorySampled);
// 	// Interpolate with constant interval distance.
// 	// size_t s_size = interpolator->getSize();
// 	// double traj_length =
// 	//     interpolator->getAccumulatedLength(s_size - 1);  // length of the trajectory
// 	// std::vector<double> out_arclength;
// 	// for (double s = 0; s < traj_length; s += resampling_ds) {
// 	// 	out_arclength.push_back(s);
// 	// }
// 	// resampleTrajectory(interpolator, trajectory, trajectorySampled, out_arclength);
// 	// trajectorySampled->points.back() = trajectory.points.back();
// 	// trajectorySampled->header = trajectory.header;
// 	// // TODO:
// 	// trajSampledWayPoints =
// autoware::motion_utils::convertToTrajectoryPointArray(trajectorySampled);

// 	// After resampling, its better to initialize the interpolator with resampled points
// 	//interpolator->initPoints(trajectorySample.points)
// }

// double calcCurvature(const size_t closest_idx)
// {
// 	// Calculate current curvature
// 	const size_t idx_dist = static_cast<size_t>(std::max(
// 	    static_cast<int>((param_.curvature_calculation_distance) / param_.resampling_ds), 1));

// 	// Find the points in trajectory to calculate curvature
// 	size_t next_idx = trajectory_resampled_->points.size() - 1;
// 	size_t prev_idx = 0;

// 	if (static_cast<size_t>(closest_idx) >= idx_dist) {
// 		prev_idx = closest_idx - idx_dist;
// 	} else {
// 		// return zero curvature when backward distance is not long enough in the trajectory
// 		return 0.0;
// 	}

// 	if (trajectory_resampled_->points.size() - 1 >= closest_idx + idx_dist) {
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
// 		current_curvature = autoware_utils::calc_curvature(
// 		    autoware_utils::get_point(trajectory_resampled_->points.at(prev_idx)),
// 		    autoware_utils::get_point(trajectory_resampled_->points.at(closest_idx)),
// 		    autoware_utils::get_point(trajectory_resampled_->points.at(next_idx)));
// 	} catch (std::exception const & e) {
// 		// ...code that handles the error...
// 		RCLCPP_WARN(rclcpp::get_logger("pure_pursuit"), "%s", e.what());
// 		current_curvature = 0.0;
// 	}
// 	return current_curvature;
// }

// double calcCurvaturewithPoints(const geometry_msgs::msg::Point & p1,
//     const geometry_msgs::msg::Point & p2, const geometry_msgs::msg::Point & p3)
// {
// 	// Calculation details are described in the following page
// 	// https://en.wikipedia.org/wiki/Menger_curvature
// 	const double denominator =
// 	    calcDistance2d(p1, p2) * calcDistance2d(p2, p3) * calcDistance2d(p3, p1);
// 	if (std::fabs(denominator) < 1e-10) {
// 		throw std::runtime_error("points are too close for curvature calculation.");
// 	}
// 	return 2.0 * ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) / denominator;
// }

/**
 * @brief find nearest point index through points container for a given pose.
 * Finding nearest point is determined by looping through the points container,
 * and finding the nearest point to the given pose in terms of squared 2D distance and yaw
 * deviation. The index of the point with minimum distance and yaw deviation comparing to the given
 * pose will be returned.
 * @param points points of trajectory, path, ...
 * @param pose given pose
 * @param max_dist max distance used to get squared distance for finding the nearest point to given
 * pose
 * @param max_yaw max yaw used for finding nearest point to given pose
 * @return index of nearest point (index or none if not found)
 */

//////////////////////////////////////////////////////////////////////////

inline std::optional<size_t> findNearestIndex(
    const std::vector<project_utils_msgs::msg::TrajectoryPoint> & points,
    const geometry_msgs::msg::Pose & pose,
    const double max_dist = std::numeric_limits<double>::max(),
    const double max_yaw = std::numeric_limits<double>::max())
{
	if (points.size() == 0) {
		throw std::runtime_error(" [pp_controller.utils]:points are empty ");
		return {};
	}

	const double max_squared_dist = max_dist * max_dist;

	double min_squared_dist = std::numeric_limits<double>::max();
	bool is_nearest_found = false;
	size_t min_idx = 0;

	for (size_t i = 0; i < points.size(); ++i) {
		const auto squared_dist =
		    mpl::geometry_utils::calcDistance2dSquare(points.at(i).pose, pose);
		if (squared_dist > max_squared_dist || squared_dist >= min_squared_dist) {
			continue;
		}
		const auto yaw = mpl::geometry_utils::calcAzimuthAngle(
		    mpl::geometry_utils::getYawFromQuaternion(points.at(i).pose.orientation),
		    mpl::geometry_utils::getYawFromQuaternion(pose.orientation));
		if (std::fabs(yaw) > max_yaw) {
			continue;
		}

		min_squared_dist = squared_dist;
		min_idx = i;
		is_nearest_found = true;
	}

	if (is_nearest_found) {
		return min_idx;
	}
	return std::nullopt;
}
}  // namespace mpl::extra_utils