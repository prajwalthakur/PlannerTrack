// Copyright 2022 Tier IV, Inc.

#include <project_utils/geometry_utils.hpp>

#include <vector>

namespace
{
constexpr double close_s_threshold = 1e-6;

static inline rclcpp::Logger get_logger()
{
	constexpr const char * logger{"autoware_motion_utils.resample_utils"};
	return rclcpp::get_logger(logger);
}

template <class T>
[[nodiscard]] bool validate_size(const T & points)
{
	return points.size() >= 2;
}

// template <class T>
// [[nodiscard]] bool validate_resampling_range(
//     const T & points, const std::vector<double> & resampling_intervals)
// {
// 	const double points_length = autoware::motion_utils::calcArcLength(points);
// 	return points_length >= resampling_intervals.back();
// }

/**
 * @brief remove overlapping points through points container.
 * Overlapping is determined by calculating the distance between 2 consecutive points.
 * If the distance between them is less than a threshold, they will be considered overlapping.
 * @param points points of trajectory, path, ...
 * @param start_idx index to start the overlap remove calculation from through the points
 * container. Indices before that index will be considered non-overlapping. Default = 0
 * @return points container without overlapping points
 */
template <class T>
[[nodiscard]] T removeOverlapPoints(const T & points, const size_t start_idx = 0)
{
	if (points.size() < start_idx + 1) {
		return points;
	}

	T dst;
	dst.reserve(points.size());

	for (size_t i = 0; i <= start_idx; ++i) {
		dst.push_back(points.at(i));
	}

	constexpr double eps = 1.0E-08;
	for (size_t i = start_idx + 1; i < points.size(); ++i) {
		const auto prev_p = mpl::geometry_utils::get_point(dst.back());
		const auto curr_p = mpl::geometry_utils::get_point(points.at(i));
		if (std::abs(prev_p.x - curr_p.x) < eps && std::abs(prev_p.y - curr_p.y) < eps) {
			continue;
		}
		dst.push_back(points.at(i));
	}
	return dst;
}

template <class T>
[[nodiscard]] bool validate_points_duplication(const T & points)
{
	for (size_t i = 0; i < points.size() - 1; ++i) {
		const auto & curr_pt = mpl::geometry_utils::get_point(points.at(i));
		const auto & next_pt = mpl::geometry_utils::get_point(points.at(i + 1));
		const double ds = mpl::geometry_utils::calcDistance2d(curr_pt, next_pt);
		if (ds < close_s_threshold) {
			return false;
		}
	}

	return true;
}

template <class T>
[[nodiscard]] bool validate_arguments(
    const T & input_points, const std::vector<double> & resampling_intervals)
{
	// Check size of the arguments
	if (!validate_size(input_points)) {
		RCLCPP_DEBUG(get_logger(), "invalid argument: The number of input points is less than 2");
		// autoware_utils_system::print_backtrace();
		return false;
	}
	if (!validate_size(resampling_intervals)) {
		RCLCPP_DEBUG(
		    get_logger(), "invalid argument: The number of resampling intervals is less than 2");
		// autoware_utils_system::print_backtrace();
		return false;
	}

	// Check resampling range
	// if (!validate_resampling_range(input_points, resampling_intervals)) {
	// 	RCLCPP_DEBUG(
	// 	    get_logger(), "invalid argument: resampling interval is longer than input points");
	// 	// autoware_utils_system::print_backtrace();
	// 	return false;
	// }

	// Check duplication
	if (!validate_points_duplication(input_points)) {
		RCLCPP_DEBUG(get_logger(), "invalid argument: input points has some duplicated points");
		// autoware_utils_system::print_backtrace();
		return false;
	}

	return true;
}

template <class T>
[[nodiscard]] bool validate_arguments(const T & input_points, const double resampling_interval)
{
	// Check size of the arguments
	if (!validate_size(input_points)) {
		RCLCPP_DEBUG(get_logger(), "invalid argument: The number of input points is less than 2");
		// autoware_utils_system::print_backtrace();
		return false;
	}

	// check resampling interval
	// if (resampling_interval < autoware::motion_utils::overlap_threshold) {
	// 	RCLCPP_DEBUG(get_logger(), "invalid argument: resampling interval is less than %f",
	// 	    autoware::motion_utils::overlap_threshold);
	// 	autoware_utils_system::print_backtrace();
	// 	return false;
	// }

	// Check duplication
	if (!validate_points_duplication(input_points)) {
		RCLCPP_DEBUG(get_logger(), "invalid argument: input points has some duplicated points");
		// autoware_utils_system::print_backtrace();
		return false;
	}

	return true;
}
}  // namespace
