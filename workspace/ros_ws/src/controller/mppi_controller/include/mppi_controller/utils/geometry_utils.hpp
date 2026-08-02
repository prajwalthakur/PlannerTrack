#pragma once

#include "mppi_controller/models/control_sequence.hpp"
#include "mppi_controller/models/optimizer_settings.hpp"
#include "mppi_controller/models/path.hpp"
#include "mppi_controller/utils/types.hpp"

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include <cmath>
#include <type_traits>
#define M_PIF 3.141592653589793238462643383279502884e+00F
#define M_PIF_2 1.5707963267948966e+00F

namespace controller::mppi_controller::utils
{

//////////////////////////////////////////////////////////////////////////

template <class T>
inline geometry_msgs::msg::Point get_point(const T & p)
{
	return geometry_msgs::build<geometry_msgs::msg::Point>().x(p.x).y(p.y).z(p.z);
}

//////////////////////////////////////////////////////////////////////////

template <>
inline geometry_msgs::msg::Point get_point(const geometry_msgs::msg::Point & p)
{
	return p;
}

//////////////////////////////////////////////////////////////////////////

template <>
inline geometry_msgs::msg::Point get_point(const geometry_msgs::msg::Pose & p)
{
	return p.position;
}

//////////////////////////////////////////////////////////////////////////

template <>
inline geometry_msgs::msg::Point get_point(const geometry_msgs::msg::PoseStamped & p)
{
	return p.pose.position;
}

//////////////////////////////////////////////////////////////////////////

template <>
inline geometry_msgs::msg::Point get_point(const geometry_msgs::msg::PoseStamped::SharedPtr & p)
{
	return (*p).pose.position;
}

//////////////////////////////////////////////////////////////////////////

template <>
inline geometry_msgs::msg::Point get_point(const geometry_msgs::msg::PoseWithCovariance & p)
{
	return p.pose.position;
}

//////////////////////////////////////////////////////////////////////////

template <typename T1, typename T2>
bool isSamePoint(const T1 & t1, const T2 & t2)
{
	const auto p1 = get_point(t1);
	const auto p2 = get_point(t2);
	constexpr float epsilon = 1e-6;
	if (epsilon < std::abs(p1.x - p2.x) && epsilon < std::abs(p1.y - p2.y)) {
		return true;
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////

template <typename T1, typename T2>
bool isSamePointEuclid(const T1 & t1, const T2 & t2)
{
	const auto p1 = get_point(t1);
	const auto p2 = get_point(t2);
	constexpr float epsilon = 1e-6;
	float dx = p1.x - p2.x;
	float dy = p1.y - p2.y;
	if (dx * dx + dy * dy < epsilon * epsilon) return true;
	return false;
}
/**
 * @brief Get the euclidean distance between 2 geometry_msgs::Points
 * @param pos1 First point
 * @param pos1 Second point
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return float L2 distance
 */

//////////////////////////////////////////////////////////////////////////

inline float euclidean_distance(const geometry_msgs::msg::Point & pos1,
    const geometry_msgs::msg::Point & pos2, const bool is_3d = false)
{
	float dx = pos1.x - pos2.x;
	float dy = pos1.y - pos2.y;

	if (is_3d) {
		float dz = pos1.z - pos2.z;
		return std::hypot(dx, dy, dz);
	}

	return std::hypot(dx, dy);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::Poses
 * @param pos1 First pose
 * @param pos1 Second pose
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return float euclidean distance
 */

//////////////////////////////////////////////////////////////////////////

inline float euclidean_distance(const geometry_msgs::msg::Pose & pos1,
    const geometry_msgs::msg::Pose & pos2, const bool is_3d = false)
{
	float dx = pos1.position.x - pos2.position.x;
	float dy = pos1.position.y - pos2.position.y;

	if (is_3d) {
		float dz = pos1.position.z - pos2.position.z;
		return std::hypot(dx, dy, dz);
	}

	return std::hypot(dx, dy);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::PoseStamped
 * @param pos1 First pose
 * @param pos1 Second pose
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return float L2 distance
 */

//////////////////////////////////////////////////////////////////////////

inline float euclidean_distance(const geometry_msgs::msg::PoseStamped & pos1,
    const geometry_msgs::msg::PoseStamped & pos2, const bool is_3d = false)
{
	return euclidean_distance(pos1.pose, pos2.pose, is_3d);
}
/**
 * @brief Calculate the length of the provided path, starting at the provided index
 * @param path Path containing the poses that are planned
 * @param start_index Optional argument specifying the starting index for
 * the calculation of path length. Provide this if you want to calculate length of a
 * subset of the path.
 * @return float Path length
 */

//////////////////////////////////////////////////////////////////////////

inline float calculatePathLength(const nav_msgs::msg::Path & path, size_t start_index = 0)
{
	if (start_index + 1 >= path.poses.size()) {
		return 0.0;
	}
	float path_length = 0.0;
	for (size_t idx = start_index; idx < path.poses.size() - 1; ++idx) {
		path_length += euclidean_distance(path.poses[idx].pose, path.poses[idx + 1].pose);
	}
	return path_length;
}

// inline float calculatePathLength(const geometry_msgs::msg::PoseArray& path,
//                                 size_t start_index = 0)
// {
//     if (start_index + 1 >= path.poses.size()) {
//         return 0.0f;
//     }

//     float path_length = 0.0f;

//     for (size_t idx = start_index; idx < path.poses.size() - 1; ++idx) {
//         path_length += euclidean_distance(
//             path.poses[idx],
//             path.poses[idx + 1]);
//     }

//     return path_length;
// }

//////////////////////////////////////////////////////////////////////////

inline geometry_msgs::msg::Pose getLastPathPose(const models::Path & path)
{
	const unsigned int path_last_idx = path.x.size() - 1;

	auto last_orientation = path.yaws(path_last_idx);

	tf2::Quaternion pose_orientation;
	pose_orientation.setRPY(0.0, 0.0, last_orientation);

	geometry_msgs::msg::Pose pathPose;
	pathPose.position.x = path.x(path_last_idx);
	pathPose.position.y = path.y(path_last_idx);
	pathPose.orientation.x = pose_orientation.x();
	pathPose.orientation.y = pose_orientation.y();
	pathPose.orientation.z = pose_orientation.z();
	pathPose.orientation.w = pose_orientation.w();

	return pathPose;
}

/**
 * Find first element in iterator that is greater integrated distance than comparevalue
 */

//////////////////////////////////////////////////////////////////////////

template <typename Iter, typename Getter>
inline Iter firstAfterIntegratedDistance(Iter begin, Iter end, Getter getCompareVal)
{
	if (begin == end) {
		return end;
	}
	Getter dist = 0.0;
	for (Iter it = begin; it != end - 1; it++) {
		dist += euclidean_distance(*it, *(it + 1));
		if (dist > getCompareVal) {
			return it + 1;
		}
	}
	return end;
}

/**
 * Find element in iterator with the minimum calculated value
 */

//////////////////////////////////////////////////////////////////////////

template <typename Iter, typename Getter>
inline Iter minBy(Iter begin, Iter end, Getter getCompareVal)
{
	if (begin == end) return end;
	auto lowest = getCompareVal(*begin);
	Iter lowestIt = begin;
	begin++;
	for (Iter it = begin; it < end; ++it) {
		auto comp = getCompareVal(*it);
		if (comp <= lowest) {
			lowest = comp;
			lowestIt = it;
		}
	}
	return lowestIt;
}

//////////////////////////////////////////////////////////////////////////

// // ---------------- Scalar ----------------
// template<typename T>
// inline T normalizeAngles(const T& angle)
// {
//     //any decimal number type (with fractions)
//     static_assert(std::is_floating_point_v<T>, "T must be floating point");
//     return std::atan2(std::sin(angle), std::cos(angle));
// }

// // ---------------- Eigen ----------------
// template<>
// inline auto normalizeAngles(const mppi_mt::ArrayX& angles)
// {
//     return angles.unaryExpr([](auto x) {
//         return std::atan2(std::sin(x), std::cos(x));
//     });
// }

// ---------------- Scalar & Eigen Universal ----------------
// We remove the template specialization and use a single function
// that leverages Eigen's ADL (Argument Dependent Lookup)

//////////////////////////////////////////////////////////////////////////

template <typename T>
inline auto normalizeAngles(const T & angle)
{
	if constexpr (std::is_floating_point_v<T>) {
		// Standard scalar path
		return std::atan2(std::sin(angle), std::cos(angle));
	} else {
		// Eigen path: angle might be a CwiseBinaryOp (Expression Template)
		// We convert it to an array expression to use .sin(), .cos(), and .atan2()
		return angle.unaryExpr([](float x) { return std::atan2(std::sin(x), std::cos(x)); });
		// auto&& arg = angle.array();
		// return arg.sin().binaryExpr(arg.cos(), [](float s, float c) {
		//     return std::atan2(s, c);
		// });
	}
}

// ---------------- Shortest distance ----------------

//////////////////////////////////////////////////////////////////////////

template <typename T, typename F>
inline auto shortestAngularDistance(const T & from, const F & to)
{
	return normalizeAngles(to - from);
}

// ---------------- Scalar normalized ----------------

//////////////////////////////////////////////////////////////////////////

template <typename T>
inline T shortestAngularDistanceNormalized(const T & from, const T & to)
{
	static_assert(std::is_floating_point_v<T>, "T must be floating point");

	T d1 = shortestAngularDistance(from, to);
	T d2 = shortestAngularDistance(from, normalizeAngles(to + M_PIF));

	return (std::abs(d1) < std::abs(d2)) ? d1 : d2;
}

// ---------------- Eigen normalized ----------------

//////////////////////////////////////////////////////////////////////////

inline Eigen::ArrayXf shortestAngularDistanceNormalized(
    const Eigen::Ref<const Eigen::ArrayXf> & from, const Eigen::Ref<const Eigen::ArrayXf> & to)
{
	auto d1 = shortestAngularDistance(from, to);
	auto d2 = shortestAngularDistance(from, normalizeAngles(to + M_PIF));
	return (d1.abs() < d2.abs()).select(d1, d2);
}

/**
 * @brief Shift the columns of a 2D Eigen Array or scalar values of
 *    1D Eigen Array by 1 place.
 * @param e Eigen Array
 * @param direction direction in which Array will be shifted.
 *     1 for shift in right direction and -1 for left direction.
 */

//////////////////////////////////////////////////////////////////////////

inline void shiftColumnsByOnePlace(Eigen::Ref<Eigen::ArrayXXf> e, const int direction)
{
	if (abs(direction) != 1)
		throw std::logic_error("Invalid direction only 1 and -1 are valid values.");
	size_t size = e.size();  // total size (num of elements)
	if (size == 1)  // i.e scalar
		return;

	if ((e.cols() == 1 || e.rows() == 1) && size > 1) {
		auto startPtr = (direction == 1) ? e.data() + size - 2 : e.data() + 1;
		auto endPtr = (direction == 1) ? e.data() : e.data() + size - 1;
		while (startPtr != endPtr) {
			*(startPtr + direction) = *startPtr;
			startPtr -= direction;
		}
		*(startPtr + direction) = *startPtr;

	} else {
		auto startPtr = (direction == 1) ? e.data() + size - 2 * e.rows() : e.data() + e.rows();
		auto endPtr = (direction == 1) ? e.data() : e.data() + size - e.rows();
		auto span = e.rows();
		while (startPtr != endPtr) {
			std::copy(startPtr, startPtr + span, startPtr + direction * span);
			startPtr -= (direction * span);
		}
		std::copy(startPtr, startPtr + span, startPtr + direction * span);
	}
}
/**
 * @brief Convert data into TwistStamped
 * @param vx X velocity
 * @param wz Angular velocity
 * @param stamp Timestamp
 * @param frame Reference frame to use
 */

//////////////////////////////////////////////////////////////////////////

inline geometry_msgs::msg::TwistStamped toTwistStamped(
    float vx, float wz, const std_msgs::msg::Header & header)
{
	geometry_msgs::msg::TwistStamped twist;
	twist.header = header;
	twist.twist.linear.x = vx;
	twist.twist.angular.z = wz;

	return twist;
}

/**
 * @brief Convert data into TwistStamped
 * @param vx X velocity
 * @param vy Y velocity
 * @param wz Angular velocity
 * @param stamp Timestamp
 * @param frame Reference frame to use
 */

//////////////////////////////////////////////////////////////////////////

inline geometry_msgs::msg::TwistStamped toTwistStamped(
    float vx, float vy, float wz, const std_msgs::msg::Header & header)
{
	auto twist = toTwistStamped(vx, wz, header);
	twist.twist.linear.y = vy;
	return twist;
}

}  // namespace controller::mppi_controller::utils
namespace mppi_utils = controller::mppi_controller::utils;