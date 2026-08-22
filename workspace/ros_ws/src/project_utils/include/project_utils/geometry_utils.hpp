// Author Prajwal Thakur
#pragma once
#include "project_utils/vector2.hpp"

#include <Eigen/Dense>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <project_utils_msgs/msg/path_point.hpp>
#include <project_utils_msgs/msg/trajectory.hpp>
#include <project_utils_msgs/msg/trajectory_point.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include <cmath>

namespace mpl::geometry_utils
{

constexpr double PI = 3.14159265358979323846;

//////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////

template <class T>
inline geometry_msgs::msg::Point create_point(T x_, T y_, T z_)
{
	return geometry_msgs::build<geometry_msgs::msg::Point>().x(x_).y(y_).z(z_);
}

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
inline geometry_msgs::msg::Point get_point(const project_utils_msgs::msg::PathPoint & p)
{
	return p.pose.position;
}

//////////////////////////////////////////////////////////////////////////

template <>
inline geometry_msgs::msg::Point get_point(const project_utils_msgs::msg::TrajectoryPoint & p)
{
	return p.pose.position;
}

// template <>
// inline geometry_msgs::msg::Point get_point(const project_utils_msgs::msg::Wpnt & waypoint)
// {
// 	return create_point(waypoint.x_m, waypoint.y_m, 0.0);
// }

//////////////////////////////////////////////////////////////////////////

inline geometry_msgs::msg::Quaternion createQuaternionFromYaw(double yaw)
{
	tf2::Quaternion q;
	q.setRPY(0, 0, yaw);  // Roll, Pitch, Yaw
	// Convert tf2::Quaternion to geometry_msgs::msg::Quaternion
	geometry_msgs::msg::Quaternion msg_q = tf2::toMsg(q);
	return msg_q;
}

//////////////////////////////////////////////////////////////////////////

inline double getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q_msg)
{
	return tf2::getYaw(q_msg);
}

//////////////////////////////////////////////////////////////////////////

template <class T1, class T2>
inline double calcDistance2d(const T1 & point1, const T2 & point2)
{
	auto p1 = get_point(point1);
	auto p2 = get_point(point2);
	return std::hypot((p1.x - p2.x), (p1.y - p2.y));
}

//////////////////////////////////////////////////////////////////////////

template <class T1, class T2>
inline double calcDistance2dSquare(const T1 & point1, const T2 & point2)
{
	auto p1 = get_point(point1);
	auto p2 = get_point(point2);
	const double dx = p1.x - p2.x;
	const double dy = p1.y - p2.y;
	return dx * dx + dy * dy;
}

//////////////////////////////////////////////////////////////////////////

template <>
inline double calcDistance2dSquare(const Vector2 & p1, const Vector2 & p2)
{
	const double dx = p1.x() - p2.x();
	const double dy = p1.y() - p2.y();
	return dx * dx + dy * dy;
}

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
inline auto calcAzimuthAngle(const T & from, const F & to)
{
	return normalizeAngles(to - from);
}

//////////////////////////////////////////////////////////////////////////

template <typename T>
inline auto calcNormalizedAngles(const T & dy, const T & dx)
{
	return normalizeAngles(std::atan2(dy, dx));
}

// ---------------- Scalar normalized ----------------

//////////////////////////////////////////////////////////////////////////

template <typename T>
inline T shortestAngularDistanceNormalized(const T & from, const T & to)
{
	static_assert(std::is_floating_point_v<T>, "T must be floating point");

	T d1 = calcAzimuthAngle(from, to);
	T d2 = calcAzimuthAngle(from, normalizeAngles(to + M_PIf));

	return (std::abs(d1) < std::abs(d2)) ? d1 : d2;
}

// ---------------- Eigen normalized ----------------

//////////////////////////////////////////////////////////////////////////

inline Eigen::ArrayXf shortestAngularDistanceNormalized(
    const Eigen::Ref<const Eigen::ArrayXf> & from, const Eigen::Ref<const Eigen::ArrayXf> & to)
{
	auto d1 = calcAzimuthAngle(from, to);
	auto d2 = calcAzimuthAngle(from, normalizeAngles(to + M_PIf));
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

//////////////////////////////////////////////////////////////////////////

inline std::vector<geometry_msgs::msg::Pose> extractPoses(
    const project_utils_msgs::msg::Trajectory & trajectory)
{
	std::vector<geometry_msgs::msg::Pose> poses;

	for (const auto & p : trajectory.points) {
		poses.push_back(p.pose);
	}

	return poses;
}

//////////////////////////////////////////////////////////////////////////

inline float circleToCircleClosestDist(
    Vector2 c1Point, float c1Radius, Vector2 c2Point, float c2Radius)
{
	Vector2 deltaVec = c1Point - c2Point;
	return (std::hypot(deltaVec.x(), deltaVec.y()) - c1Radius - c2Radius);
}

//////////////////////////////////////////////////////////////////////////

inline float circleToLineSegClosestDist(
    Vector2 c1Point, float c1Radius, Vector2 start_point, Vector2 end_point)
{
	// projection parameter
	auto vec1 = end_point - start_point;
	auto vec2 = c1Point - start_point;
	double t = (vec1).dot(vec2) / vec1.dot(vec1);
	t = std::max(0.0, std::min(1.0, t));
	auto closestVec = start_point + t * (vec1);
	auto deltaVec = closestVec - c1Point;
	float dist = std::hypot(deltaVec.x(), deltaVec.y()) - c1Radius;
	return dist;
}

//////////////////////////////////////////////////////////////////////////

inline std::pair<float, Vector2> circleToLineSegClosestPoint(
    Vector2 c1Point, [[maybe_unused]] float c1Radius, Vector2 start_point, Vector2 end_point)
{
	// projection parameter
	auto vec1 = end_point - start_point;
	auto vec2 = c1Point - start_point;
	double t = (vec1).dot(vec2) / vec1.dot(vec1);
	t = std::max(0.0, std::min(1.0, t));
	auto closestVec = start_point + t * (vec1);
	return std::make_pair(t, closestVec);
}

//////////////////////////////////////////////////////////////////////////

inline float toRadians(float degree)
{
	return degree * (PI / 180.0);
}

//////////////////////////////////////////////////////////////////////////

inline geometry_msgs::msg::PoseStamped odomToPoseStamped(const nav_msgs::msg::Odometry & odom)
{
	geometry_msgs::msg::PoseStamped pose_stamped;

	pose_stamped.header = odom.header;
	pose_stamped.pose = odom.pose.pose;

	return pose_stamped;
}

// inline std::pair<float, Vector2> proejction(
//     Vector2 c1Point, Vector2 vectorLine)
// {
// 	// projection parameter
// 	auto vec1 = vectorLine;
// 	auto vec2 = c1Point - start_point;
// 	double t = (vec1).dot(vec2) / vec1.dot(vec1);
// 	auto closestVec = start_point + t * (vec1);
// 	return std::make_pair(t, closestVec);
// }

}  // namespace mpl::geometry_utils