#include "project_utils/planning_utils.hpp"

#include "project_utils/geometry_utils.hpp"
// get closest point index from current pose
namespace mpl::planning_utils
{

//////////////////////////////////////////////////////////////////////////

std::pair<bool, int32_t> findClosestIdxWithDistAngThr(
    const std::vector<geometry_msgs::msg::Pose> & poses,
    const geometry_msgs::msg::Pose & current_pose, double th_dist, double th_yaw)
{
	double dist_squared_min = std::numeric_limits<double>::max();
	int32_t idx_min = -1;

	for (size_t i = 0; i < poses.size(); ++i) {
		const double ds =
		    mpl::geometry_utils::calcDistance2dSquare(poses.at(i).position, current_pose.position);
		if (ds > th_dist * th_dist) {
			continue;
		}

		const double yaw_pose = mpl::geometry_utils::getYawFromQuaternion(current_pose.orientation);
		const double yaw_ps = mpl::geometry_utils::getYawFromQuaternion(poses.at(i).orientation);
		const double yaw_diff = mpl::geometry_utils::calcAzimuthAngle(yaw_pose, yaw_ps);
		if (fabs(yaw_diff) > th_yaw) {
			continue;
		}

		if (ds < dist_squared_min) {
			dist_squared_min = ds;
			idx_min = i;
		}
	}

	return (idx_min >= 0) ? std::make_pair(true, idx_min) : std::make_pair(false, idx_min);
}

//////////////////////////////////////////////////////////////////////////

int8_t getPathDirection(const std::vector<geometry_msgs::msg::Pose> & poses, double th_dist)
{
	if (poses.size() < 2) {
		throw std::runtime_error("size of waypoints is smaller than 2");
		return 2;
	}
	for (uint32_t i = 0; i < poses.size(); i++) {
		geometry_msgs::msg::Pose prev;
		geometry_msgs::msg::Pose next;

		if (i == (poses.size() - 1)) {
			prev = poses.at(i - 1);
			next = poses.at(i);
		} else {
			prev = poses.at(i);
			next = poses.at(i + 1);
		}

		if (mpl::geometry_utils::calcDistance2dSquare(prev.position, next.position) >
		    th_dist * th_dist) {
			const auto rel_p = transformToRelativeCoordinate2D(next.position, prev);
			return (rel_p.x > 0.0) ? 0 : 1;
		}
	}

	throw std::runtime_error("path is something wrong");
	return 2;
}

// ref: http://www.mech.tohoku-gakuin.ac.jp/rde/contents/course/robotics/coordtrans.html
// (pu, pv): relative, (px, py): absolute, (ox, oy): origin
// (pu, pv) = rot^-1 * {(px, py) - (ox, oy)}

//////////////////////////////////////////////////////////////////////////

geometry_msgs::msg::Point transformToRelativeCoordinate2D(
    const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & origin)
{
	// translation
	geometry_msgs::msg::Point trans_p;
	trans_p.x = point.x - origin.position.x;
	trans_p.y = point.y - origin.position.y;

	// rotation (use inverse matrix of rotation)
	double yaw = tf2::getYaw(origin.orientation);

	geometry_msgs::msg::Point res;
	res.x = (cos(yaw) * trans_p.x) + (sin(yaw) * trans_p.y);
	res.y = ((-1) * sin(yaw) * trans_p.x) + (cos(yaw) * trans_p.y);
	res.z = origin.position.z;

	return res;
}

//////////////////////////////////////////////////////////////////////////

double calcRadius(
    const geometry_msgs::msg::Point & target, const geometry_msgs::msg::Pose & current_pose)
{
	constexpr double RADIUS_MAX = 1e9;
	const double denominator = 2 * transformToRelativeCoordinate2D(target, current_pose).y;
	const double numerator =
	    mpl::geometry_utils::calcDistance2dSquare(target, current_pose.position);

	if (fabs(denominator) > 0) {
		return numerator / denominator;
	} else {
		return RADIUS_MAX;
	}
}

//////////////////////////////////////////////////////////////////////////

double calcCurvature(
    const geometry_msgs::msg::Point & target, const geometry_msgs::msg::Pose & current_pose)
{
	constexpr double KAPPA_MAX = 1e9;
	const double radius = calcRadius(target, current_pose);

	if (fabs(radius) > 0) {
		return 1 / radius;
	} else {
		return KAPPA_MAX;
	}
}

//////////////////////////////////////////////////////////////////////////

double convertCurvatureToSteeringAngle(double wheel_base, double kappa)
{
	return atan(wheel_base * kappa);
}

/* a_vec = line_e - line_s, b_vec = point - line_s
 * a_vec x b_vec = |a_vec| * |b_vec| * sin(theta)
 *               = |a_vec| * lateral_error ( because, lateral_error = |b_vec| * sin(theta) )
 *
 * lateral_error = a_vec x b_vec / |a_vec|
 *        = (a_x * b_y - a_y * b_x) / |a_vec|
 */

//////////////////////////////////////////////////////////////////////////

double calcLateralError2D(const geometry_msgs::msg::Point & line_s,
    const geometry_msgs::msg::Point & line_e, const geometry_msgs::msg::Point & point)
{
	tf2::Vector3 a_vec((line_e.x - line_s.x), (line_e.y - line_s.y), 0.0);
	tf2::Vector3 b_vec((point.x - line_s.x), (point.y - line_s.y), 0.0);

	double lat_err = (a_vec.length() > 0) ? a_vec.cross(b_vec).z() / a_vec.length() : 0.0;
	return lat_err;
}

}  // namespace mpl::planning_utils