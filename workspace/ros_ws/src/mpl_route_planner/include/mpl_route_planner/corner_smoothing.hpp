/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
// Adapted from Open Navigation LLC's nav2_route / Polymath Robotics
// (corner_smoothing.hpp). Same tangent-line-corner-fillet construction used
// by hand to build the intersection's turn connectors earlier -- here it's
// applied once, at path-densification time, to whatever kink exists
// between two consecutive route edges, instead of being pre-baked into the
// graph file.
#pragma once

#include "mpl_route_planner/types.hpp"
#include "mpl_route_planner/utils.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace mpl_route
{

/**
 * @class mpl_route::CornerArc
 * @brief Fits a circular fillet of a given radius into the corner formed by
 * (start -> corner -> end), for use by PathConverter to smooth a route's
 * dense path at each node it passes through.
 */
class CornerArc
{
  public:
	/**
	 * @param start start coordinate of the corner to be smoothed
	 * @param corner corner coordinate (the shared node between two edges)
	 * @param end end coordinate of the corner to be smoothed
	 * @param minimum_radius smoothing radius to fit to the corner
	 * @param angle_threshold Angle threshold (rad). If the corner's angle
	 *        exceeds this (i.e. it's nearly straight already), no smoothing
	 *        is applied.
	 */
	CornerArc(const Coordinates & start, const Coordinates & corner, const Coordinates & end,
	    float minimum_radius, float angle_threshold)
	{
		start_edge_length_ = hypotf(corner.x - start.x, corner.y - start.y);
		end_edge_length_ = hypotf(end.x - corner.x, end.y - corner.y);

		// Degenerate (zero-length) edge would blow up the equations below.
		if (start_edge_length_ == 0.0f || end_edge_length_ == 0.0f) {
			return;
		}

		float angle = getAngleBetweenEdges(start, corner, end);

		// Cannot smooth a 0 degree angle (u-turn/back-and-forth) or an
		// angle greater than angle_threshold (already nearly straight).
		if (std::abs(angle) < 1E-6f || std::abs(angle) > angle_threshold) {
			return;
		}

		float tangent_length = minimum_radius / std::tan(std::fabs(angle) / 2.0f);

		// If the requested radius doesn't fit within the available edge
		// lengths, decline to smooth rather than overrun into the previous/
		// next corner -- caller falls back to passing straight through.
		if (tangent_length < start_edge_length_ && tangent_length < end_edge_length_) {
			float start_ux = (start.x - corner.x) / start_edge_length_;
			float start_uy = (start.y - corner.y) / start_edge_length_;
			float end_ux = (end.x - corner.x) / end_edge_length_;
			float end_uy = (end.y - corner.y) / end_edge_length_;

			float bisector_x = start_ux + end_ux;
			float bisector_y = start_uy + end_uy;
			float bisector_mag = std::sqrt(bisector_x * bisector_x + bisector_y * bisector_y);
			float unit_bisector_x = bisector_x / bisector_mag;
			float unit_bisector_y = bisector_y / bisector_mag;

			start_coordinate_.x = corner.x + start_ux * tangent_length;
			start_coordinate_.y = corner.y + start_uy * tangent_length;

			end_coordinate_.x = corner.x + end_ux * tangent_length;
			end_coordinate_.y = corner.y + end_uy * tangent_length;

			float bisector_length = minimum_radius / std::sin(angle / 2.0f);
			circle_center_.x = corner.x + unit_bisector_x * bisector_length;
			circle_center_.y = corner.y + unit_bisector_y * bisector_length;

			valid_corner_ = true;
		}
	}

	~CornerArc() = default;

	/**
	 * @brief Sample points along the fitted arc
	 * @param max_angle_resolution Angular step (rad) between sampled points
	 * @param poses Output vector to append the sampled points to
	 */
	void interpolateArc(
	    const float & max_angle_resolution, std::vector<geometry_msgs::msg::PoseStamped> & poses)
	{
		float r_start_x = start_coordinate_.x - circle_center_.x;
		float r_start_y = start_coordinate_.y - circle_center_.y;
		float r_end_x = end_coordinate_.x - circle_center_.x;
		float r_end_y = end_coordinate_.y - circle_center_.y;

		float cross = r_start_x * r_end_y - r_start_y * r_end_x;
		float dot = r_start_x * r_end_x + r_start_y * r_end_y;
		float signed_angle = std::atan2(cross, dot);

		// Lower bound of 1 guards against divide-by-zero.
		int n = std::max(1, static_cast<int>(std::ceil(std::abs(signed_angle) / max_angle_resolution)));
		float angle_resolution = signed_angle / n;

		for (int i = 0; i < n; i++) {
			float angle = i * angle_resolution;
			float x = circle_center_.x + (r_start_x * std::cos(angle) - r_start_y * std::sin(angle));
			float y = circle_center_.y + (r_start_x * std::sin(angle) + r_start_y * std::cos(angle));
			poses.push_back(utils::toMsg(x, y));
		}
	}

	/**
	 * @brief Whether a valid arc was fit (false if the requested radius
	 * didn't fit in the available edge lengths, or the corner was
	 * degenerate/already-straight)
	 */
	bool isCornerValid() const { return valid_corner_; }

	Coordinates getCornerStart() const { return start_coordinate_; }
	Coordinates getCornerEnd() const { return end_coordinate_; }

  protected:
	/**
	 * @brief Unsigned angle (rad) at the corner formed by start-corner-end
	 */
	float getAngleBetweenEdges(
	    const Coordinates & start, const Coordinates & corner, const Coordinates & end)
	{
		float start_dx = start.x - corner.x;
		float start_dy = start.y - corner.y;
		float end_dx = end.x - corner.x;
		float end_dy = end.y - corner.y;

		return std::acos(
		    (start_dx * end_dx + start_dy * end_dy) / (start_edge_length_ * end_edge_length_));
	}

  private:
	bool valid_corner_{false};
	float start_edge_length_{0.0f};
	float end_edge_length_{0.0f};
	Coordinates start_coordinate_;
	Coordinates end_coordinate_;
	Coordinates circle_center_;
};

}  // namespace mpl_route
