/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "ssc_planner/ssc_visualizer.hpp"

#include <std_msgs/msg/color_rgba.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace ssc_planner
{

visualization_msgs::msg::MarkerArray buildCorridorMarkers(
    const mt::vec_E<mt::vec_E<SpatioTemporalSemanticCubeNd<2>>> & finalCorridor,
    const std::vector<int> & ifCorridorValid, const std::string & frameId,
    const rclcpp::Time & stamp)
{
	visualization_msgs::msg::MarkerArray markerArray;
	int id = 0;

	for (std::size_t corridorIdx = 0; corridorIdx < finalCorridor.size(); ++corridorIdx) {
		const bool isValid =
		    corridorIdx < ifCorridorValid.size() && ifCorridorValid[corridorIdx] != 0;

		for (const auto & cube : finalCorridor[corridorIdx]) {
			visualization_msgs::msg::Marker marker;
			marker.header.frame_id = frameId;
			marker.header.stamp = stamp;
			marker.ns = "ssc_corridor";
			marker.id = id++;
			marker.type = visualization_msgs::msg::Marker::CUBE;
			marker.action = visualization_msgs::msg::Marker::ADD;

			marker.pose.position.x = 0.5 * (cube.pLb[0] + cube.pUb[0]);
			marker.pose.position.y = 0.5 * (cube.pLb[1] + cube.pUb[1]);
			marker.pose.position.z = 0.5 * (cube.tLb + cube.tUb);
			marker.pose.orientation.w = 1.0;

			// A degenerate (zero-extent) cube would be invisible/warn in
			// RViz -- floor each scale at a small epsilon.
			constexpr float kMinScale = 0.01f;
			marker.scale.x = std::max(cube.pUb[0] - cube.pLb[0], kMinScale);
			marker.scale.y = std::max(cube.pUb[1] - cube.pLb[1], kMinScale);
			marker.scale.z = std::max(cube.tUb - cube.tLb, kMinScale);

			if (isValid) {
				marker.color.r = 0.2f;
				marker.color.g = 0.6f;
				marker.color.b = 1.0f;
			} else {
				marker.color.r = 1.0f;
				marker.color.g = 0.1f;
				marker.color.b = 0.1f;
			}
			marker.color.a = 0.35f;

			markerArray.markers.push_back(marker);
		}
	}

	return markerArray;
}

namespace
{
geometry_msgs::msg::Point toCartesian(
    const mpl::interpolation::SplineInterpolationPoints2d & refLane, double s, double d,
    double z)
{
	// The corridor's s-extent (bounded by ssc_planner.yaml's grid config /
	// kinematic reachability) can legitimately grow beyond how long the
	// actual reference path data is. getSplineInterpolatedYaw clamps s
	// internally, but getSplineInterpolatedPointAt doesn't -- it throws
	// std::invalid_argument on out-of-domain s. Clamp here so a corridor
	// that outgrows the path doesn't crash the node, just visually pins
	// those cubes to the path's last known position/heading.
	const double sMin = refLane.getAccumulatedLength(0);
	const double sMax = refLane.getAccumulatedLength(refLane.getSize() - 1);
	const double sClamped = std::clamp(s, sMin, sMax);

	const auto p = refLane.getSplineInterpolatedPointAt(sClamped);
	const double yaw = refLane.getSplineInterpolatedYaw(0, sClamped);
	geometry_msgs::msg::Point out;
	out.x = p.x - d * std::sin(yaw);
	out.y = p.y + d * std::cos(yaw);
	out.z = z;
	return out;
}
}  // namespace

visualization_msgs::msg::MarkerArray buildCorridorMarkersCartesian(
    const mt::vec_E<mt::vec_E<SpatioTemporalSemanticCubeNd<2>>> & finalCorridor,
    const std::vector<int> & ifCorridorValid,
    const mpl::interpolation::SplineInterpolationPoints2d & refLane, const std::string & frameId,
    const rclcpp::Time & stamp)
{
	// Just above ground so it reads as a road overlay rather than floating.
	constexpr double kZOffset = 0.05;

	visualization_msgs::msg::Marker fillMarker;
	fillMarker.header.frame_id = frameId;
	fillMarker.header.stamp = stamp;
	fillMarker.ns = "ssc_corridor_fill";
	fillMarker.id = 0;
	fillMarker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
	fillMarker.action = visualization_msgs::msg::Marker::ADD;
	fillMarker.pose.orientation.w = 1.0;
	fillMarker.scale.x = 1.0;
	fillMarker.scale.y = 1.0;
	fillMarker.scale.z = 1.0;

	visualization_msgs::msg::Marker outlineMarker;
	outlineMarker.header = fillMarker.header;
	outlineMarker.ns = "ssc_corridor_outline";
	outlineMarker.id = 0;
	outlineMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
	outlineMarker.action = visualization_msgs::msg::Marker::ADD;
	outlineMarker.pose.orientation.w = 1.0;
	outlineMarker.scale.x = 0.01;
	outlineMarker.color.r = 1.0f;
	outlineMarker.color.g = 1.0f;
	outlineMarker.color.b = 1.0f;
	outlineMarker.color.a = 0.6f;

	// SSC cubes deliberately overlap heavily in s (generous overlap keeps
	// the downstream QP's continuity constraint feasible -- see the plan
	// doc's reasoning on why CorridorRelaxation is skippable). Drawing
	// every individual cube's rectangle produces a messy, jagged outline
	// where they overlap. Since all cubes here share essentially the same
	// d-extent, their union collapses to one clean envelope: [min(s_lb),
	// max(s_ub)] x [min(d_lb), max(d_ub)]. Sample that envelope finely
	// along s (not just its 4 corners) so the ribbon actually curves with
	// the reference lane through a turn, instead of cutting a straight
	// chord across it.
	constexpr int kNumSamples = 60;

	for (std::size_t corridorIdx = 0; corridorIdx < finalCorridor.size(); ++corridorIdx) {
		const auto & cubes = finalCorridor[corridorIdx];
		if (cubes.empty()) {
			continue;
		}
		const bool isValid =
		    corridorIdx < ifCorridorValid.size() && ifCorridorValid[corridorIdx] != 0;

		std_msgs::msg::ColorRGBA fillColor;
		fillColor.a = 0.55f;
		if (isValid) {
			fillColor.r = 0.15f;
			fillColor.g = 0.55f;
			fillColor.b = 1.0f;
		} else {
			fillColor.r = 1.0f;
			fillColor.g = 0.15f;
			fillColor.b = 0.15f;
		}

		float sMin = cubes.front().pLb[0];
		float sMax = cubes.front().pUb[0];
		float dMin = cubes.front().pLb[1];
		float dMax = cubes.front().pUb[1];
		for (const auto & cube : cubes) {
			sMin = std::min(sMin, cube.pLb[0]);
			sMax = std::max(sMax, cube.pUb[0]);
			dMin = std::min(dMin, cube.pLb[1]);
			dMax = std::max(dMax, cube.pUb[1]);
		}
		if (sMax <= sMin) {
			continue;
		}

		std::vector<geometry_msgs::msg::Point> topEdge, bottomEdge;
		topEdge.reserve(kNumSamples + 1);
		bottomEdge.reserve(kNumSamples + 1);
		for (int i = 0; i <= kNumSamples; ++i) {
			const double s = sMin + (sMax - sMin) * static_cast<double>(i) / kNumSamples;
			topEdge.push_back(toCartesian(refLane, s, dMax, kZOffset));
			bottomEdge.push_back(toCartesian(refLane, s, dMin, kZOffset));
		}

		for (int i = 0; i < kNumSamples; ++i) {
			const auto & p00 = bottomEdge[i];
			const auto & p10 = bottomEdge[i + 1];
			const auto & p01 = topEdge[i];
			const auto & p11 = topEdge[i + 1];
			for (const auto & pt :
			    {p00, p01, p11, p00, p11, p10, p00, p11, p01, p00, p10, p11}) {
				fillMarker.points.push_back(pt);
				fillMarker.colors.push_back(fillColor);
			}
		}

		for (std::size_t i = 0; i + 1 < topEdge.size(); ++i) {
			outlineMarker.points.push_back(topEdge[i]);
			outlineMarker.points.push_back(topEdge[i + 1]);
		}
		for (std::size_t i = 0; i + 1 < bottomEdge.size(); ++i) {
			outlineMarker.points.push_back(bottomEdge[i]);
			outlineMarker.points.push_back(bottomEdge[i + 1]);
		}
		outlineMarker.points.push_back(topEdge.front());
		outlineMarker.points.push_back(bottomEdge.front());
		outlineMarker.points.push_back(topEdge.back());
		outlineMarker.points.push_back(bottomEdge.back());
	}

	visualization_msgs::msg::MarkerArray markerArray;
	markerArray.markers.push_back(fillMarker);
	markerArray.markers.push_back(outlineMarker);
	return markerArray;
}

}  // namespace ssc_planner
