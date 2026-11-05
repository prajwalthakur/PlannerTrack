// Copyright 2026 Prajwal Thakur
#include "regulated_pure_pursuit/lookahead_search.hpp"

#include "project_utils/geometry_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace la = regulatedpp_controller::lookahead;
using geometry_msgs::msg::Pose;

namespace
{
constexpr int kN = 100;        // waypoints on the full circle
constexpr double kR = 5.0;     // circle radius [m]
const double kDs = 2.0 * M_PI * kR / kN;  // spacing ~0.314 m

Pose poseOnCircle(double theta)
{
	Pose p;
	p.position.x = kR * std::cos(theta);
	p.position.y = kR * std::sin(theta);
	p.orientation = mpl::geometry_utils::createQuaternionFromYaw(theta + M_PI_2);  // CCW tangent
	return p;
}

/// \p n of the kN circle waypoints, counter-clockwise from theta = 0.
std::vector<Pose> circle(int n = kN)
{
	std::vector<Pose> wps;
	for (int i = 0; i < n; ++i) {
		wps.push_back(poseOnCircle(2.0 * M_PI * i / kN));
	}
	return wps;
}

double dist(const Pose & a, const Pose & b)
{
	return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
}
}  // namespace

TEST(LookaheadSearch, ClosedLoopDetection)
{
	EXPECT_EQ(la::closedLoopSize(circle()), kN);  // gap = one spacing (min_curv_planner)
	auto duplicated = circle();
	duplicated.push_back(duplicated.front());  // last == first (global_waypoints.json)
	EXPECT_EQ(la::closedLoopSize(duplicated), kN);
	EXPECT_EQ(la::closedLoopSize(circle(kN / 2)), 0);  // half circle: open
	EXPECT_EQ(la::closedLoopSize(circle(2)), 0);
}

TEST(LookaheadSearch, PreviousIdxWrapsOnlyWhenClosed)
{
	EXPECT_EQ(la::previousIdx(0, kN), kN - 1);
	EXPECT_EQ(la::previousIdx(7, kN), 6);
	EXPECT_EQ(la::previousIdx(7, 0), 6);
}

TEST(LookaheadSearch, WrapsAcrossTheSeam)
{
	const auto wps = circle();
	const Pose car = wps[kN - 2];
	const double ld = 1.0;
	const int idx = la::findNextPointIdx(car, wps, ld, kN - 2, kN, kDs / 2.0);
	// First waypoint farther than ld: 4 spacings ahead (chord of 4 * 0.314 m arc > 1 m), past the seam.
	EXPECT_EQ(idx, 2);
	EXPECT_GT(dist(wps[idx], car), ld);
	EXPECT_LT(dist(wps[la::previousIdx(idx, kN)], car), ld);
}

TEST(LookaheadSearch, OpenPathKeepsOriginalBehaviour)
{
	const auto wps = circle(kN / 2);
	const int last = kN / 2 - 1;
	const Pose car = wps[last - 1];
	// Nothing far enough before the end: the last waypoint.
	EXPECT_EQ(la::findNextPointIdx(car, wps, 1.0, last - 1, 0, kDs / 2.0), last);
	// Far enough inside the path: same answer as on the closed loop.
	EXPECT_EQ(la::findNextPointIdx(wps[10], wps, 1.0, 10, 0, kDs / 2.0), 14);
	EXPECT_EQ(la::findNextPointIdx(wps[10], circle(), 1.0, 10, kN, kDs / 2.0), 14);
}

TEST(LookaheadSearch, DuplicatedEndPointIsSkipped)
{
	auto wps = circle();
	wps.push_back(wps.front());
	const int loop = la::closedLoopSize(wps);
	// Closest waypoint is the duplicate (index kN): the search still wraps to 2 + ... ahead.
	const Pose car = wps[kN - 2];
	const int idx = la::findNextPointIdx(car, wps, 1.0, kN, loop, kDs / 2.0);
	EXPECT_EQ(idx, 2);
	EXPECT_LT(idx, kN);
}

TEST(LookaheadSearch, NothingFarEnoughOnClosedLoop)
{
	// ld larger than the circle's diameter: no waypoint qualifies.
	EXPECT_EQ(la::findNextPointIdx(circle()[0], circle(), 2.5 * kR, 0, kN, kDs / 2.0), -1);
}
