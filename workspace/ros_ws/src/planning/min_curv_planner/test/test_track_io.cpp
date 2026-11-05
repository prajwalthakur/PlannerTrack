/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "min_curv_planner/track_io.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

using min_curv_planner::loadTrackFromWaypointsJson;

TEST(TrackIo, LoadsRacingCenterline)
{
	const auto track = loadTrackFromWaypointsJson(RACING_WAYPOINTS_JSON);
	EXPECT_GT(track.size(), 100);
	EXPECT_EQ(track.wRight.size(), track.size());
	EXPECT_EQ(track.wLeft.size(), track.size());
	EXPECT_TRUE((track.wRight.array() > 0.0).all());
	EXPECT_TRUE((track.wLeft.array() > 0.0).all());
	// first point of centerline_waypoints in the scenario file
	EXPECT_NEAR(track.xy(0, 0), -0.874336576639307, 1e-12);
	EXPECT_NEAR(track.xy(0, 1), -7.37913434268504, 1e-12);
}

TEST(TrackIo, MissingKeyThrows)
{
	EXPECT_THROW(loadTrackFromWaypointsJson(RACING_WAYPOINTS_JSON, "no_such_key"),
	    std::runtime_error);
}

TEST(TrackIo, MissingFileThrows)
{
	EXPECT_THROW(loadTrackFromWaypointsJson("/nonexistent/file.json"), std::runtime_error);
}
