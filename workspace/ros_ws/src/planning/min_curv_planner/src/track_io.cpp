/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "min_curv_planner/track_io.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace min_curv_planner
{

Track loadTrackFromWaypointsJson(const std::string & filePath, const std::string & key)
{
	YAML::Node root;
	try {
		root = YAML::LoadFile(filePath);
	} catch (const YAML::Exception & e) {
		throw std::runtime_error("loadTrackFromWaypointsJson: cannot read " + filePath + ": " +
		                         e.what());
	}

	const YAML::Node wpnts = root[key]["wpnts"];
	if (!wpnts || !wpnts.IsSequence()) {
		throw std::runtime_error(
		    "loadTrackFromWaypointsJson: no '" + key + ".wpnts' list in " + filePath);
	}

	int n = static_cast<int>(wpnts.size());
	Track track;
	track.xy.resize(n, 2);
	track.wRight.resize(n);
	track.wLeft.resize(n);
	for (int i = 0; i < n; ++i) {
		const YAML::Node & w = wpnts[i];
		track.xy(i, 0) = w["x_m"].as<double>();
		track.xy(i, 1) = w["y_m"].as<double>();
		track.wRight(i) = w["d_right"].as<double>();
		track.wLeft(i) = w["d_left"].as<double>();
	}

	// Closed tracks are stored without the repeated closing point.
	if (n > 1 && (track.xy.row(0) - track.xy.row(n - 1)).norm() < 1e-4) {
		--n;
		track.xy.conservativeResize(n, 2);
		track.wRight.conservativeResize(n);
		track.wLeft.conservativeResize(n);
	}
	if (n < 3) {
		throw std::runtime_error(
		    "loadTrackFromWaypointsJson: '" + key + "' has fewer than 3 points in " + filePath);
	}
	return track;
}

}  // namespace min_curv_planner
