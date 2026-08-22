/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include <string>

namespace ssc_planner
{

//Map Config (x=s longitudinal, y=d lateral,z=t time).
struct InflateSteps
{
	int xP{5}, xN{5}, yP{5}, yN{5}, zP{1}, zN{1};
};

struct DynamicBounds
{
	double maxLonVel{2.0};
	double minLonVel{0.0};
	double maxLonAcc{2.0};
	double maxLonDec{-3.0};
	double maxLatVel{1.0};
	double maxLatAcc{1.0};
};

// Ego-footprint inflation mode for other agents' rasterized vertices, see
// projectAgentsToFrenet() -- mutually exclusive, simple checked first.
// Neither set means no inflation (other agents' raw footprint only).
struct EgoFootprintInflationConfig
{
	bool simple{true};
	bool minkowski{false};
};

struct SscMapConfig
{
	int mapSizeX{0}, mapSizeY{0}, mapSizeZ{0};
	double mapReslX{0.0}, mapReslY{0.0}, mapReslZ{0.0};
	double sBackLen{1.0};
	DynamicBounds dynBounds;
	int maxGridsAlongTime{2};
	InflateSteps inflSteps;
	EgoFootprintInflationConfig egoInflation;
};

// Loads this planner's own dedicated config
// (scenarios/intersection/params/ssc_planner.yaml, NOT the scenario's
// params.yaml -- self-contained, doesn't reach into the scenario's own
// corridor/Lanes definitions): "ssc_map"'s map_resolution (x,y,z) and its
// own lane_width/arm_length, then derives mapSizeY from
// lane_width / mapReslY -- the bound on how far a generated corridor can
// deviate laterally from the reference lane. d is measured relative to
// mRefLane (ego's own already-curving seed path), so the turn's sweep is
// already captured by s moving along that curve -- d only needs to cover
// ego's own drivable strip at any point along the path, i.e. one
// lane_width, not the full road_width (that would let the corridor
// structurally extend into the oncoming lane). This project doesn't
// rasterize static obstacles (no curb/road-edge occupancy), unlike the SSC
// reference, which relies on real occupancy to keep a corridor on the road
// despite using an arbitrary fixed map_size_y (see EPSILON's
// ssc_config.pb.txt: map_size_y=71 @ 0.2m resolution = a hand-picked 14.2m,
// unrelated to any real road width) -- so here, deriving mapSizeY from the
// actual lane width does the job occupancy does for them.
//
// mapSizeX gets a generous arm-to-arm estimate from lane_width/arm_length.
// sBackLen, dynBounds, maxGridsAlongTime, and inflSteps are all read
// directly from ssc_planner.yaml (dynBounds sourced from this scenario's
// real vehicle limits in pure_pursuit_controller.yaml, kept as a
// conservative fraction of the physical ceiling -- see ssc_planner.yaml's
// comments for the reasoning).
SscMapConfig loadSscMapConfig(const std::string & sscConfigFilePath);

}  // namespace ssc_planner
