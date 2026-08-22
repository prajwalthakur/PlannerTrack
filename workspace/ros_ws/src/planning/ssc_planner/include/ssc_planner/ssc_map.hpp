/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 * Adapted from EPSILON's planning::SscMap
 * (util/ssc_planner/inc/ssc_planner/ssc_map.h) -- see .docs/project.md M3/M4
 * for the deliberate deltas from that reference:
 *   - one grid only. EPSILON's p_3d_inflated_grid_/InflateObstacleGrid()
 *     (a second, margin-dilated copy of the grid) is dead in EPSILON's own
 *     live path (call site commented out) -- we get the same ego-footprint
 *     margin more cheaply, geometrically, in SscPlanner::projectAgentsToFrenet()
 *     before rasterization ever happens, so no second grid is needed here.
 *   - no static obstacles (FillStaticPart has no equivalent here -- this
 *     scenario has no rasterized road/curb occupancy).
 *   - no CorridorRelaxation / GetTimeCoveredCubeIndices (EPSILON's own call
 *     site for CorridorRelaxation is commented out -- skipped deliberately,
 *     not just by inheritance, see the plan doc's reasoning).
 *   - no GetInflationDirections / InflateCubeOnZNegAxis (both dead even in
 *     EPSILON's reference -- never called on the live path).
 *   - no per-behavior loop (EPSILON loops over MPDM candidate behaviors,
 *     building one corridor per behavior). This project has one fixed ego
 *     route, so one seed trajectory -> one corridor.
 *
 * STATUS: architecture only -- every non-trivial method is a stub (TODO),
 * filled in across step 4 (rasterization: constructSscMap/fillDynamicPart/
 * fillMapWithFsVehicleTraj) and step 5 (cube inflation: everything else).
 */
#pragma once

#include "planner_base/world_snapshot.hpp"
#include "project_utils/logger.hpp"
#include "project_utils/types.hpp"
#include "project_utils/unique_id.hpp"
#include "ssc_planner/ssc_grid.hpp"
#include "ssc_planner/ssc_map_config.hpp"

#include <array>
#include <unordered_map>
#include <vector>

namespace ssc_planner
{

// Mirrors EPSILON's common::AxisAlignedCubeNd<T, N_DIM>
// (core/common/inc/common/basics/shapes.h:131-140).
template <typename T, int N_DIM>
struct AxisAlignedCubeNd
{
	std::array<T, N_DIM> upperBound;
	std::array<T, N_DIM> lowerBound;
};

// Mirrors EPSILON's common::DrivingCube (semantics.h:792-795). seeds are
// grid-index coordinates (not world-frame), same convention as
// AxisAlignedCubeNd<int,3> -- EPSILON uses Vec3i here, but this project has
// no integer Eigen vector alias, and grid coordinates are already
// std::array<int,3> everywhere else (GridMapND), so we stay consistent with
// that instead of introducing a new type.
struct DrivingCube
{
	mt::vec_E<std::array<int, 3>> seeds;
	AxisAlignedCubeNd<int, 3> cube;
};

// Mirrors EPSILON's common::DrivingCorridor (semantics.h:797-801).
struct DrivingCorridor
{
	int id{0};
	bool isValid{false};
	mt::vec_E<DrivingCube> cubes;
};

// Mirrors EPSILON's common::SpatioTemporalSemanticCubeNd<N_DIM>
// (semantics.h:760-790) -- the final, metric (not grid-index) cube bounds
// handed to the Bezier QP (step 6). N_DIM=2 for (s, d); t is carried
// separately as [tLb, tUb].
template <int N_DIM>
struct SpatioTemporalSemanticCubeNd
{
	float tLb{0.0f}, tUb{1.0f};
	std::array<float, N_DIM> pLb{}, pUb{};
	std::array<float, N_DIM> vLb{}, vUb{};
	std::array<float, N_DIM> aLb{}, aUb{};
};

// Mirrors EPSILON's planning::SscMap (util/ssc_planner/inc/ssc_planner/ssc_map.h).
// Owns the occupancy grid + driving-corridor state for one planning cycle;
// SscPlanner owns one instance (mSscMap), analogous to EPSILON's
// SscPlanner::p_ssc_map_.
class SscMap
{
  public:
	using GridMap3D = ssc_planner::GridMapND<uint8_t, 3>;

	SscMap() = default;
	SscMap(const SscMapConfig & config, mpl::rclcpp_utils::Logger & logger);

	SscMapConfig config() const { return mConfig; }
	GridMap3D * grid() { return &mGrid; }

	mt::vec_E<DrivingCorridor> drivingCorridorVec() const { return mDrivingCorridorVec; }
	mt::vec_E<mt::vec_E<SpatioTemporalSemanticCubeNd<2>>> finalCorridorVec() const
	{
		return mFinalCorridorVec;
	}
	std::vector<int> ifCorridorValid() const { return mIfCorridorValid; }

	// TODO(step 5): set mStartTime.
	void setStartTime(const float & t);
	// TODO(step 5): set mInitialFs.
	void setInitialFs(const FrenetState & fs);
	// TODO(step 5): mirrors SscMap::UpdateMapOrigin -- re-origins the grid
	// around ori_fs (s_back_len behind ego's current s, d centered, t at
	// ego's current time).
	void updateMapOrigin(const FrenetState & oriFs);

	// TODO(step 4): mirrors SscMap::ConstructSscMap minus FillStaticPart (no
	// static obstacles here) -- clears the grid, then rasterizes every other
	// agent's already ego-margin-inflated FsVehicle trajectory into it
	// (fillDynamicPart).
	ErrorType constructSscMap(const std::unordered_map<UniqueId, mt::vec_E<FsVehicle>> & surVehicleTrajsFs);

	// TODO(step 5): mirrors SscMap::ConstructCorridorUsingInitialTrajectory --
	// seed generation from trajs (ego's own seed trajectory) + cube-chaining
	// loop (Algorithm 1), writing into mDrivingCorridorVec.
	ErrorType constructCorridorUsingInitialTrajectory(const mt::vec_E<FsVehicle> & trajs);

	// TODO(step 4/5 bookkeeping): mirrors SscMap::ClearGridMap.
	ErrorType clearGridMap();
	// TODO(step 4/5 bookkeeping): mirrors SscMap::ClearDrivingCorridor.
	ErrorType clearDrivingCorridor();
	// TODO(step 5): mirrors SscMap::GetFinalGlobalMetricCubesList -- converts
	// each cube's grid-index bounds to real (s,d,t) metric bounds, attaches
	// fixed velocity/accel bounds from mConfig.dynBounds.
	ErrorType getFinalGlobalMetricCubesList();
	// TODO(step 4/5 bookkeeping): mirrors SscMap::ResetSscMap -- clears
	// corridor + grid, then updateMapOrigin around iniFrenetState.
	ErrorType resetSscMap(const FrenetState & iniFrenetState);

  private:
	//  brute-force volume occupancy check, used once per candidate initial cube.
	bool checkIfCubeIsFree(const AxisAlignedCubeNd<int, 3> & cube) const;
	bool checkIfPlaneIsFreeOnXAxis(const AxisAlignedCubeNd<int, 3> & cube, const int& x) const;
	
	bool checkIfPlaneIsFreeOnYAxis(const AxisAlignedCubeNd<int, 3> & cube, const int& y) const;
	bool checkIfPlaneIsFreeOnZAxis(const AxisAlignedCubeNd<int, 3> & cube, const int& z) const;
	bool checkIfCubeContainsSeed(const AxisAlignedCubeNd<int, 3> & cubeA, const std::array<int, 3> & seed) const;

	// axis-aligned box spanning two seed grid-coordinates.
	ErrorType getInitialCubeUsingSeed(
	    const std::array<int, 3> & seed0, const std::array<int, 3> & seed1, AxisAlignedCubeNd<int, 3> * cube) const;


	ErrorType inflateCubeIn3dGrid(const InflateSteps & steps, AxisAlignedCubeNd<int, 3> * cube);

	
	bool inflateCubeOnXPosAxis(int nStep, AxisAlignedCubeNd<int, 3> * cube);
	
	bool inflateCubeOnXNegAxis(int nStep, AxisAlignedCubeNd<int, 3> * cube);
	
	bool inflateCubeOnYPosAxis(int nStep, AxisAlignedCubeNd<int, 3> * cube);
	
	bool inflateCubeOnYNegAxis(int nStep, AxisAlignedCubeNd<int, 3> * cube);
	
	bool inflateCubeOnZPosAxis(int nStep, AxisAlignedCubeNd<int, 3> * cube);

	// TODO(step 4): mirrors SscMap::FillDynamicPart -- loops surVehicleTrajsFs,
	// calls fillMapWithFsVehicleTraj per agent.
	ErrorType fillDynamicPart(const std::unordered_map<UniqueId, mt::vec_E<FsVehicle>> & surVehicleTrajsFs);
	// TODO(step 4): mirrors SscMap::FillMapWithFsVehicleTraj -- rasterizes one
	// agent's per-timestep FsVehicle.vertices polygon into that timestep's
	// (s,d) grid layer (our own scanline fill, no OpenCV -- see the plan
	// doc's reasoning for dropping cv::fillPoly).
	ErrorType fillMapWithFsVehicleTraj(const mt::vec_E<FsVehicle> & traj);

	SscMapConfig mConfig;
	GridMap3D mGrid;
	mpl::rclcpp_utils::Logger mLogger;

	float mStartTime{0.0f};
	FrenetState mInitialFs;

	mt::vec_E<DrivingCorridor> mDrivingCorridorVec;
	std::vector<int> mIfCorridorValid;
	mt::vec_E<mt::vec_E<SpatioTemporalSemanticCubeNd<2>>> mFinalCorridorVec;
};

}  // namespace ssc_planner
