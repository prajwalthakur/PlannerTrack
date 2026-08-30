/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 * Adapted from EPSILON's planning::SscMap -- see ssc_map.hpp for what's
 * deliberately different (single grid, no static obstacles, no
 * CorridorRelaxation, no z-neg inflation, no per-behavior loop).
 *
 * STATUS: architecture only. Every non-trivial method below is a stub,
 * filled in across step 4 (rasterization) and step 5 (cube inflation).
 */
#include "ssc_planner/ssc_map.hpp"

#include <algorithm>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace
{

//////////////////////////////////////////////////////////////////////////

std::array<int, 3> toSizeArray(const ssc_planner::SscMapConfig & config)
{
	return {config.mapSizeX, config.mapSizeY, config.mapSizeZ};
}

//////////////////////////////////////////////////////////////////////////

std::array<float, 3> toResolutionArray(const ssc_planner::SscMapConfig & config)
{
	return {static_cast<float>(config.mapReslX), static_cast<float>(config.mapReslY),
	    static_cast<float>(config.mapReslZ)};
}
}  // namespace

namespace ssc_planner
{

//////////////////////////////////////////////////////////////////////////

SscMap::SscMap(const SscMapConfig & config, mpl::rclcpp_utils::Logger & logger)
    : mConfig(config),
      mGrid(toSizeArray(config), toResolutionArray(config),

//////////////////////////////////////////////////////////////////////////

          std::array<std::string, 3>{"s", "d", "t"}),
      mLogger(logger)
{
	mLogger.info("[SscMap] grid size (s,d,t) = (%d, %d, %d), resolution = (%.3f, %.3f, %.3f)",
	    mConfig.mapSizeX, mConfig.mapSizeY, mConfig.mapSizeZ, mConfig.mapReslX, mConfig.mapReslY,
	    mConfig.mapReslZ);
}

//////////////////////////////////////////////////////////////////////////

void SscMap::setStartTime(const float & t)
{
	mStartTime = t;
}

//////////////////////////////////////////////////////////////////////////

void SscMap::setInitialFs(const FrenetState & fs)
{
	mInitialFs = fs;
}

//////////////////////////////////////////////////////////////////////////

void SscMap::updateMapOrigin(const FrenetState & fs)
{
	mInitialFs = fs;

	std::array<float, 3> mapOrigin;
	mapOrigin[0] = static_cast<float>(fs.s - mConfig.sBackLen);
	// Centers the d-window around d=0 (the reference lane itself) --
	// independent of fs.d, since d is already relative to mRefLane.
	mapOrigin[1] = static_cast<float>(-1.0 * (mConfig.mapSizeY - 1) * mConfig.mapReslY / 2.0);
	mapOrigin[2] = fs.t;

	mGrid.set_origin(mapOrigin);
}

//////////////////////////////////////////////////////////////////////////

ErrorType SscMap::constructSscMap(
    const std::unordered_map<UniqueId, mt::vec_E<FsVehicle>> & surVehicleTrajsFs)
{
	clearGridMap();
	// FillStaticPart(obstacle_grids); -- no static obstacles for now
	fillDynamicPart(surVehicleTrajsFs);
	return kSuccess;
}

//////////////////////////////////////////////////////////////////////////

ErrorType SscMap::constructCorridorUsingInitialTrajectory(const mt::vec_E<FsVehicle> & egoFTraj)
{
	//  Stage I: Get seeds
	mt::vec_E<std::array<int, 3>> trajSeeds;
	bool firstSeedDetermined = false;
	for (const auto & fv : egoFTraj) {
		if (!firstSeedDetermined) {
			float s_0 = mInitialFs.s;
			float d_0 = mInitialFs.d;
			float t_0 = mInitialFs.t;
			std::array<float, 3> p_w_0 = {s_0, d_0, t_0};
			auto coord_0 = mGrid.GetCoordUsingGlobalPosition(p_w_0);

			float s_1 = fv.frenetState.s;
			float d_1 = fv.frenetState.d;
			float t_1 = fv.frenetState.t;
			std::array<float, 3> p_w_1 = {s_1, d_1, t_1};
			auto coord_1 = mGrid.GetCoordUsingGlobalPosition(p_w_1);
			// * remove the states out of range
			if (!mGrid.CheckCoordInRange(coord_1)) {
				continue;
			}
			// earlier than start time
			if (coord_1[2] <= 0) {
				continue;
			}

			firstSeedDetermined = true;
			trajSeeds.push_back(std::array<int, 3>{coord_0[0], coord_0[1], coord_0[2]});
			trajSeeds.push_back(std::array<int, 3>{coord_1[0], coord_1[1], coord_1[2]});
		} else {
			float s_k = fv.frenetState.s;
			float d_k = fv.frenetState.d;
			float t_k = fv.frenetState.t;
			std::array<float, 3> p_w = {s_k, d_k, t_k};
			auto coord = mGrid.GetCoordUsingGlobalPosition(p_w);
			// * remove the states out of range
			if (!mGrid.CheckCoordInRange(coord)) {
				continue;
			}
			trajSeeds.push_back(std::array<int, 3>{coord[0], coord[1], coord[2]});
		}
	}

	// Stage II: Inflate cubes
	DrivingCorridor drivingCorridor;
	bool isValid = true;
	auto seedNum = static_cast<int>(trajSeeds.size());
	if (seedNum < 2) {
		drivingCorridor.isValid = false;
		mDrivingCorridorVec.push_back(drivingCorridor);
		return kWrongStatus;
	}
	for (int i = 0; i < seedNum; ++i) {
		if (i == 0) {
			AxisAlignedCubeNd<int, 3> cube;
			getInitialCubeUsingSeed(trajSeeds[i], trajSeeds[i + 1], &cube);
			if (!checkIfCubeIsFree(cube)) {
				// Convert the failing seed's grid-index coords to real
				// metric (s,d,t) so the error names WHERE this actually is,
				// not just which index into the (already-filtered) seed list.
				float sMetric = 0, dMetric = 0, tMetric = 0;
				mGrid.GetGlobalMetricUsingCoordOnSingleDim(trajSeeds[i][0], 0, &sMetric);
				mGrid.GetGlobalMetricUsingCoordOnSingleDim(trajSeeds[i][1], 1, &dMetric);
				mGrid.GetGlobalMetricUsingCoordOnSingleDim(trajSeeds[i][2], 2, &tMetric);
				mLogger.error(
				    "[SccMap]: Initial cube is not free, seed id: %d, at s=%.2f d=%.2f t=%.2f", i,
				    sMetric, dMetric, tMetric);
				DrivingCube drivingCube;
				drivingCube.cube = cube;
				drivingCube.seeds.push_back(trajSeeds[i]);
				drivingCube.seeds.push_back(trajSeeds[i + 1]);
				drivingCorridor.cubes.push_back(drivingCube);

				drivingCorridor.isValid = false;
				mDrivingCorridorVec.push_back(drivingCorridor);
				isValid = false;
				break;
			}
			inflateCubeIn3dGrid(mConfig.inflSteps, &cube);
			DrivingCube drivingCube;
			drivingCube.cube = cube;
			drivingCube.seeds.push_back(trajSeeds[i]);
			drivingCorridor.cubes.push_back(drivingCube);
		} else {
			if (checkIfCubeContainsSeed(drivingCorridor.cubes.back().cube, trajSeeds[i])) {
				drivingCorridor.cubes.back().seeds.push_back(trajSeeds[i]);
				continue;
			} else {
				// ~ Get the last seed in cube
				const auto seedR = drivingCorridor.cubes.back().seeds.back();
				drivingCorridor.cubes.back().seeds.pop_back();
				// ~ Cut cube on time axis
				drivingCorridor.cubes.back().cube.upperBound[2] = seedR[2];
				i = i - 1;

				AxisAlignedCubeNd<int, 3> cube;
				getInitialCubeUsingSeed(trajSeeds[i], trajSeeds[i + 1], &cube);
				if (!checkIfCubeIsFree(cube)) {
					float sMetric = 0, dMetric = 0, tMetric = 0;
					mGrid.GetGlobalMetricUsingCoordOnSingleDim(trajSeeds[i][0], 0, &sMetric);
					mGrid.GetGlobalMetricUsingCoordOnSingleDim(trajSeeds[i][1], 1, &dMetric);
					mGrid.GetGlobalMetricUsingCoordOnSingleDim(trajSeeds[i][2], 2, &tMetric);
					mLogger.error(
					    "[SccMap]: Initial cube is not free, seed id: %d, at s=%.2f d=%.2f t=%.2f", i,
					    sMetric, dMetric, tMetric);
					DrivingCube drivingCube;
					drivingCube.cube = cube;
					drivingCube.seeds.push_back(trajSeeds[i]);
					drivingCube.seeds.push_back(trajSeeds[i + 1]);
					drivingCorridor.cubes.push_back(drivingCube);
					drivingCorridor.isValid = false;
					mDrivingCorridorVec.push_back(drivingCorridor);
					isValid = false;
					break;
				}

				inflateCubeIn3dGrid(mConfig.inflSteps, &cube);
				DrivingCube drivingCube;
				drivingCube.cube = cube;
				drivingCube.seeds.push_back(trajSeeds[i]);
				drivingCorridor.cubes.push_back(drivingCube);
			}
		}
	}
	if (isValid) {
		// CorridorRelaxation skipped deliberately here -- see ssc_map.hpp's
		// top-of-file comment for the reasoning.
		drivingCorridor.cubes.back().cube.upperBound[2] = trajSeeds.back()[2];
		drivingCorridor.isValid = true;
		mDrivingCorridorVec.push_back(drivingCorridor);
	}
	return kSuccess;
}

//////////////////////////////////////////////////////////////////////////

ErrorType SscMap::clearGridMap()
{
	mGrid.clear_data();
	return kSuccess;
}

//////////////////////////////////////////////////////////////////////////

ErrorType SscMap::clearDrivingCorridor()
{
	mDrivingCorridorVec.clear();
	return kSuccess;
}

//////////////////////////////////////////////////////////////////////////

ErrorType SscMap::getFinalGlobalMetricCubesList()
{
	mFinalCorridorVec.clear();
	mIfCorridorValid.clear();
	for (const auto & corridor : mDrivingCorridorVec) {
		mt::vec_E<SpatioTemporalSemanticCubeNd<2>> cubes;
		mIfCorridorValid.push_back(corridor.isValid ? 1 : 0);
		// Convert cube bounds to metric even when the corridor is invalid --
		// an infeasible corridor still has the ONE cube that failed
		// checkIfCubeIsFree (constructCorridorUsingInitialTrajectory pushes
		// it before marking isValid=false), and visualizing exactly where/
		// how it failed is more useful than publishing nothing. Downstream,
		// computeBezierTrajectory() already gates the actual QP solve on
		// ifCorridorValid()[0]==0 independent of whether cubes is empty, so
		// populating it here can't let an invalid corridor reach the solver.
		for (int k = 0; k < static_cast<int>(corridor.cubes.size()); ++k) {
			SpatioTemporalSemanticCubeNd<2> cube;
			float xLb, xUb;
			float yLb, yUb;
			float zLb, zUb;

			mGrid.GetGlobalMetricUsingCoordOnSingleDim(corridor.cubes[k].cube.lowerBound[0], 0, &xLb);
			mGrid.GetGlobalMetricUsingCoordOnSingleDim(corridor.cubes[k].cube.upperBound[0], 0, &xUb);
			mGrid.GetGlobalMetricUsingCoordOnSingleDim(corridor.cubes[k].cube.lowerBound[1], 1, &yLb);
			mGrid.GetGlobalMetricUsingCoordOnSingleDim(corridor.cubes[k].cube.upperBound[1], 1, &yUb);
			mGrid.GetGlobalMetricUsingCoordOnSingleDim(corridor.cubes[k].cube.lowerBound[2], 2, &zLb);
			mGrid.GetGlobalMetricUsingCoordOnSingleDim(corridor.cubes[k].cube.upperBound[2], 2, &zUb);

			cube.tLb = zLb;
			cube.tUb = zUb;

			// s: min/max longitudinal vel and accel/decel are separate,
			// independently-tuned bounds -- forward and backward motion
			// along s aren't symmetric for a car (see the v_lb[1]
			// discussion a few turns back).
			cube.pLb[0] = xLb;
			cube.pUb[0] = xUb;
			cube.vLb[0] = static_cast<float>(mConfig.dynBounds.minLonVel);
			cube.vUb[0] = static_cast<float>(mConfig.dynBounds.maxLonVel);
			cube.aLb[0] = static_cast<float>(mConfig.dynBounds.maxLonDec);
			cube.aUb[0] = static_cast<float>(mConfig.dynBounds.maxLonAcc);

			// d: lateral vel/accel are symmetric (no inherent left/right
			// preference), so a single magnitude is negated for both signs.
			cube.pLb[1] = yLb;
			cube.pUb[1] = yUb;
			cube.vLb[1] = static_cast<float>(-mConfig.dynBounds.maxLatVel);
			cube.vUb[1] = static_cast<float>(mConfig.dynBounds.maxLatVel);
			cube.aLb[1] = static_cast<float>(-mConfig.dynBounds.maxLatAcc);
			cube.aUb[1] = static_cast<float>(mConfig.dynBounds.maxLatAcc);

			// The first cube of a VALID corridor must actually contain
			// ego's real (un-rounded) current d -- not structurally
			// guaranteed the way s is, since d's bounds come from
			// occupancy-driven inflation, not directly from mInitialFs.d.
			// Catches a discretization edge case that would otherwise
			// silently produce an unusable corridor. Only meaningful for a
			// valid corridor -- an invalid one's only cube is the one that
			// already failed checkIfCubeIsFree, so this invariant isn't
			// expected to hold there and shouldn't block visualizing it.
			if (corridor.isValid && k == 0) {
				if (yLb > mInitialFs.d || yUb < mInitialFs.d) {
					mLogger.error(
					    "[SccMap]: Initial state out of bound d: %.3f, lb: %.3f, ub: %.3f",
					    mInitialFs.d, yLb, yUb);
					return kWrongStatus;
				}
			}

			cubes.push_back(cube);
		}
		mFinalCorridorVec.push_back(cubes);
	}

	return kSuccess;
}

//////////////////////////////////////////////////////////////////////////

ErrorType SscMap::resetSscMap(const FrenetState & initFrenetState)
{
	clearDrivingCorridor();
	clearGridMap();
	setInitialFs(initFrenetState);
	setStartTime(initFrenetState.t);
	updateMapOrigin(mInitialFs);
	return kSuccess;
}

//////////////////////////////////////////////////////////////////////////

bool SscMap::checkIfCubeIsFree(const AxisAlignedCubeNd<int, 3> & cube) const
{
	int f0_min = cube.lowerBound[0];
	int f0_max = cube.upperBound[0];
	int f1_min = cube.lowerBound[1];
	int f1_max = cube.upperBound[1];
	int f2_min = cube.lowerBound[2];
	int f2_max = cube.upperBound[2];
	int i, j, k;
	std::array<int, 3> coord;
	bool is_free;
	for (i = f0_min; i <= f0_max; ++i) {
		for (j = f1_min; j <= f1_max; ++j) {
			for (k = f2_min; k <= f2_max; ++k) {
				coord = {i, j, k};
				mGrid.CheckIfEqualUsingCoordinate(coord, 0, &is_free);
				if (!is_free) {
					return false;
				}
			}
		}
	}
	return true;
}

//////////////////////////////////////////////////////////////////////////

bool SscMap::checkIfPlaneIsFreeOnXAxis(const AxisAlignedCubeNd<int, 3> & cube, const int & x) const
{
	int f0_min = cube.lowerBound[1];
	int f0_max = cube.upperBound[1];
	int f1_min = cube.lowerBound[2];
	int f1_max = cube.upperBound[2];
	std::array<int, 3> coord;
	bool is_free;
	for (int i = f0_min; i <= f0_max; ++i) {
		for (int j = f1_min; j <= f1_max; ++j) {
			coord = {x, i, j};
			mGrid.CheckIfEqualUsingCoordinate(coord, 0, &is_free);
			if (!is_free) {
				return false;
			}
		}
	}
	return true;
}

//////////////////////////////////////////////////////////////////////////

bool SscMap::checkIfPlaneIsFreeOnYAxis(const AxisAlignedCubeNd<int, 3> & cube, const int & y) const
{
	int f0_min = cube.lowerBound[0];
	int f0_max = cube.upperBound[0];
	int f1_min = cube.lowerBound[2];
	int f1_max = cube.upperBound[2];
	std::array<int, 3> coord;
	bool is_free;
	for (int i = f0_min; i <= f0_max; ++i) {
		for (int j = f1_min; j <= f1_max; ++j) {
			coord = {i, y, j};
			mGrid.CheckIfEqualUsingCoordinate(coord, 0, &is_free);
			if (!is_free) {
				return false;
			}
		}
	}
	return true;
}

//////////////////////////////////////////////////////////////////////////

bool SscMap::checkIfPlaneIsFreeOnZAxis(const AxisAlignedCubeNd<int, 3> & cube, const int & z) const
{
	int f0_min = cube.lowerBound[0];
	int f0_max = cube.upperBound[0];
	int f1_min = cube.lowerBound[1];
	int f1_max = cube.upperBound[1];
	std::array<int, 3> coord;
	bool is_free;
	for (int i = f0_min; i <= f0_max; ++i) {
		for (int j = f1_min; j <= f1_max; ++j) {
			coord = {i, j, z};
			mGrid.CheckIfEqualUsingCoordinate(coord, 0, &is_free);
			if (!is_free) {
				return false;
			}
		}
	}
	return true;
}

//////////////////////////////////////////////////////////////////////////

bool SscMap::checkIfCubeContainsSeed(
    const AxisAlignedCubeNd<int, 3> & cube_a, const std::array<int, 3> & seed) const
{
	for (int i = 0; i < 3; ++i) {
		if (cube_a.lowerBound[i] > seed[i] || cube_a.upperBound[i] < seed[i]) {
			return false;
		}
	}
	return true;
}

//////////////////////////////////////////////////////////////////////////

ErrorType SscMap::getInitialCubeUsingSeed(const std::array<int, 3> & seed_0,
    const std::array<int, 3> & seed_1, AxisAlignedCubeNd<int, 3> * cube) const
{
	std::array<int, 3> lb;
	std::array<int, 3> ub;
	lb[0] = std::min(seed_0[0], seed_1[0]);  // s_min
	lb[1] = std::min(seed_0[1], seed_1[1]);  // d_min
	lb[2] = std::min(seed_0[2], seed_1[2]);  // t_min
	ub[0] = std::max(seed_0[0], seed_1[0]);  // s_max
	ub[1] = std::max(seed_0[1], seed_1[1]);  // d_max
	ub[2] = std::max(seed_0[2], seed_1[2]);  // t_max

	cube->upperBound = ub;
	cube->lowerBound = lb;

	return kSuccess;
}

//////////////////////////////////////////////////////////////////////////

ErrorType SscMap::inflateCubeIn3dGrid(const InflateSteps & steps, AxisAlignedCubeNd<int, 3> * cube)
{
	// if expansion is finished in respective directions
	bool xPFinish = false, xNFinish = false, yPFinish = false, yNFinish = false, zPFinish = false;
	// worst case upper bound on t
	int t_max_grids = cube->lowerBound[2] + mConfig.maxGridsAlongTime;
	// max duration that can be expanded
	float t = t_max_grids * mGrid.dims_resolution(2);
	// max lon acc
	float a_max = mConfig.dynBounds.maxLonAcc;
	// min lon acc
	float a_min = mConfig.dynBounds.maxLonDec;
	//
	float d_comp = mInitialFs.sDot * 1.0f;
	// x_f = x_u + vdot*t + 0.5*a*t*t + margin
	float s_u = mInitialFs.s + mInitialFs.sDot * t + 0.5f * a_max * t * t + d_comp;
	float s_l = mInitialFs.s + mInitialFs.sDot * t + 0.5f * a_min * t * t - d_comp;

	int s_idx_u, s_idx_l;
	mGrid.GetCoordUsingGlobalMetricOnSingleDim(s_u, 0, &s_idx_u);
	mGrid.GetCoordUsingGlobalMetricOnSingleDim(s_l, 0, &s_idx_l);

	// max s-negative is mConfig.sBackLen / 2.0,
	// index sitting halfway between the back edge and ego's current position.
	s_idx_l = std::max(s_idx_l, static_cast<int>((mConfig.sBackLen / 2.0) / mConfig.mapReslX));

	while (!(xPFinish && xNFinish && yPFinish && yNFinish)) {
		if (!xPFinish) xPFinish = inflateCubeOnXPosAxis(steps.xP, cube);
		if (!xNFinish) xNFinish = inflateCubeOnXNegAxis(steps.xN, cube);
		if (!yPFinish) yPFinish = inflateCubeOnYPosAxis(steps.yP, cube);
		if (!yNFinish) yNFinish = inflateCubeOnYNegAxis(steps.yN, cube);

		if (cube->upperBound[0] >= s_idx_u) xPFinish = true;
		if (cube->lowerBound[0] <= s_idx_l) xNFinish = true;
	}
	// no need to expand in t-neg
	while (!zPFinish) {
		zPFinish = inflateCubeOnZPosAxis(steps.zP, cube);

		if (cube->upperBound[2] - cube->lowerBound[2] >= mConfig.maxGridsAlongTime) {
			zPFinish = true;
		}
	}
	return kSuccess;
}

//////////////////////////////////////////////////////////////////////////

bool SscMap::inflateCubeOnXPosAxis(int nStep, AxisAlignedCubeNd<int, 3> * cube)
{
	for (int i = 0; i < nStep; ++i) {
		int x = cube->upperBound[0] + 1;
		if (!mGrid.CheckCoordInRangeOnSingleDim(x, 0)) {
			return true;
		} else {
			if (checkIfPlaneIsFreeOnXAxis(*cube, x)) {
				// The plane in 3D obstacle grid is free
				cube->upperBound[0] = x;
			} else {
				// The plane in 3D obstacle grid is not free, finish
				return true;
			}
		}
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////

bool SscMap::inflateCubeOnXNegAxis(int nStep, AxisAlignedCubeNd<int, 3> * cube)
{
	for (int i = 0; i < nStep; ++i) {
		int x = cube->lowerBound[0] - 1;
		if (!mGrid.CheckCoordInRangeOnSingleDim(x, 0)) {
			return true;
		} else {
			if (checkIfPlaneIsFreeOnXAxis(*cube, x)) {
				cube->lowerBound[0] = x;
			} else {
				return true;
			}
		}
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////

bool SscMap::inflateCubeOnYPosAxis(int nStep, AxisAlignedCubeNd<int, 3> * cube)
{
	for (int i = 0; i < nStep; ++i) {
		int y = cube->upperBound[1] + 1;
		if (!mGrid.CheckCoordInRangeOnSingleDim(y, 1)) {
			return true;
		} else {
			if (checkIfPlaneIsFreeOnYAxis(*cube, y)) {
				cube->upperBound[1] = y;
			} else {
				return true;
			}
		}
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////

bool SscMap::inflateCubeOnYNegAxis(int nStep, AxisAlignedCubeNd<int, 3> * cube)
{
	for (int i = 0; i < nStep; ++i) {
		int y = cube->lowerBound[1] - 1;
		if (!mGrid.CheckCoordInRangeOnSingleDim(y, 1)) {
			return true;
		} else {
			if (checkIfPlaneIsFreeOnYAxis(*cube, y)) {
				cube->lowerBound[1] = y;
			} else {
				return true;
			}
		}
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////

bool SscMap::inflateCubeOnZPosAxis(int nStep, AxisAlignedCubeNd<int, 3> * cube)
{
	for (int i = 0; i < nStep; ++i) {
		int z = cube->upperBound[2] + 1;
		if (!mGrid.CheckCoordInRangeOnSingleDim(z, 2)) {
			return true;
		} else {
			if (checkIfPlaneIsFreeOnZAxis(*cube, z)) {
				cube->upperBound[2] = z;
			} else {
				return true;
			}
		}
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////

ErrorType SscMap::fillDynamicPart(
    const std::unordered_map<UniqueId, mt::vec_E<FsVehicle>> & sur_vehicle_trajs_fs)
{
	for (auto it = sur_vehicle_trajs_fs.begin(); it != sur_vehicle_trajs_fs.end(); ++it) {
		auto status = fillMapWithFsVehicleTraj(it->second);
		if (status == kWrongStatus) {
			mLogger.error("[SscMap] wrong status for vehicle id %d",it->first.value());
			return kWrongStatus;
		}
	}
	return kSuccess;
}

//////////////////////////////////////////////////////////////////////////

ErrorType SscMap::fillMapWithFsVehicleTraj(const mt::vec_E<FsVehicle> & traj)
{
	if (traj.size() == 0) {
		mLogger.error("[SscMap]: Trajectory is empty");
		return kWrongStatus;
	}
	for (int i = 0; i < static_cast<int>(traj.size()); ++i) {
		bool isValid = true;
		// check if all the vertices are within the range of the
		// ego-ref frenet lane
		for (const auto& v : traj[i].vertices) {
			if (v(0) <= 0) {
				isValid = false;
				break;
			}
		}
		if (!isValid) {
			continue;
		}
		float z = traj[i].frenetState.t;
		int tIdx = 0;
		std::vector<cv::Point2i> vCoordCv;
		std::array<float, 3> pointWorld;
		const int w = mGrid.dims_size()[0];
		const int h = mGrid.dims_size()[1];
		for (const auto& v : traj[i].vertices) {
			pointWorld = {v(0), v(1), z};
			auto coord = mGrid.GetCoordUsingGlobalPosition(pointWorld);
			tIdx = coord[2];
			// Time axis: an out-of-range t means no grid layer exists to
			// write into at all, so skipping the whole point here is
			// correct (all 4 vertices share the same t/tIdx -- it comes
			// from traj[i].frenetState.t, a per-point value, not
			// per-vertex).
			if (!mGrid.CheckCoordInRangeOnSingleDim(tIdx, 2)) {
				isValid = false;
				break;
			}
			// s/d axes: clamp into range instead of rejecting the whole
			// point. Previously used the same "any out-of-range vertex
			// drops the whole point" rule as the time axis -- that
			// silently made any agent whose inflated footprint ever
			// poked past the s/d grid edges invisible to corridor
			// construction for its ENTIRE trajectory, not just the
			// out-of-range instant (confirmed live: agent_2 crossing
			// through d up to +0.50 produced zero narrowing anywhere in
			// the corridor, d staying the full unconstrained
			// [-0.625,0.625] the whole route -- see
			// project_docs/ssc_planner_status.md). A footprint that
			// straddles the grid boundary should rasterize wherever it
			// overlaps the grid, not vanish entirely.
			const int sIdx = std::clamp(coord[0], 0, w - 1);
			const int dIdx = std::clamp(coord[1], 0, h - 1);
			vCoordCv.push_back(cv::Point2i(sIdx, dIdx));
		}
		if (!isValid) {
			continue;
		}
		std::vector<std::vector<cv::Point2i>> vvCoordCv{vCoordCv};
		int layerOffset = tIdx * w * h;
		// get the layer at layerOffset
		// now this a  layer at time index tIdx, with size hxw
		cv::Mat layerMat(
		    h, w, CV_MAKETYPE(cv::DataType<uint8_t>::type, 1), mGrid.get_data_ptr() + layerOffset);
		cv::fillPoly(layerMat, vvCoordCv, 100);
	}
	return kSuccess;
}

}  // namespace ssc_planner
