#include "mppi_controller/utils/nav2_utils/feasible_path_handler.hpp"

/** \file
 * \brief \ref FeasiblePathHandler implementation: plan clipping/pruning,
 * frame transforms, and the racing/looping-lap lookahead window.
 */

namespace controller::mppi_controller::utils
{

// using mppi_utils::euclidean_distance;

//////////////////////////////////////////////////////////////////////////

void FeasiblePathHandler::initialize(Parameters * parameters, Logger & logger,
    const std::shared_ptr<CostMapRos> costMapRos, const std::string & parentName,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf)
{
	mParameters = parameters;
	mLogger = logger;
	mCostmapRos = costMapRos;
	mTfBuffer = tf;
	mParentName = parentName;
	mName = name;
	// Transform tolerance
	mTransformTolerance = mCostmapRos->getTransformTolerance();

	// Fully qualified parameter namespace
	std::string full_name = mName;
	if (!mParentName.empty()) {
		full_name = mParentName + "." + mName;
	}
	auto getParam = mParameters->getParamGetter(full_name);

	// Parameters
	getParam(mRejectUnitPath, "reject_unit_path", false);
	getParam(mMaxRobotPoseSearchDist, "max_robot_pose_search_dist", getCostmapMaxExtent());
	getParam(mPruneDistance, "prune_distance", 2.0);
	getParam(mEnforcePathInversion, "enforce_path_inversion", false);
	getParam(mEnforcePathRotation, "enforce_path_rotation", false);
	getParam(mInversionXYTolerance, "inversion_xy_tolerance", 0.2);
	getParam(mInversionYawTolerance, "inversion_yaw_tolerance", 0.4);
	getParam(mMinimumRotationAngle, "minimum_rotation_angle", 0.785);

	// Validation
	if (mMaxRobotPoseSearchDist < 0.0) {
		mLogger.warn("Max robot search distance is negative, setting to max.");
		mMaxRobotPoseSearchDist = std::numeric_limits<double>::max();
	}

	// Constraint locale
	mConstraintLocale = 0u;

	// Rotation enforcement check
	if (!mEnforcePathRotation) {
		mMinimumRotationAngle = 0.0;
	}

	mLogger.info("full name %s, feasible path handler initialized, prune distance %.3f",
	    full_name.c_str(), mPruneDistance);
}

//////////////////////////////////////////////////////////////////////////

double FeasiblePathHandler::getCostmapMaxExtent() const
{
	const double maxCostMapDimMeters = std::max(mCostmapRos->getCostmap()->getSizeInMetersX(),
	    mCostmapRos->getCostmap()->getSizeInMetersY());
	return maxCostMapDimMeters / 2.0;
}

//////////////////////////////////////////////////////////////////////////

void FeasiblePathHandler::prunePlan(nav_msgs::msg::Path & plan, const mppi_utils::PathIterator end)
{
	plan.poses.erase(plan.poses.begin(), end);
}

//////////////////////////////////////////////////////////////////////////

bool FeasiblePathHandler::isWithinInversionTolerances(
    const geometry_msgs::msg::PoseStamped & robotPose)
{
	const auto lastPose = mGlobalPlanUpToConstraint.poses.back();
	float distance = hypotf(robotPose.pose.position.x - lastPose.pose.position.x,
	    robotPose.pose.position.y - lastPose.pose.position.y);
	float angularDistance = mppi_utils::shortestAngularDistance(
	    tf2::getYaw(robotPose.pose.orientation), tf2::getYaw(lastPose.pose.orientation));

	return (distance <= mInversionXYTolerance) && (fabs(angularDistance) <= mInversionYawTolerance);
}

//////////////////////////////////////////////////////////////////////////

void FeasiblePathHandler::setPlan(const nav_msgs::msg::Path & path)
{
	// TODO: add lock (check nav2)
	mGlobalPlanOriginal = path;
	mGlobalPlanUpToConstraint = mGlobalPlanOriginal;
	mGlobalPlan = mGlobalPlanOriginal;
	if (mEnforcePathInversion || mEnforcePathRotation) {
		mConstraintLocale = mppi_utils::removePosesAfterFirstConstraint(
		    mGlobalPlanUpToConstraint, mEnforcePathInversion, mMinimumRotationAngle);
	}
	mPrevClosestPoint = 0;
	mLookAheadNumPoints = 40;
	mRacingPathPadded = false;
}

//////////////////////////////////////////////////////////////////////////

nav_msgs::msg::Path FeasiblePathHandler::setPathForRacing(
    const geometry_msgs::msg::PoseStamped & pose)
{
	const size_t numPoints = mGlobalPlanOriginal.poses.size();
	const size_t windowSize = std::min(mLookAheadNumPoints, numPoints);

	if (!mRacingPathPadded) {
		mGlobalPlanPadded = mGlobalPlanOriginal;
		mGlobalPlanPadded.poses.insert(mGlobalPlanPadded.poses.end(),
		    mGlobalPlanOriginal.poses.begin(), mGlobalPlanOriginal.poses.begin() + windowSize);
		mRacingPathPadded = true;
	}

	mGlobalPose = transformToGlobalPlanFrame(pose);

	auto closestPoint = mppi_utils::minBy(mGlobalPlanPadded.poses.begin() + mPrevClosestPoint,
	    mGlobalPlanPadded.poses.begin() + mPrevClosestPoint + windowSize,
	    [this](const geometry_msgs::msg::PoseStamped & ps) {
		    return mppi_utils::euclidean_distance(mGlobalPose, ps);
	    });


	mPrevClosestPoint = static_cast<size_t>(
	    std::distance(mGlobalPlanPadded.poses.begin(), closestPoint)) % numPoints;

	mGlobalPlan.header = mGlobalPlanPadded.header;
	mGlobalPlan.poses.assign(mGlobalPlanPadded.poses.begin() + mPrevClosestPoint,
	    mGlobalPlanPadded.poses.begin() + mPrevClosestPoint + windowSize);

	return transformPlanToCostmapFrame(mGlobalPlanPadded.poses.begin() + mPrevClosestPoint,
	    mGlobalPlanPadded.poses.begin() + mPrevClosestPoint + windowSize);
}

//////////////////////////////////////////////////////////////////////////

geometry_msgs::msg::PoseStamped FeasiblePathHandler::transformToGlobalPlanFrame(
    const geometry_msgs::msg::PoseStamped & pose)
{
	if (mGlobalPlanUpToConstraint.poses.empty()) {
		mLogger.error("Received plan with zero length");
		throw std::runtime_error("Received plan with zero length");
	}
	if (mRejectUnitPath && mGlobalPlanUpToConstraint.poses.size() == 1) {
		mLogger.error("Received plan with length of one");
		throw std::runtime_error("Received plan with length of one");
	}
	// let's get the pose of the robot in the frame of the plan
	geometry_msgs::msg::PoseStamped robotPose;
	if (!mppi_utils::transformPoseInTargetFrame(pose, robotPose, *mTfBuffer,
	        mGlobalPlanUpToConstraint.header.frame_id, mTransformTolerance, mLogger)) {
		mLogger.error("Unable to transform robot pose into global plan's frame");
		throw std::runtime_error("Unable to transform robot pose into global plan's frame");
	}
	return robotPose;
}

//////////////////////////////////////////////////////////////////////////

PathSegment FeasiblePathHandler::findPlanSegment(const geometry_msgs::msg::PoseStamped & pose)
{
	// std::lock_guard<std::mutex> lock_reinit(mutex_);
	//  Get the robot pose in (path frame)
	//  require to compare the distance
	mGlobalPose = transformToGlobalPlanFrame(pose);
	// Limit the search for the closest pose up to max_robot_pose_search_dist on the path
	auto closestPointUpperBound =
	    mppi_utils::firstAfterIntegratedDistance(mGlobalPlanUpToConstraint.poses.begin(),
	        mGlobalPlanUpToConstraint.poses.end(), mMaxRobotPoseSearchDist);
	// First find the closest pose on the path to the robot
	// bounded by when the path turns around (if it does) so we don't get a pose from a later
	// portion of the path
	auto closestPoint = mppi_utils::minBy(mGlobalPlanUpToConstraint.poses.begin(),
	    closestPointUpperBound, [this](const geometry_msgs::msg::PoseStamped & ps) {
		    return mppi_utils::euclidean_distance(mGlobalPose, ps);
	    });
	// closestPoint: closest Iter on the "trimmed" global path , trimmed by
	// two operations: 1. Until Constraint ( inversion, in place rotation), 2. Search Distance
	// Now we find the closest point on path from the robot.

	// Make sure we always have at least 2 points on the transformed plan and that we don't prune
	// the global plan below 2 points in order to have always enough point to interpolate the
	// end of path direction
	if (mGlobalPlanUpToConstraint.poses.begin() != closestPointUpperBound &&
	    mGlobalPlanUpToConstraint.poses.size() > 1 &&
	    closestPoint == std::prev(closestPointUpperBound)) {
		closestPoint = std::prev(std::prev(closestPointUpperBound));
	}
	auto prunedPlanEnd = mppi_utils::firstAfterIntegratedDistance(
	    closestPoint, mGlobalPlanUpToConstraint.poses.end(), mPruneDistance);

	return {closestPoint, prunedPlanEnd};
}

//////////////////////////////////////////////////////////////////////////

nav_msgs::msg::Path FeasiblePathHandler::transformPlanToCostmapFrame(
    const mppi_utils::PathIterator & begin, const mppi_utils::PathIterator & end)
{
	nav_msgs::msg::Path transformedPlan;
	transformedPlan.header.frame_id = mCostmapRos->getGlobalFrameID();
	transformedPlan.header.stamp = mGlobalPose.header.stamp;
	unsigned int mx, my;
	// Find the furthest relevant pose on the path to consider within costmap
	// bounds
	// Transforming it to the costmap frame in the same loop.
	for (auto globalPlanPoint = begin; globalPlanPoint < end; ++globalPlanPoint) {
		// Transform from global plan frame to costmap frame
		geometry_msgs::msg::PoseStamped costMapPlanPose;
		globalPlanPoint->header.stamp = mGlobalPose.header.stamp;
		globalPlanPoint->header.frame_id = mGlobalPlan.header.frame_id;
		mppi_utils::transformPoseInTargetFrame(*globalPlanPoint, costMapPlanPose, *mTfBuffer,
		    mCostmapRos->getGlobalFrameID(), mTransformTolerance, mLogger);
		// Check if pose is inside the costmap
		if (!mCostmapRos->getCostmap()->worldToMap(
		        costMapPlanPose.pose.position.x, costMapPlanPose.pose.position.y, mx, my)) {
			break;
		}
		// Filling the transformed plan to return with the transformed pose
		transformedPlan.poses.push_back(costMapPlanPose);
	}
	return transformedPlan;
}

//////////////////////////////////////////////////////////////////////////

nav_msgs::msg::Path FeasiblePathHandler::transformLocalPlan(
    const mppi_utils::PathIterator & closestPoint, const mppi_utils::PathIterator & prunedPlanEnd)
{
	// std::lock_guard<std::mutex> lock_reinit(mutex_);
	nav_msgs::msg::Path transformedPlan = transformPlanToCostmapFrame(closestPoint, prunedPlanEnd);

	// Remove the portion of the global plan that we've already passed so we don't
	// process it on the next iteration (this is called path pruning)
	prunePlan(mGlobalPlanUpToConstraint, closestPoint);

	if ((mEnforcePathInversion || mEnforcePathRotation) && mConstraintLocale != 0u) {
		if (isWithinInversionTolerances(mGlobalPose)) {
			prunePlan(mGlobalPlan, mGlobalPlan.poses.begin() + mConstraintLocale);
			mGlobalPlanUpToConstraint = mGlobalPlan;
			mConstraintLocale = mppi_utils::removePosesAfterFirstConstraint(
			    mGlobalPlanUpToConstraint, mEnforcePathInversion, mMinimumRotationAngle);
		}
	}

	if (transformedPlan.poses.empty()) {
		// logger.error("Resulting plan has 0 poses in it.");
		throw std::runtime_error("Resulting plan has 0 poses in it.");
	}

	return transformedPlan;
}

//////////////////////////////////////////////////////////////////////////

geometry_msgs::msg::PoseStamped FeasiblePathHandler::getTransformedGoal(
    const builtin_interfaces::msg::Time & stamp)
{
	auto goal = mGlobalPlan.poses.back();
	goal.header.frame_id = mGlobalPlan.header.frame_id;
	goal.header.stamp = stamp;
	if (goal.header.frame_id.empty()) {
		mLogger.error("Goal pose has an empty frame_id");
		throw std::runtime_error("Goal pose has an empty frame_id");
	}
	geometry_msgs::msg::PoseStamped transformedGoal;
	if (!mppi_utils::transformPoseInTargetFrame(goal, transformedGoal, *mTfBuffer,
	        mCostmapRos->getGlobalFrameID(), mTransformTolerance, mLogger)) {
		mLogger.error("Unable to transform goal pose into costmap frame");
		throw std::runtime_error("Unable to transform goal pose into costmap frame");
	}
	return transformedGoal;
}

}  // namespace controller::mppi_controller::utils

namespace mppi_utils = controller::mppi_controller::utils;