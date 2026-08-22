#include "mppi_controller/critics/common/path_angle_critic.hpp"
#include "mppi_controller/utils/utils.hpp"

/** \file
 * \brief \c mppi_critic::PathAngleCritic "PathAngleCritic" implementation.
 */

void mppi_critic::PathAngleCritic::initialize()
{
    auto getParentParam = mParameter->getParamGetter(mParentName);
    float vx_min;;
    getParentParam(vx_min, "vx_min", -0.35f);
    // mLogger.info("PathAngleCriticCritic vx_min %f", vx_min );
    
    if(fabs(vx_min) < std::numeric_limits<float>::epsilon() )
        mReversingAllowed = false;
    else if ( vx_min < 0.0f )
        mReversingAllowed = true;

    
    auto getParam = mParameter->getParamGetter(mName);
    getParam(mPower, "cost_power", 1);
    getParam(mWeight, "cost_weight", 2.2f);
    getParam(mThresholdToConsider, "threshold_to_consider", 0.5f);
    getParam(mOffsetFromFurthest, "offset_from_furthest", 4);
    getParam(mMaxAngleToFurthest,"max_angle_to_furthest",0.7785498f);
    int mode = 0;
    getParam(mode,"mode",mode);
    mMode = toPathAngleMode(mode);
    if(!mReversingAllowed && mMode == PathAngleMode::NO_DIRECTIONAL_PREFERENCE)
    {
        mLogger.info("Path angle mode set to no directional preference, but controller's settings "
      "don't allow for reversing! Setting mode to forward preference.");
    }

    mLogger.info("PathAngleCriticCritic instantiated with %d power and %f weight. Mode set to: %s",mPower, mWeight, modeToStr(mMode).c_str() );

}

void mppi_critic::PathAngleCritic::score(CriticData& data)
{

    if(!mEnabled || data.model->state()->local_path_length < mThresholdToConsider)
        return;
    
    models::Path* path = data.model->path();
    models::State* state = data.model->state();
    models::Trajectories* trajectories = data.model->trajectories();
    const size_t pathSize = path->x.size();

    mppi_utils::setPathFurthestPointIfNotSet(data);
    auto offsetedPathIdx = std::min(data.furthest_reached_path_point.value() + mOffsetFromFurthest, pathSize);
    
    
    const float goalX = path->x(offsetedPathIdx);
    const float goalY = path->y(offsetedPathIdx);
    const float goalYaw = path->yaws(offsetedPathIdx);

    const geometry_msgs::msg::Pose& pose = state->pose.pose;
    mLogger.info("calculating cost");
    switch(mMode)
    {
        case PathAngleMode::FORWARD_PREFERENCE:
            if(mppi_utils::computePoseToPointAlignmentErrorForward(pose,goalX,goalY,true) < mMaxAngleToFurthest)
            {
                return;
            }
        break;
        case PathAngleMode::NO_DIRECTIONAL_PREFERENCE:
            if(mppi_utils::computePoseToPointAlignmentErrorForward(pose,goalX,goalY,false) < mMaxAngleToFurthest)
            {
                return;
            }
        break;
        case PathAngleMode::CONSIDER_FEASIBLE_PATH_ORIENTATIONS:
            if(mppi_utils::computePoseToPointAlignmentError(pose, goalX, goalY, goalYaw) < mMaxAngleToFurthest)
            {
                return;
            }
        break;
        default:
        throw std::runtime_error("Invalid path angle mode!");

    }
    int lastIdx = trajectories->y.cols() -1;
    auto diffX = goalX - trajectories->x.col(lastIdx);
    auto diffY = goalY - trajectories->y.col(lastIdx);
    auto yawsBetweenPoints = diffX.binaryExpr(diffY, [](const float& x, const float& y){return atan2f(y,x);}).eval();
    

    switch(mMode)
    {
        case PathAngleMode::FORWARD_PREFERENCE:
        {
            auto lastYaws = trajectories->yaws.col(lastIdx);
            // find the shorter yaw-difference from the last
            // yaw(s) given by the end of the trajectories
            // to the goal-direction yaws
            auto yawsDiff = mppi_utils::shortestAngularDistance(lastYaws,yawsBetweenPoints).abs();
            if(mPower>1u)
            {
                data.costs += (yawsDiff*mWeight).pow(mPower);
            }
            else
            {
                data.costs += (yawsDiff*mWeight).eval();
            }
            return;
        }
        case PathAngleMode::NO_DIRECTIONAL_PREFERENCE:
        {
            
            auto lastYaws = trajectories->yaws.col(lastIdx);
            auto yawDiffNormalized = mppi_utils::shortestAngularDistanceNormalized(lastYaws,yawsBetweenPoints).abs().eval();

            //auto  yawsBetweenPointsCorrected = mppi_utils::normalizeYawsBetweenPoints(lastYaws, yawsBetweenPoints);
            //auto correctedYawDiff = mppi_utils::shortestAngularDistance(lastYaws,yawsBetweenPointsCorrected).abs();
            if(mPower>1u)
            {
                data.costs += (yawDiffNormalized*mWeight).pow(mPower);
            }
            else
            {
                data.costs += (yawDiffNormalized*mWeight).eval();
            }
            return;
        }
        case PathAngleMode::CONSIDER_FEASIBLE_PATH_ORIENTATIONS:
        {
            auto lastYaws = trajectories->yaws.col(lastIdx);
            //Penalize trajectories based on how well their final orientation aligns with the path direction,
            //while allowing forward OR reverse alignment (180° ambiguity).
            auto yawBetweenPointsCorrected = mppi_utils::selectClosestYawOrientation(goalYaw, yawsBetweenPoints);
            auto correctedYawDiff = mppi_utils::shortestAngularDistance(lastYaws,yawBetweenPointsCorrected).abs();
            if(mPower>1u)
            {
                data.costs += (correctedYawDiff*mWeight).pow(mPower);
            }
            else
            {
                data.costs += (correctedYawDiff*mWeight).eval();
            }
            return;
        }
        case PathAngleMode::NONE:
        {
            throw std::runtime_error("Invalid PathAngleMode");
            return;
        }
    }
}