#include "mppi_controller/critics/common/path_follow_critic.hpp"
#include "mppi_controller/utils/utils.hpp"

/** \file
 * \brief \c mppi_critic::PathFollowCritic "PathFollowCritic" implementation.
 */

void mppi_critic::PathFollowCritic::initialize()
{
    auto getParam = mParameter->getParamGetter(mName);
    getParam(mPower, "cost_power", 1);
    getParam(mWeight, "cost_weight", 5.0f);
    getParam(mThresholdToConsider, "threshold_to_consider", 1.4f);
    getParam(mOffsetFromFurthest, "offset_from_furthest", 6);
    mLogger.info("PathFollow Critic is enabled %d  instantiated with %d power and %f weight and threshold %.3f ", mEnabled, mPower, mWeight, mThresholdToConsider);

}

void mppi_critic::PathFollowCritic::score(CriticData& data)
{

    models::Path* path = data.model->path();
    models::Trajectories* trajectories = data.model->trajectories();
    const size_t pathSize = path->x.size();
    if(!mEnabled || (data.model->state()->local_path_length < mThresholdToConsider) || pathSize < 2)
        return;
    //mLogger.info("computing the cost");
    // find the furthest point (index) on path which any trajectory is able to acheive
    mppi_utils::setPathFurthestPointIfNotSet(data);
    mppi_utils::setPathCostsIfNotSet(data, mCostmapRos);

    auto offsetedPathIdx = std::min(data.furthest_reached_path_point.value() + mOffsetFromFurthest, pathSize);
    
    // first valid point might be  offsetedPathIdx, but check if its in collision,
    // if it is, then find the next valid point ( and that would be the current short term goal)
    bool isValid = false;
    while( !isValid && (offsetedPathIdx < pathSize - 1) )
    {
        isValid = (data.path_pts_valid.value())[offsetedPathIdx];
        if(!isValid)
            offsetedPathIdx++;
    }
    
    //mLogger.info("offsetedPathIdx %ld", offsetedPathIdx);
    if(offsetedPathIdx== pathSize )
        offsetedPathIdx = pathSize - 1;
    
    const auto pathValidX = path->x(offsetedPathIdx);
    const auto pathValidY = path->y(offsetedPathIdx);

    const int rightmostIdx = trajectories->x.cols()-1;

    const auto lastX = trajectories->x.col(rightmostIdx);
    const auto lastY = trajectories->y.col(rightmostIdx);

    const auto deltaX = lastX - pathValidX;
    const auto deltaY = lastY - pathValidY;
    if(mPower > 1u)
    {
        data.costs += ((deltaX.square() + deltaY.square()).sqrt()*mWeight).pow(mPower);
    }
    else
    {
        data.costs += ((deltaX.square() + deltaY.square()).sqrt()*mWeight).eval();
    }

}