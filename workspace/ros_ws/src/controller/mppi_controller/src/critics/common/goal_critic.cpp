#include "mppi_controller/critics/common/goal_critic.hpp"

/** \file
 * \brief \c mppi_critic::GoalCritic "GoalCritic" implementation.
 */

void mppi_critic::GoalCritic::initialize()
{

    auto getParam = mParameter->getParamGetter(mName);
    getParam(mPower, "cost_power", 1);
    getParam(mWeight, "cost_weight", 5.0f);
    getParam(mThresholdToConsider, "threshold_to_consider", 1.4f);
    getParam(mPathFollow, "path_follow", true);
    mLogger.info("Goal Critic instantiated with %d power and %f weight",mPower, mWeight);

}

void mppi_critic::GoalCritic::score(CriticData& data)
{
    
    // Goal Critic only activated when the current pose
    // TODO: need to change when there is no global path
    //mLogger.info("calling the score: local-path length %.3f threshold %.3f",data.model->state()->local_path_length, mThresholdToConsider);
    if(!mEnabled)
        return;
    if(data.model->state()->local_path_length > mThresholdToConsider)
        return; 
    geometry_msgs::msg::Pose goal = mppi_utils::getLastPathPose(*(data.model->path()));
    auto goal_x = goal.position.x;
    auto goal_y = goal.position.y;

    models::Trajectories& trajectories = *data.model->trajectories();
    const auto delta_x = trajectories.x - goal_x;
    const auto delta_y =  trajectories.y - goal_y;
    if(mPower>1u)
    {
        data.costs += (((delta_x.square() + delta_y.square()).sqrt().rowwise().mean())*mWeight).pow(mPower);
    }
    else
    {
        data.costs += (((delta_x.square() + delta_y.square()).sqrt().rowwise().mean())*mWeight).eval();
    }
    
}
