#include "mppi_controller/critics/common/goal_angle_critic.hpp"

/** \file
 * \brief \c mppi_critic::GoalAngleCritic "GoalAngleCritic" implementation.
 */

void mppi_critic::GoalAngleCritic::initialize()
{

    auto getParam = mParameter->getParamGetter(mName);
    getParam(mPower, "cost_power", 1);
    getParam(mWeight, "cost_weight", 3.0f);
    getParam(mThresholdToConsider, "threshold_to_consider", 1.4f);
    getParam(mSymmetricYawTolerance, "symmetric_yaw_tolerance", false);
    mLogger.info("Goal Critic instantiated with %d power and %f weight and %f angular threshold %s",mPower, mWeight, mThresholdToConsider,  mSymmetricYawTolerance ? "enabled" : "disabled");
}

void mppi_critic::GoalAngleCritic::score(CriticData& data)
{
    
    // Goal Critic only activated when the current pose
    // TODO: need to change when there is no global path
    if(!mEnabled || data.model->state()->local_path_length > mThresholdToConsider)
        return;
    
    geometry_msgs::msg::Pose goal = mppi_utils::getLastPathPose(*(data.model->path()));
    controller::mppi_controller::models::Trajectories* trajectoriesPtr = data.model->trajectories();
    float goalYaw = static_cast<float>(tf2::getYaw(goal.orientation));
    auto angularDistances = mppi_utils::shortestAngularDistance(goalYaw,trajectoriesPtr->yaws).abs().eval();
    
    // if true
    // Then robot is allowed to reach in opposite orientation as well
    if(mSymmetricYawTolerance)
    {
        float symmetricGoalYaw = mppi_utils::normalizeAngles(goalYaw + M_PIF);
        auto symAngularDistances = mppi_utils::shortestAngularDistance(symmetricGoalYaw,trajectoriesPtr->yaws).abs().eval();
        angularDistances =  angularDistances.min(symAngularDistances);

    }
    if(mPower>1u)
    {
        data.costs += (((angularDistances.rowwise().mean())*mWeight)).pow(mPower);
    }
    else
    {
        data.costs += (((angularDistances.rowwise().mean())*mWeight)).eval();

    }
}
