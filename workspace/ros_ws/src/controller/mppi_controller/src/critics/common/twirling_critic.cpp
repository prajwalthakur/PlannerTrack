#include "mppi_controller/critics/common/twirling_critic.hpp"
#include "mppi_controller/utils/utils.hpp"

void mppi_critic::TwirlingCritic::initialize()
{
    auto getParam = mParameter->getParamGetter(mName);
    getParam(mPower, "cost_power", 1);
    getParam(mWeight, "cost_weight", 5.0f);
    getParam(mPositionXTol , "pose_tolerance.x",0.1f);
    mLogger.info("Twirling Critic instantiated with %d power and %f weight ",mPower, mWeight);

}

void mppi_critic::TwirlingCritic::score(CriticData& data)
{
    if(!mEnabled)
        return;

    models::State* state = data.model->state();

    if(state->local_path_length < mPositionXTol)
        return;
    if (mPower > 1u) 
    {
        data.costs += ((state->wz.abs().rowwise().mean()) * mWeight).pow(mPower).eval();
    } else 
    {
        data.costs += ((state->wz.abs().rowwise().mean()) * mWeight).eval();
    }
}