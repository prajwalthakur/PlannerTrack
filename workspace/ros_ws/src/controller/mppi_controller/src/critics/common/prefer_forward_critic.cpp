#include "mppi_controller/critics/common/prefer_forward_critic.hpp"
#include "mppi_controller/utils/utils.hpp"

void mppi_critic::PreferForwardCritic::initialize()
{
    auto getParam = mParameter->getParamGetter(mName);
    getParam(mPower, "cost_power", 1);
    getParam(mWeight, "cost_weight", 5.0f);
    getParam(mThresholdToConsider, "threshold_to_consider", 1.4);
    mLogger.info("PreferForward Critic instantiated with %d power and %f weight ",mPower, mWeight);

}

void mppi_critic::PreferForwardCritic::score(CriticData& data)
{
    
    models::State* state = data.model->state();
    if(!mEnabled || data.model->state()->local_path_length < mThresholdToConsider)
        return;
    const auto& vx = state->vx;
    float dt = data.model_dt;
    mLogger.info("calculating cost");
    if(mPower > 1u)
    {
        data.costs += (((-vx).cwiseMax(0.0f)*dt).rowwise().sum()*mWeight).pow(mPower); 
    }
    else
    {
        data.costs += (((-vx).cwiseMax(0.0f)*dt).rowwise().sum()*mWeight); 

    }

}