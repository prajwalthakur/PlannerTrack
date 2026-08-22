#include "mppi_controller/critics/common/velocity_deadband_critic.hpp"
#include "mppi_controller/utils/utils.hpp"

/** \file
 * \brief \c mppi_critic::VelocityDeadbandCritic "VelocityDeadbandCritic" implementation.
 */

void mppi_critic::VelocityDeadbandCritic::initialize()
{
    auto getParam = mParameter->getParamGetter(mName);
    getParam(mPower, "cost_power", 1);
    getParam(mWeight, "cost_weight", 4.0f);


    std::vector<double> deadband_velocities{0.0, 0.0, 0.0};
    getParam(deadband_velocities, "deadband_velocities", std::vector<double>{0.0, 0.0, 0.0});
    mDeadbandVelocities.resize(deadband_velocities.size());
    std::transform(
    deadband_velocities.begin(), deadband_velocities.end(), mDeadbandVelocities.begin(),
    [](double d) {return static_cast<float>(fabs(d));});
    mLogger.info("Constraint Critic instantiated with %d power and %f weight and abs deadband velocities [ %f %f %f ]",mPower, mWeight, mDeadbandVelocities[0],mDeadbandVelocities[1],mDeadbandVelocities[2]);
    
}

void mppi_critic::VelocityDeadbandCritic::score(CriticData& data)
{
    if(!mEnabled)
        return;
    
    auto diffModel = dynamic_cast<models::AckermannModel*>(data.model);
    if(diffModel != nullptr)
    {
        models::AckermannState* state = static_cast<models::AckermannState*>(diffModel->state());
        // differential drive model
        auto vx = state->vx;
        auto wz = state->wz;
        float dt  = data.model_dt;
        
        auto violation = (((( mDeadbandVelocities[0] - vx.abs() ).max(0.0f) + (mDeadbandVelocities[2] - wz.abs()).max(0.0f))*dt).rowwise().sum().eval())*mWeight;
        if (mPower > 1u)
        {
            data.costs += violation.pow(mPower);
        }
        else
        {
            data.costs += violation;
        }
        return;

    }
}


