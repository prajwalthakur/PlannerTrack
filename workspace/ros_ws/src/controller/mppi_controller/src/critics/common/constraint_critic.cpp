#include "mppi_controller/critics/common/constraint_critic.hpp"
#include "mppi_controller/utils/utils.hpp"

void mppi_critic::ConstraintCritic::initialize()
{
    auto getParam = mParameter->getParamGetter(mName);
    auto getParentParam = mParameter->getParamGetter(mParentName);
    getParam(mPower, "cost_power", 1);
    getParam(mWeight, "cost_weight", 4.0f);
    mLogger.info("Constraint Critic instantiated with %d power and %f weight ",mPower, mWeight);

    float vx_max, vy_max, vx_min;
    getParentParam(vx_max, "vx_max", 0.5f);
    getParentParam(vy_max, "vy_max", 0.0f);
    getParentParam(vx_min, "vx_min", -0.35f);
    mMaxVelX = vx_max; 
    mMinVelX = vx_min;
    mMaxVelY =  vy_max;

}

void mppi_critic::ConstraintCritic::score(CriticData& data)
{
    if(!mEnabled)
        return;

    auto diffModel = dynamic_cast<models::AckermannModel*>(data.model);
    if(diffModel == nullptr)
        return;

    models::AckermannState* state = static_cast<models::AckermannState*>(diffModel->state());
    auto& vx = state->vx;
    auto& wz = state->wz;
    float dt = data.model_dt;

    const float minTurningRadius = diffModel->getMinTurningRadius();
    const float epsilon = 1e-6f;
    auto wzSafe = wz.abs().max(epsilon);  // avoid division by 0 for near-zero yaw rate
    auto turningRadiusViolation = (minTurningRadius - (vx.abs() / wzSafe)).max(0.0f);

    auto violation = ((vx - mMaxVelX).max(0.0f) + (mMinVelX - vx).max(0.0f) + turningRadiusViolation) * dt;
    auto integrated = violation.rowwise().sum() * mWeight;

    if (mPower > 1u)
    {
        data.costs += integrated.pow(mPower);
    }
    else
    {
        data.costs += integrated;
    }
}


