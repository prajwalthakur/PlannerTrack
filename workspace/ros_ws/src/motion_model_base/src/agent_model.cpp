/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "motion_model_base/agent_model.hpp"

//////////////////////////////////////////////////////////////////////////

AgentModel::AgentModel(mpl::rclcpp_utils::Logger & logger, const UniqueId & id,
    ptSharedPtr<DynamicModel> dynamicModel, ptSharedPtr<GeometricModel> geometricModel,
    ptSharedPtr<CollisionFootPrint> collisionModel, ptSharedPtr<SensorModel> sensorModel)
{
	mLogger = logger;
	mId = id;
	mDynamicModel = dynamicModel;
	mGeomModel = geometricModel;
	mCollisionFootPrint = collisionModel;
	mSensorModel = sensorModel;
}

//////////////////////////////////////////////////////////////////////////

const UniqueId & AgentModel::id() const
{
	return mId;
}

//////////////////////////////////////////////////////////////////////////

const GeometricModel & AgentModel::geometry() const
{
	return *mGeomModel;
}

//////////////////////////////////////////////////////////////////////////

const CollisionFootPrint & AgentModel::collisionFootprint() const
{
	return *mCollisionFootPrint;
}

//////////////////////////////////////////////////////////////////////////

const SensorModel & AgentModel::sensor() const
{
	return *mSensorModel;
}

//////////////////////////////////////////////////////////////////////////

stPose AgentModel::getStatePose() const
{
	return mDynamicModel->getStatePose();
}

//////////////////////////////////////////////////////////////////////////

const StateVector & AgentModel::getState() const
{
	return mDynamicModel->getState();
}

//////////////////////////////////////////////////////////////////////////

void AgentModel::updateCommandedControl(const InputVector & u)
{
	mDynamicModel->updateCommandedControl(u);
}

//////////////////////////////////////////////////////////////////////////

void AgentModel::step(const WorldSnapshot & world)
{
	mLogger.info("stepping agent, id: %d ", mId.value());
	mDynamicModel->step();
	auto pose = mDynamicModel->getStatePose();
	mGeomModel->step(pose);
	mSensorModel->step(mId, pose, world);
	mCollisionFootPrint->step(pose);
}