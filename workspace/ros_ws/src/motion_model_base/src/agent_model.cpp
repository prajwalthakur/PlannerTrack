/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "motion_model_base/agent_model.hpp"

/** \file
 * \brief \ref AgentModel implementation: thin forwarding to the composed
 * dynamics/geometry/collision/sensor plugins.
 */

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

const DynamicModel & AgentModel::dynamicModel() const
{
	return *mDynamicModel;
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

void AgentModel::resetToInitialState()
{
	mDynamicModel->reset();
}

//////////////////////////////////////////////////////////////////////////

void AgentModel::updateCommandedControl(const InputVector & u)
{
	mDynamicModel->updateCommandedControl(u);
}

//////////////////////////////////////////////////////////////////////////

void AgentModel::step(const WorldSnapshot & world,bool stepSensorModel)
{
	//mLogger.info("stepping agent, id: %d ", mId.value());
	mDynamicModel->step();
	auto pose = mDynamicModel->getStatePose();
	mGeomModel->step(pose);
	if(stepSensorModel)
	{
		mSensorModel->step(mId, pose, world);
	}
	mCollisionFootPrint->step(pose);
}

void AgentModel::publishStates() const
{
	mDynamicModel->publishStates();
}