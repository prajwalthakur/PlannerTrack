#include "mppi_controller/vanilla_mppi_class.hpp"

using namespace controller::mppi_controller;

// Optimizer::Optimizer( std::shared_ptr<Parameters>  mppi_parameters , const std::string& name)
// {
//     mMppiCtrlParamPtr = mppi_parameters;
//     mName = name;
// }
// Optimizer::Optimizer(std::shared_ptr<Parameters>  mppi_parameters, std::shared_ptr<CostmapRos>
// costMapRos, const std::string& name)
// {
//     mCostMapRosPtr = costMapRos;
//     mMppiCtrlParamPtr = mppi_parameters;
//     mName = name;
// }

Optimizer::Optimizer(std::shared_ptr<Parameters> mppi_parameters,
    std::shared_ptr<CostmapRos> costMapRos, Logger logger, const std::string & name,
    const rclcpp::Clock::SharedPtr & clock)
    : mClock{clock}
{
	mCostMapRosPtr = costMapRos;
	mMppiCtrlParamPtr = mppi_parameters;
	mLogger = logger;
	mName = name;
	mLogger.info("mppi-controller initialized");
}

//////////////////////////////////////////////////////////////////////////

void Optimizer::onConfigure()
{
	configure();
	mLogger.info("mppi-controller controller configured and activated");
}

//////////////////////////////////////////////////////////////////////////

void Optimizer::configure()
{
	// Optimizer Settings
	mOptimizerSettings = std::make_shared<models::OptimizerSettings>();
	mOptimizerSettings->onConfigure(*mMppiCtrlParamPtr, mName);

	// Get the specific model
	auto getParam = mMppiCtrlParamPtr->getParamGetter(mName);
	std::string modelType;
	getParam(modelType, "motion_model", std::string(""));
	if (modelType.empty()) {
		mLogger.error("No model_type param in the dictonary");
		return;
	}
	// Create model
	Logger modelLogger(mLogger, "model");
	mModel = controller::mppi_controller::models::createModel(modelType);
	mLogger.error("model %s", modelType.c_str());
	if (!mModel) {
		mLogger.error("Model not generated");
		return;
	}
	// Configure model
	// mLogger.info("mOptimizerSettings info batch_size %ld time-steps %ld",
	// mOptimizerSettings->batch_size, mOptimizerSettings->time_steps);
	mModel->onConfigure(mMppiCtrlParamPtr, mOptimizerSettings, mName, modelLogger);

	// Critics Manager
	Logger crticsLogger(mLogger, "critics");
	mCriticsManager = std::make_unique<CriticsManager>(mClock);
	mCriticsManager->onConfigure(mMppiCtrlParamPtr.get(), mName, mCostMapRosPtr, crticsLogger);

	// Critic data
	mCriticsData.model = mModel.get();
	mCriticsData.model_dt = mOptimizerSettings->model_dt;

	mLogger.info("all critics loaded");
	float controllerFrequency;
	auto getParentParam = mMppiCtrlParamPtr->getParamGetter("");
	getParentParam(controllerFrequency, "controller_frequency", 0.0);
	setOffset(controllerFrequency, mOptimizerSettings->model_dt);

	// Check if path-to follow enabled or disabled
	getParam(mPathFollow, "path_follow", true);

	reset();
}

//////////////////////////////////////////////////////////////////////////

void Optimizer::setOffset(float controllerFrequency, float modelDt)
{
	const float controllerPeriod = 1.0 / controllerFrequency;
	constexpr float eps = 1e-6;

	if ((controllerPeriod + eps) < modelDt) {
		mLogger.warn("Controller period is less then model dt, consider setting it equal");
	} else if (abs(controllerPeriod - modelDt) < eps) {
		mLogger.info("Controller period is equal to model dt. Control sequence : shifting is ON");
		mOptimizerSettings->shift_control_sequence = true;
	} else {
		throw std::logic_error("Controller period more then model dt, set it equal to model dt");
	}
}

//////////////////////////////////////////////////////////////////////////

OutputData Optimizer::computeControl([[maybe_unused]] InputData & inputData, CostMap * costMap2d)
{
	[[maybe_unused]] CostMap * costMap2dPtr = costMap2d;

	OutputData OutputData;
	computeControlInternal(inputData, OutputData);
	return OutputData;
}

//////////////////////////////////////////////////////////////////////////

OutputData Optimizer::computeControl([[maybe_unused]] InputData & inputData)
{
	OutputData outputData;
	computeControlInternal(inputData, outputData);
	return outputData;
}

//////////////////////////////////////////////////////////////////////////

void Optimizer::reset(bool resetDynamicSpeedLimits)
{
	mModel->reset();
	mCosts.setZero(mOptimizerSettings->batch_size);
	[[maybe_unused]] bool ifReset = resetDynamicSpeedLimits;
	// TODO: speed-zone constraints
	// mOptimizerSettings->mResetDynamicSpeedLimits = resetDynamicSpeedLimits;
	// if (resetDynamicSpeedLimits)
	// {
	//     mOptimizerSettings->
	//     settings_.constraints = settings_.base_constraints;
	// }
	// TODO:
	//   trajectory_validator_->initialize(
	//     parent_, name_ + ".TrajectoryValidator",
	//     costmap_ros_, parameters_handler_, tf_buffer_, settings_);
	mLogger.info("Optimize reset");
}

//////////////////////////////////////////////////////////////////////////

void Optimizer::prepare(const InputData & inputData)
{
	mGoal = inputData.mGoalPose;
	mCosts.setZero(mOptimizerSettings->batch_size);
	// mLogger.info("preparation  mCosts!");

	mCriticsData.reset();
	// mLogger.info("preparation mCriticsData.path_pts_valid.reset() !");
	//  TODO:(prajwalthakur)
	//  critics_data_.goal_checker = goal_checker;
	mModel->setState(inputData);
	// mLogger.info("preparation done !");
}

//////////////////////////////////////////////////////////////////////////

void Optimizer::computeControlInternal(InputData & inputData, OutputData & outputData)
{
	// std::string modelType = mModel->getModelType();
	// std::cerr<<" model type " << modelType;
	// // store the optimal trajectory
	// // state-dimxnum_steps
	// mppi_mt::ArrayXX optimalTrajectory;
	// [[maybe_unused]] bool trajectoryValid = true;

	[[maybe_unused]] bool trajectory_valid = true;
	prepare(inputData);
	// TODO: when not to compute the control command and send the zero
	// // goal reached ?
	if ((mModel->state()->local_path_length < 1e-6)) {
		mLogger.warn("path to follow length %.3f ", mModel->state()->local_path_length);
		outputData.mPublishZeroVelocity = true;
		return;
	}
	mLogger.info("running mppi optimization");
	// 7 ms above loop

	// TODO: do while loop , check the trajectory validator
	optimize();

	mppi_mt::ArrayXX optimalTrajectory = getOptimizedTrajectory();

	calculateControlFromSequenceAsTwist();

	geometry_msgs::msg::TwistStamped cmd = getControlCommand();

	outputData.mOptimalTrajectory = optimalTrajectory;
	outputData.mControlCommand = cmd;

	if (mOptimizerSettings->shift_control_sequence) {
		shiftControlSequence();
	}
}

//////////////////////////////////////////////////////////////////////////

void Optimizer::optimize()
{
	for (size_t itr = 0; itr < mModel->settings().iteration_count; ++itr) {
		generateNoisedTrajectory();
		updateStateVelocities();
		integrateStateVelocities();

		evalTrajectoriesScores();
		updateControlSequence();
	}
}

//////////////////////////////////////////////////////////////////////////

void Optimizer::generateNoisedTrajectory()
{
	mModel->generateNoisedTrajectory();
}

//////////////////////////////////////////////////////////////////////////

void Optimizer::updateStateVelocities()
{
	mModel->updateStateVelocities();
}

//////////////////////////////////////////////////////////////////////////

void Optimizer::integrateStateVelocities()
{
	mModel->integrateStateVelocities();
}

//////////////////////////////////////////////////////////////////////////

void Optimizer::evalTrajectoriesScores()
{
	// CriticData data;
	mCriticsManager->evalTrajectoriesScores(mCriticsData);
}

//////////////////////////////////////////////////////////////////////////

void Optimizer::updateControlSequence()
{
	mModel->updateControlSequence(mCosts);
}

//////////////////////////////////////////////////////////////////////////

mppi_mt::ArrayXX Optimizer::getOptimizedTrajectory()
{
	mppi_mt::ArrayX3 optimizedTrajectory = mModel->getOptimizedTrajectory();
	return optimizedTrajectory;
}

//////////////////////////////////////////////////////////////////////////

void Optimizer::calculateControlFromSequenceAsTwist()
{
	mModel->calculateControlFromSequenceAsTwist();
}

//////////////////////////////////////////////////////////////////////////

geometry_msgs::msg::TwistStamped Optimizer::getControlCommand()
{
	return mModel->getControlCommand();
}

//////////////////////////////////////////////////////////////////////////

void Optimizer::shiftControlSequence()
{
	mModel->shiftControlSequence();
}