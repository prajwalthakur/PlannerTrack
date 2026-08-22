#include "mppi_controller/models/ackermann/ackermann.hpp"

#include "mppi_controller/utils/common_utils.hpp"
#include "mppi_controller/utils/geometry_utils.hpp"

/** \file
 * \brief \c models::AckermannModel "AckermannModel" implementation:
 * configuration, kinematic integration, and control-sequence
 * shift/constraint handling.
 */

//////////////////////////////////////////////////////////////////////////

void models::AckermannModel::onConfigure(std::shared_ptr<Parameters> parameters,
    std::shared_ptr<OptimizerSettings> settings, const std::string & name, Logger logger)
{
	mName = name;
	mLogger = logger;
	configure(parameters, settings);
}

//////////////////////////////////////////////////////////////////////////

void models::AckermannModel::configure(
    std::shared_ptr<Parameters> parameters, std::shared_ptr<OptimizerSettings> settings)
{
	mParameters = parameters;
	mControlConstraints = std::make_unique<AckermannControlConstraints>();
	mSamplingStdPtr = std::make_unique<AckermannSamplingStd>();

	mPath = std::make_unique<AckermannPath>();
	mTrajectories = std::make_unique<AckermannTrajectories>();
	mState = std::make_unique<AckermannState>();
	mSettings = settings;
	mControlSequence = std::make_unique<AckermannControlSequence>();

	Logger noiseLogger(mLogger, "noise_logger");
	mNoiseGenerator = std::make_unique<AckermannNoiseGenerator>(
	    *mSettings, *mControlConstraints, *mSamplingStdPtr, *mParameters, noiseLogger);
	setParameters(*mParameters);
}

//////////////////////////////////////////////////////////////////////////

void models::AckermannModel::setParameters(Parameters & parameters)
{
	mControlConstraints->onConfigure(parameters, mName);
	mSamplingStdPtr->onConfigure(parameters, mName);
	mNoiseGenerator->onConfigure();
}

//////////////////////////////////////////////////////////////////////////

void models::AckermannModel::toTensor(const nav_msgs::msg::Path & path)
{
	auto & poses = path.poses;
	if (poses.empty()) {
		// mLogger.warn("Received empty path!")

		return;
	}

	size_t numPoints = poses.size();
	mPath->reset(numPoints);

	for (size_t i = 0; i < numPoints; ++i) {
		const auto & p = poses[i].pose;

		mPath->x(i) = p.position.x;
		mPath->y(i) = p.position.y;
		mPath->yaws(i) = tf2::getYaw(p.orientation);
	}
	// mLogger.info("new path size %zu, last pose: x %.3f, y %.3f, yaw %.3f",
	//          numPoints,
	//          mPath->x(numPoints-1),
	//          mPath->y(numPoints-1),
	//          mPath->yaws(numPoints-1));
	if (!mPath->geom_path_initialized) mPath->setGeometricPath();
}

//////////////////////////////////////////////////////////////////////////

void models::AckermannModel::setState(const InputData & inputData)
{
	// geometry_msgs::msg::PoseStamped pose;
	// pose.header = inputData.mCurrentOdom.header;
	// pose.pose = inputData.mCurrentOdom.pose.pose;
	mState->pose = inputData.mRobotPose;

	if (mSettings->path_follow) {
		mState->local_path_length = mppi_utils::calculatePathLength(inputData.mPlanToFollow);
		// mLogger.info("local-path size %ld, length %.3f",inputData.mPlanToFollow.poses.size(),
		// mState->local_path_length);
		toTensor(inputData.mPlanToFollow);
	}

	mState->speed = mSettings->open_loop ? mLastCmdSpeed.twist : inputData.mSpeed;
	// mCosts.setZero(mSettings->batch_size);

	mHeader = std::make_shared<std_msgs::msg::Header>(inputData.mPlanToFollow.header);
	mHeader->frame_id = inputData.mBaseFrameID;
}

//////////////////////////////////////////////////////////////////////////

void models::AckermannModel::generateNoisedTrajectory()
{
	mNoiseGenerator->setNoisedControls(*mState, *mControlSequence);
	mNoiseGenerator->generateNextNoises();

	// Debugger
	// int batchSize = mSettings->batch_size;
	// int numTimeSteps = mSettings->time_steps;
	// // cvx : batch_sizexT
	// static std::mt19937 gen(std::random_device{}());
	// std::uniform_int_distribution<int> batch_dist(0, batchSize - 1);
	// std::uniform_int_distribution<int> time_dist(0, numTimeSteps - 1);

	// int i = batch_dist(gen);
	// int j = time_dist(gen);
	// mLogger.info("Noise sample at (i=%d, j=%d): cvx=C, cw=%.4f",
	//          i, j,
	//          mState->cvx(i, j),
	//          mState->cwz(i, j));
}

//////////////////////////////////////////////////////////////////////////

void models::AckermannModel::updateStateVelocities()
{
	float model_dt = mSettings->model_dt;

	// set initial vx,wz
	// mLogger.info("size vx  %ld size cvx cols %ld",  mState->vx.cols(), mState->cvx.cols());
	mState->vx.col(0) = static_cast<float>(mState->speed.linear.x);
	mState->wz.col(0) = static_cast<float>(mState->speed.angular.z);

	[[maybe_unused]] float maxDeltaVx = model_dt * mControlConstraints->axMax;
	[[maybe_unused]] float minDeltaVx = model_dt * mControlConstraints->axMin;
	[[maybe_unused]] float maxDeltaWz = model_dt * mControlConstraints->azMax;

	[[maybe_unused]] size_t n = mState->vx.cols();
	// mLogger.info("n %ld", n);
	for (size_t i = 1; i < n; ++i) {
		auto lowerBoundVx =
		    (mState->vx.col(i - 1) > 0)
		        .select(mState->vx.col(i - 1) + minDeltaVx, mState->vx.col(i - 1) - maxDeltaVx);

		auto upperBoundVx =
		    (mState->vx.col(i - 1) > 0)
		        .select(mState->vx.col(i - 1) + maxDeltaVx, mState->vx.col(i - 1) - minDeltaVx);

		mState->cvx.col(i - 1) =
		    mState->cvx.col(i - 1).cwiseMax(lowerBoundVx).cwiseMin(upperBoundVx);
		mState->cwz.col(i - 1) = mState->cwz.col(i - 1)
		                             .cwiseMax(mState->wz.col(i - 1) - maxDeltaWz)
		                             .cwiseMin(mState->wz.col(i - 1) + maxDeltaWz);

		mState->vx.col(i) = mState->cvx.col(i - 1);
		mState->wz.col(i) = mState->cwz.col(i - 1);
	}
}

//////////////////////////////////////////////////////////////////////////

void models::AckermannModel::integrateStateVelocities()
{
	auto & state = *(mState);
	auto & trajectories = *(mTrajectories);
	auto & settings = *(mSettings);
	[[maybe_unused]] const size_t n_cols = trajectories.yaws.cols();
	const size_t n_rows = trajectories.yaws.rows();
	const float model_dt = settings.model_dt;
	// Set the initial state
	Eigen::ArrayXf prev_x = Eigen::ArrayXf::Constant(n_rows, state.pose.pose.position.x);
	Eigen::ArrayXf prev_y = Eigen::ArrayXf::Constant(n_rows, state.pose.pose.position.y);
	auto initial_yaw = static_cast<float>(tf2::getYaw(state.pose.pose.orientation));
	Eigen::ArrayXf prev_yaw = Eigen::ArrayXf::Constant(n_rows, initial_yaw);

	// Set the yaw first
	for (size_t i = 0; i < n_cols; ++i) {
		prev_yaw += state.wz.col(i) * model_dt;
		trajectories.yaws.col(i) = prev_yaw;
	}
	// Faster to compute the trig this way, triggers SIMD
	Eigen::ArrayXXf yaw_cos = trajectories.yaws.cos();
	Eigen::ArrayXXf yaw_sin = trajectories.yaws.sin();
	mppi_utils::shiftColumnsByOnePlace(yaw_cos, 1);
	mppi_utils::shiftColumnsByOnePlace(yaw_sin, 1);
	yaw_cos.col(0) = cosf(initial_yaw);
	yaw_sin.col(0) = sinf(initial_yaw);

	// Forward simulate differential robot dynamics
	// x += v_x cos(yaw)*model_dt
	// y += v_x sin(yaw)*model_dt
	// yaw +=  wz*model_dt
	// yaw = arctan2(sin(yaw),cos(yaw))
	auto dx = (state.vx * yaw_cos).eval();
	auto dy = (state.vx * yaw_sin).eval();
	for (size_t i = 0; i < n_cols; ++i) {
		prev_x += dx.col(i) * model_dt;
		prev_y += dy.col(i) * model_dt;
		trajectories.x.col(i) = prev_x;
		trajectories.y.col(i) = prev_y;
	}
}

//////////////////////////////////////////////////////////////////////////

void models::AckermannModel::integrateStateVelocities(
    mppi_mt::ArrayX3 & trajectory, const mppi_mt::ArrayX2 & controlSequence) const
{
	auto & state = *(mState);
	auto & settings = *(mSettings);
	[[maybe_unused]] const size_t n_cols = trajectory.cols();
	const size_t n_rows = trajectory.rows();
	const float model_dt = settings.model_dt;

	auto vx = controlSequence.col(0);
	auto wz = controlSequence.col(1);

	auto traj_x = trajectory.col(0);
	auto traj_y = trajectory.col(1);
	auto traj_yaws = trajectory.col(2);

	if (n_rows == 0) {
		return;
	}

	// Set the initial state
	float prev_x = static_cast<float>(state.pose.pose.position.x);
	float prev_y = static_cast<float>(state.pose.pose.position.y);
	float prev_yaw = static_cast<float>(tf2::getYaw(state.pose.pose.orientation));
	float initial_yaw = prev_yaw;

	// Set the yaw first
	for (size_t i = 0; i < n_rows; ++i) {
		prev_yaw += wz(i) * model_dt;
		traj_yaws(i) = prev_yaw;
	}
	// Faster to compute the trig this way, triggers SIMD
	Eigen::ArrayXXf yaw_cos = traj_yaws.cos();
	Eigen::ArrayXXf yaw_sin = traj_yaws.sin();

	mppi_utils::shiftColumnsByOnePlace(yaw_cos, 1);
	mppi_utils::shiftColumnsByOnePlace(yaw_sin, 1);
	yaw_cos(0) = cosf(initial_yaw);
	yaw_sin(0) = sinf(initial_yaw);

	// Forward simulate differential robot dynamics
	// x += (v_x cos(yaw) )*model_dt
	// y += (v_x sin(yaw) )*model_dt
	// yaw +=  wz*model_dt
	// yaw = arctan2(sin(yaw),cos(yaw))
	auto dx = (vx * yaw_cos).eval();
	auto dy = (vx * yaw_sin).eval();

	for (size_t i = 0; i < n_rows; ++i) {
		prev_x += dx(i) * model_dt;
		prev_y += dy(i) * model_dt;
		traj_x(i) = prev_x;
		traj_y(i) = prev_y;
	}
}

//////////////////////////////////////////////////////////////////////////
/**
 * @brief Apply hard vehicle constraints to a control sequence
 * @param control_sequence Control sequence to apply constraints to
 */

//////////////////////////////////////////////////////////////////////////

void models::AckermannModel::applyConstraints()
{
	auto & controlSequence = *mControlSequence;
	float min_turning_r_ = mControlConstraints->getMinTurningRadius();
	const auto wz_constrained = controlSequence.vx.abs() / min_turning_r_;
	controlSequence.wz = controlSequence.wz.max((-wz_constrained)).min(wz_constrained);
	return;
}

/**
 * @brief Get minimum turning radius of ackermann drive
 * @return Minimum turning radius
 */

//////////////////////////////////////////////////////////////////////////

float models::AckermannModel::getMinTurningRadius()
{
	return mControlConstraints->getMinTurningRadius();
}

//////////////////////////////////////////////////////////////////////////

void models::AckermannModel::applyControlSequenceConstraints()
{
	auto & settings = *mSettings;
	auto & constraints = *mControlConstraints;
	auto & controlSequence = *mControlSequence;
	float modelDt = settings.model_dt;

	float maxDeltaVx = modelDt * constraints.axMax;
	float minDeltaVx = modelDt * constraints.axMin;
	float maxDeltaWz = modelDt * constraints.azMax;

	float vxPrev = mppi_utils::clamp(constraints.vxMin, constraints.vxMax, controlSequence.vx(0));
	float wzPrev = mppi_utils::clamp(-constraints.wzMax, constraints.wzMax, controlSequence.wz(0));

	// should be clamp again to make sure of acceleration constraints ?
	controlSequence.vx(0) = vxPrev;
	controlSequence.wz(0) = wzPrev;

	size_t numControls = controlSequence.vx.size();

	for (size_t i = 1; i < numControls; ++i) {
		float & vxCurr = controlSequence.vx(i);
		vxCurr = mppi_utils::clamp(constraints.vxMin, constraints.vxMax, vxCurr);
		if (vxPrev > 0) {
			vxCurr = mppi_utils::clamp(vxPrev + minDeltaVx, vxPrev + maxDeltaVx, vxCurr);
		} else {
			vxCurr = mppi_utils::clamp(vxPrev - maxDeltaVx, vxPrev - minDeltaVx, vxCurr);
		}
		vxPrev = vxCurr;

		float & wzCurr = controlSequence.wz(i);
		wzCurr = mppi_utils::clamp(-constraints.wzMax, constraints.wzMax, wzCurr);
		wzCurr = mppi_utils::clamp(wzPrev - maxDeltaWz, wzPrev + maxDeltaWz, wzCurr);
		wzPrev = wzCurr;
	}
}

//////////////////////////////////////////////////////////////////////////

void models::AckermannModel::updateControlSequence(mppi_mt::ArrayX & costs)
{
	auto & controlSequence = *mControlSequence;
	auto & settings = *mSettings;
	auto & state = *mState;
	auto & samplingStd = *mSamplingStdPtr;

	// Penalizes the control effort by calculating the inner product of noise and mean,
	// ensuring the optimizer balances goal-seeking with energy efficiency and smoothness.
	auto vxT = controlSequence.vx.transpose();  // previously computed optimal control-trajectory
	auto boundedNoiseVx = state.cvx.rowwise() - vxT;
	const float gammaVx = settings.gamma / (samplingStd.vx * samplingStd.vx);
	costs += (gammaVx * (boundedNoiseVx.rowwise() * vxT).rowwise().sum()).eval();

	if (samplingStd.wz > 0.0f) {
		auto wzT = controlSequence.wz.transpose();
		auto boundedNoiseWz = state.cwz.rowwise() - wzT;
		const float gammaWz = settings.gamma / (samplingStd.wz * samplingStd.wz);
		costs += (gammaWz * (boundedNoiseWz.rowwise() * wzT).rowwise().sum()).eval();
	}

	auto costsNormalized = costs - costs.minCoeff();
	const float invTemp = 1.0f / settings.temperature;
	auto softmaxes = (-invTemp * costsNormalized).exp().eval();
	softmaxes /= softmaxes.sum();

	auto softmax_mat = softmaxes.matrix();
	controlSequence.vx = state.cvx.transpose().matrix() * softmax_mat;
	controlSequence.wz = state.cwz.transpose().matrix() * softmax_mat;

	savitskyGolayFilter(*mControlSequence, mControlHistory, *mSettings);
	applyControlSequenceConstraints();
	applyConstraints();
}

//////////////////////////////////////////////////////////////////////////

mppi_mt::ArrayXX models::AckermannModel::getOptimizedTrajectory()
{
	// vx, wz
	auto controlSequence = mppi_mt::ArrayX2(mSettings->time_steps, 2);
	// trajectories
	auto trajectory = mppi_mt::ArrayX3(mSettings->time_steps, 3);
	controlSequence.col(0) = mControlSequence->vx;
	controlSequence.col(1) = mControlSequence->wz;
	integrateStateVelocities(trajectory, controlSequence);
	return trajectory;
}

//////////////////////////////////////////////////////////////////////////

const models::OptimizerSettings & models::AckermannModel::settings() const
{
	return *mSettings;
}

//////////////////////////////////////////////////////////////////////////

std::string models::AckermannModel::getModelType() const
{
	std::string modelType = "ackermann";
	return modelType;
}

//////////////////////////////////////////////////////////////////////////

void models::AckermannModel::reset()
{
	mState->reset(mSettings->batch_size, mSettings->time_steps);
	// mLogger.error(" resetting model size vx  %ld size cvx cols %ld",  mState->vx.cols(),
	// mState->cvx.cols());

	mControlSequence->reset(mSettings->time_steps);
	mNoiseGenerator->reset();
	mTrajectories->reset(mSettings->batch_size, mSettings->time_steps);
	// mCosts.setZero(mSettings->batch_size);

	if (mSettings->open_loop) {
		mLastCmdSpeed = geometry_msgs::msg::TwistStamped();
	}

	mControlHistory[0] = {0.0f, 0.0f};
	mControlHistory[1] = {0.0f, 0.0f};
	mControlHistory[2] = {0.0f, 0.0f};
	mControlHistory[3] = {0.0f, 0.0f};
}

//////////////////////////////////////////////////////////////////////////

models::State * models::AckermannModel::state() const
{
	return mState.get();
}

//////////////////////////////////////////////////////////////////////////

models::Trajectories * models::AckermannModel::trajectories() const
{
	return mTrajectories.get();
}

//////////////////////////////////////////////////////////////////////////

models::Path * models::AckermannModel::path() const
{
	return mPath.get();
}

// model::ControlSequence* models::AckermannModel::controlSequence() const
// {
//     return mControlSequence.get();
// }

//////////////////////////////////////////////////////////////////////////

void models::AckermannModel::calculateControlFromSequenceAsTwist()
{
	unsigned int offset = mSettings->shift_control_sequence ? 1 : 0;

	auto vx = mControlSequence->vx(offset);
	auto wz = mControlSequence->wz(offset);
	mLastCmdSpeed = mppi_utils::toTwistStamped(vx, wz, *mHeader);
}

//////////////////////////////////////////////////////////////////////////

geometry_msgs::msg::TwistStamped models::AckermannModel::getControlCommand() const
{
	return mLastCmdSpeed;
}

//////////////////////////////////////////////////////////////////////////

void models::AckermannModel::shiftControlSequence()
{
	auto & controlSequence = *mControlSequence;
	auto size = controlSequence.vx.size();
	utils::shiftColumnsByOnePlace(controlSequence.vx, -1);
	utils::shiftColumnsByOnePlace(controlSequence.wz, -1);
	controlSequence.vx(size - 1) = controlSequence.vx(size - 2);
	controlSequence.wz(size - 1) = controlSequence.wz(size - 2);
}

//////////////////////////////////////////////////////////////////////////

void models::AckermannModel::savitskyGolayFilter(
    models::AckermannControlSequence & control_sequence,
    std::array<models::AckermannControl, 4> & control_history,
    const models::OptimizerSettings & settings)
{
	// Savitzky-Golay Quadratic, 9-point Coefficients
	Eigen::Array<float, 9, 1> filter = {
	    -21.0f, 14.0f, 39.0f, 54.0f, 59.0f, 54.0f, 39.0f, 14.0f, -21.0f};
	filter /= 231.0f;

	// Too short to smooth meaningfully
	const unsigned int num_sequences = control_sequence.vx.size() - 1;
	if (num_sequences < 20) {
		return;
	}

	auto applyFilter = [&](const Eigen::Array<float, 9, 1> & data) -> float {
		return (data * filter).eval().sum();
	};

	auto applyFilterOverAxis =
	    [&](Eigen::ArrayXf & sequence, const Eigen::ArrayXf & initial_sequence, const float hist_0,
	        const float hist_1, const float hist_2, const float hist_3) -> void {
		float pt_m4 = hist_0;
		float pt_m3 = hist_1;
		float pt_m2 = hist_2;
		float pt_m1 = hist_3;
		float pt = initial_sequence(0);
		float pt_p1 = initial_sequence(1);
		float pt_p2 = initial_sequence(2);
		float pt_p3 = initial_sequence(3);
		float pt_p4 = initial_sequence(4);

		for (unsigned int idx = 0; idx != num_sequences; idx++) {
			sequence(idx) =
			    applyFilter({pt_m4, pt_m3, pt_m2, pt_m1, pt, pt_p1, pt_p2, pt_p3, pt_p4});
			pt_m4 = pt_m3;
			pt_m3 = pt_m2;
			pt_m2 = pt_m1;
			pt_m1 = pt;
			pt = pt_p1;
			pt_p1 = pt_p2;
			pt_p2 = pt_p3;
			pt_p3 = pt_p4;

			if (idx + 5 < num_sequences) {
				pt_p4 = initial_sequence(idx + 5);
			} else {
				// Return the last point
				pt_p4 = initial_sequence(num_sequences);
			}
		}
	};

	// Filter trajectories
	const models::AckermannControlSequence initial_control_sequence = control_sequence;
	applyFilterOverAxis(control_sequence.vx, initial_control_sequence.vx, control_history[0].vx,
	    control_history[1].vx, control_history[2].vx, control_history[3].vx);
	applyFilterOverAxis(control_sequence.wz, initial_control_sequence.wz, control_history[0].wz,
	    control_history[1].wz, control_history[2].wz, control_history[3].wz);

	// Update control history
	unsigned int offset = settings.shift_control_sequence ? 1 : 0;
	control_history[0] = control_history[1];
	control_history[1] = control_history[2];
	control_history[2] = control_history[3];
	control_history[3] = {control_sequence.vx(offset), control_sequence.wz(offset)};
}