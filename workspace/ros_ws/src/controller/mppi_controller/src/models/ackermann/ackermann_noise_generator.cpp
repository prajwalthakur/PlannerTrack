#include "mppi_controller/models/ackermann/ackermann_noise_generator.hpp"


models::AckermannNoiseGenerator::AckermannNoiseGenerator(
    const OptimizerSettings& optimizerSettings,
    const ControlConstraints& controlConstraints,
    const SamplingStd& samplingStd,
    const mppi_controller::Parameters& parameters, Logger logger
    )
    :BaseType(optimizerSettings,controlConstraints, samplingStd, parameters, logger),
      mSettings{optimizerSettings},
      mControlConstraints{static_cast<const models::AckermannControlConstraints&>(controlConstraints)},
      mSamplingStd{static_cast<const models::AckermannSamplingStd&>(samplingStd)},
      mParameters{parameters}{}


void models::AckermannNoiseGenerator::onConfigure()
{
    mActive = true;
    mNormalDistributionVx = std::normal_distribution(0.0f, mSamplingStd.vx);
    mNormalDistributionWz = std::normal_distribution(0.0f, mSamplingStd.wz);
    mRegenerateNoises =   mSettings.regenerate_noise;

    if (mRegenerateNoises) 
    {
        mNoiseThread = std::thread(std::bind(&models::AckermannNoiseGenerator::noiseThread, this));
    
    } else 
    {
        generateNoisedControls();
    }
    // mLogger.info("Configuring. Regenerate noise is: %s", mSettings.regenerate_noise ? "TRUE" : "FALSE");
    // mLogger.info("Configuring, mSamplingStd.vx %.3f ,  mSamplingStd.wz %.3f ", mSamplingStd.vx, mSamplingStd.wz);

}

void models::AckermannNoiseGenerator::shutdown()
{
    mActive = false;
    mReady = true;
    mNoiseCondition.notify_all();
    if(mNoiseThread.joinable())
    {
        mNoiseThread.join();
    }
}

void models::AckermannNoiseGenerator::generateNextNoises()
{
    //Trigger the thread to run in parallel to this iteration
    // generating the noise for the next iteration.
    // By the time you will require the noise, 
    // we will already have computed it.
    {
        std::unique_lock<std::mutex> guard(mNoiseLock);
        mReady = true;
    }
    mNoiseCondition.notify_all();
    // mLogger.info("called generateNextNoises");

}

void models::AckermannNoiseGenerator::setNoisedControls(models::State& state, const models::ControlSequence& controlSeq)
{
    models::AckermannState & diffState = static_cast< models::AckermannState&>(state);
    const models::AckermannControlSequence & diffControlSeq = static_cast<const models::AckermannControlSequence&>(controlSeq);
    
    std::unique_lock<std::mutex> guard(mNoiseLock);
    diffState.cvx = mNoisesVx.rowwise() + diffControlSeq.vx.transpose(); //diffControlSeq.vx is vector of (num-time-stepsx1), transpose it
    diffState.cwz = mNoisesWz.rowwise() + diffControlSeq.wz.transpose();

    // mLogger.info("called setNoisedControls");

}

void models::AckermannNoiseGenerator::reset()
{
    // Reset
    {
        std::unique_lock<std::mutex> guard(mNoiseLock);
        mNoisesVx.setZero(mSettings.batch_size,mSettings.time_steps);
        mNoisesWz.setZero(mSettings.batch_size,mSettings.time_steps);
        mReady = true;

    }
    if(mRegenerateNoises)
    {
        mNoiseCondition.notify_all();
    }
    else
    {
        generateNoisedControls();
    }
}


void models::AckermannNoiseGenerator::noiseThread()
{
    do
    {
        std::unique_lock<std::mutex> guard(mNoiseLock);
        mNoiseCondition.wait(guard,[this](){return mReady;});
        generateNoisedControls();
        mReady = false;
    }while(mActive);
}


void models::AckermannNoiseGenerator::generateNoisedControls()
{
    mNoisesVx = mppi_mt::ArrayXX::NullaryExpr(mSettings.batch_size,mSettings.time_steps,[this](){return mNormalDistributionVx(mGenerator); });
    mNoisesWz = mppi_mt::ArrayXX::NullaryExpr(mSettings.batch_size,mSettings.time_steps,[this](){return mNormalDistributionWz(mGenerator); });
    // mLogger.info("called generateNoisedControls");
}
