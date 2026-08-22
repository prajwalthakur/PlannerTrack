#pragma once
#include "mppi_controller/models/noise_generator.hpp"
#include "mppi_controller/models/ackermann/ackermann_constraints.hpp"
#include "mppi_controller/models/ackermann/ackermann_control_sequence.hpp"
#include "mppi_controller/models/ackermann/ackermann_path.hpp"
#include "mppi_controller/models/ackermann/ackermann_state.hpp"
#include "mppi_controller/models/ackermann/ackermann_trajectory.hpp"
namespace controller::mppi_controller::models
{
   
    /**
     * \brief \ref NoiseGenerator for \ref AckermannModel -- samples
     * independent Gaussian noise for `vx`/`wz` (\ref mNormalDistributionVx
     * / \ref mNormalDistributionWz, scaled by \ref AckermannSamplingStd) on
     * a background thread, ready for \ref setNoisedControls to apply to
     * the next rollout batch.
     */
    class AckermannNoiseGenerator: public NoiseGenerator
    {
        using BaseType = NoiseGenerator;
        public :

            AckermannNoiseGenerator(
                const models::OptimizerSettings& optimizerSettings, 
                const models::ControlConstraints& controlConstraints,
                const models::SamplingStd& samplingStd,
                const mppi_controller::Parameters& parameters, Logger logger) ;
            ~AckermannNoiseGenerator() override =default;

            void onConfigure() override;

            void shutdown() override;

            void generateNextNoises() override;

            void setNoisedControls(models::State& state, const models::ControlSequence& controlSeq) override;

            void reset() override;

        protected :
            void noiseThread() override;
            void generateNoisedControls() override;
        private:

            const models::OptimizerSettings& mSettings;
            const models::AckermannControlConstraints& mControlConstraints;
            const models::AckermannSamplingStd& mSamplingStd;
            const Parameters& mParameters;
            std::normal_distribution<float> mNormalDistributionVx;
            std::normal_distribution<float> mNormalDistributionWz;
            
            mppi_mt::ArrayXX mNoisesVx;
            
            mppi_mt::ArrayXX mNoisesWz;



    };

}//namespace controller::mppi_controller

namespace models = controller::mppi_controller::models;