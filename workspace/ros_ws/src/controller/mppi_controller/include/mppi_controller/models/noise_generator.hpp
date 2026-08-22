#pragma once
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <random>
#include "mppi_controller/models/optimizer_settings.hpp"
#include "mppi_controller/models/constraints.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/models/control_sequence.hpp"
#include "mppi_controller/parameters.hpp"
#include "mppi_controller/utils/logger_utils.hpp"
namespace controller::mppi_controller
{
    namespace models = ::controller::mppi_controller::models;

    /**
    * @brief Generates noise trajectories from optimal trajectory
    */
    class NoiseGenerator
    {

        public:
        /**
        * @brief Constructor for mppi::NoiseGenerator
        */
        NoiseGenerator(
            [[maybe_unused]] const models::OptimizerSettings& optimizerSettings, 
            [[maybe_unused]] const models::ControlConstraints& controlConstraints,
            [[maybe_unused]] const models::SamplingStd& samplingStd,
            [[maybe_unused]] const mppi_controller::Parameters& parameters, Logger logger): mLogger{logger}
            {};
        
        virtual ~NoiseGenerator()
        {
            shutdown();
        };

        /**
        * @brief Initialize noise generator with settings and model types
        * @param settings Settings of controller
        * @param is_holonomic If base is holonomic
        * @param name Namespace for configs
        * @param param_handler Get parameters util
        */
        virtual void onConfigure()=0;

        /**
        * @brief Shutdown noise generator thread
        */
        virtual void shutdown()
        {
            mActive = false;
            mReady = true;
            mNoiseCondition.notify_all();
            if(mNoiseThread.joinable())
            {
                mNoiseThread.join();
            }
        }

        /**
        * @brief Signal to the noise thread the controller is ready to generate a new
        * noised control for the next iteration
        * @param excludeControlDim 
        */
        virtual void generateNextNoises()=0;

        /**
        * @brief set noised control_sequence to state controls
        * @return noises vx, vy, wz
        */
        virtual void setNoisedControls(models::State& state, const models::ControlSequence& controlSeq)=0;

        /**
        * @brief Reset noise generator with settings and model types
        * @param settings Settings of controller
        * @param is_holonomic If base is holonomic
        */
        virtual void reset()=0;

        protected:
        /**
        * @brief Thread to execute noise generation process
        */
        virtual void noiseThread()=0;

        /**
        * @brief Generate random controls by gaussian noise with mean in
        * control_sequence_
        *
        * @return tensor of shape [ batch_size_, time_steps_, control-dim]
        * where control-dim is the dimension of the controller
        */
        virtual void generateNoisedControls()=0;


        // random generator
        std::default_random_engine mGenerator;

        // Thread
        std::thread mNoiseThread;
        std::condition_variable mNoiseCondition;
        std::mutex mNoiseLock;
        bool mActive{false};
        bool mReady{false};
        bool mRegenerateNoises{false};
        // Logger
        Logger mLogger;

};

}  // namespace controller::mppi_controller
namespace models = controller::mppi_controller::models;
