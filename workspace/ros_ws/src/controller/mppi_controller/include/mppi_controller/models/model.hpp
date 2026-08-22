#pragma once
#include "mppi_controller/parameters.hpp"
#include "mppi_controller/input_data.hpp"
#include "mppi_controller/models/optimizer_settings.hpp"
#include "mppi_controller/utils/logger_utils.hpp"
#include "mppi_controller/models/trajectory.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/models/path.hpp"
#include "mppi_controller/utils/geometry_utils.hpp"
#include "mppi_controller/utils/types.hpp"
namespace controller::mppi_controller::models
{
    /**
     * \brief Abstract vehicle model interface driving \c Optimizer's
     * rollout batch: owns the noised control sequences, the state/pose
     * rollouts they produce, the reference \ref Path, and conversion of
     * the optimized control sequence back into a \c Twist command.
     *
     * \ref AckermannModel is the one concrete implementation loaded by
     * \ref createModel; created behind this interface so \c Optimizer
     * has no compile-time dependency on any specific model type.
     */
    class Model
    {
        public:
            Model()=default;
            virtual ~Model()=default;

            /// \brief Configure from ROS-param-derived settings, ready to start rolling out.
            virtual void onConfigure(std::shared_ptr<Parameters> parameters, std::shared_ptr<OptimizerSettings> settings, const std::string& name, Logger logger)=0;

            /// \brief Reset internal rollout/control-sequence state (e.g. on a new goal/replan).
            virtual void reset() =0;
            /**
            * @brief Integrate batch of controls to get the trajectories rollout.
            * 
            */
            virtual void integrateStateVelocities()=0;

            /// \brief Get the current optimized (nominal) control sequence as a plain array.
            virtual mppi_mt::ArrayXX getOptimizedTrajectory()=0;
            // Setters
            /// \brief Re-set the shared \ref Parameters object.
            virtual void setParameters(Parameters& parameters) = 0;

            /// \brief Set the current pose/velocity/reference-path state from this cycle's \ref InputData.
            virtual void setState(const InputData& inputData)=0;

            /// \brief Sample a batch of randomly-perturbed control sequences around the nominal one.
            virtual void generateNoisedTrajectory()=0;

            /// \brief Roll each noised control sequence forward one step to get per-rollout state velocities.
            virtual void updateStateVelocities()=0;

            /// \brief Update the nominal control sequence from \p costs (the cost-weighted rollout average).
            virtual void updateControlSequence(mppi_mt::ArrayX&)=0;

            /// \brief This model's \c OptimizerSettings.
            virtual const OptimizerSettings& settings() const =0;

            /// \brief This model's type name (e.g. `"Ackermann"`).
            virtual std::string getModelType() const = 0;

            /// \brief Non-owning access to this model's current \ref State.
            virtual State* state() const=0;

            /// \brief Non-owning access to the last-generated rollout \ref models::Trajectories "Trajectories".
            virtual models::Trajectories* trajectories() const=0;

            /// \brief Non-owning access to the reference \ref models::Path "Path" being tracked.
            virtual models::Path* path() const = 0;

            // virtual  models::ControlSequence* controlSequence() const = 0;
            /// \brief Convert the current control-sequence head into a \c geometry_msgs::msg::Twist.
            virtual void calculateControlFromSequenceAsTwist() =0;

            /// \brief Get this cycle's commanded twist (see \ref calculateControlFromSequenceAsTwist).
            virtual geometry_msgs::msg::TwistStamped getControlCommand() const = 0;

            /// \brief Shift the control sequence one step forward in time, for warm-starting the next cycle.
            virtual void shiftControlSequence()=0;

        protected:
            std::string mName;
    };

}//controller::mppi_controller::models

namespace models = controller::mppi_controller::models;