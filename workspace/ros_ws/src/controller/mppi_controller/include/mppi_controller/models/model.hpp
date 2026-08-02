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
    class Model
    {
        public:
            // Constructors
            Model()=default;
            // Destructors
            virtual ~Model()=default;

            virtual void onConfigure(std::shared_ptr<Parameters> parameters, std::shared_ptr<OptimizerSettings> settings, const std::string& name, Logger logger)=0;
            
            
            virtual void reset() =0;
            /**
            * @brief Integrate batch of controls to get the trajectories rollout.
            * 
            */
            virtual void integrateStateVelocities()=0;

            virtual mppi_mt::ArrayXX getOptimizedTrajectory()=0;
            // Setters
            /**
            * @brief Set the Parameters object
            * 
            * @param parameters 
            */
            virtual void setParameters(Parameters& parameters) = 0;

            virtual void setState(const InputData& inputData)=0;

            virtual void generateNoisedTrajectory()=0; 
            
            virtual void updateStateVelocities()=0;

            virtual void updateControlSequence(mppi_mt::ArrayX&)=0;

            virtual const OptimizerSettings& settings() const =0;
            
            virtual std::string getModelType() const = 0;

            virtual State* state() const=0;

            virtual models::Trajectories* trajectories() const=0;
             
            virtual models::Path* path() const = 0;
            
            // virtual  models::ControlSequence* controlSequence() const = 0;
            virtual void calculateControlFromSequenceAsTwist() =0;

            virtual geometry_msgs::msg::TwistStamped getControlCommand() const = 0;

            virtual void shiftControlSequence()=0;

        protected:
            std::string mName;
    };

}//controller::mppi_controller::models

namespace models = controller::mppi_controller::models;