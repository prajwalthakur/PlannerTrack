#pragma once 
#include <memory>
#include "mppi_controller/parameters.hpp"
namespace controller::mppi_controller::models
{
    /**
    * @struct controller::mppi_controller::models::ControlConstraints
    * @brief Possible Constraints on control
    */
    struct ControlConstraints
    {
        virtual ~ControlConstraints() = default;
        /// \brief Load constraint bounds (e.g. max velocity, min turning radius) from ROS params under \p name.
        virtual void onConfigure(Parameters& parameters, const std::string& name)=0;
    };

    /**
    * @struct  controller::mppi_controller::models::SamplingStd
    * @brief Possible Noise parameters for sampling trajectories
    */
    struct SamplingStd
    {
        virtual ~SamplingStd()=default;
        /// \brief Load per-control-dimension noise standard deviations from ROS params under \p name.
        virtual void onConfigure(Parameters& parameters, const std::string& name)=0;

        //virtual void setParameters(std::shared_ptr<Parameters> parameters)=0;
    };

} // namespadce constraints
