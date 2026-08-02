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
        virtual void onConfigure(Parameters& parameters, const std::string& name)=0;
    };

    /**
    * @struct  controller::mppi_controller::models::SamplingStd
    * @brief Possible Noise parameters for sampling trajectories
    */
    struct SamplingStd
    {
        virtual ~SamplingStd()=default;
        virtual void onConfigure(Parameters& parameters, const std::string& name)=0;

        //virtual void setParameters(std::shared_ptr<Parameters> parameters)=0;
    };

} // namespadce constraints
