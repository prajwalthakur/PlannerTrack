#include "mppi_controller/models/ackermann/ackermann_constraints.hpp"

/** \file
 * \brief \c models::AckermannControlConstraints "AckermannControlConstraints"
 * / \c models::AckermannSamplingStd "AckermannSamplingStd" implementation.
 */

void models::AckermannControlConstraints::onConfigure(Parameters& parameters, const std::string& name)
{
    auto getParam = parameters.getParamGetter(name);
    getParam(vxMax, "vx_max", 0.5f);
    getParam(vxMin, "vx_min", -0.35f);


    getParam(wzMax, "wz_max", 1.9f);

    getParam(axMax, "ax_max", 3.0f);
    getParam(axMin, "ax_min", -3.0f);
    
    getParam(azMax, "az_max", 3.5f);

    getParam(minTurningRadius,"min_turning_radius",0.4f);


};

void models::AckermannSamplingStd::onConfigure(Parameters& parameters, const std::string& name)
{
     auto getParam = parameters.getParamGetter(name);
     getParam(vx,"vx_std",0.2f);
     getParam(wz,"wz_std",0.4f);

}