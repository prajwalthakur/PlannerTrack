#pragma once
#include "mppi_controller/models/constraints.hpp"
#include "mppi_controller/parameters.hpp"
namespace controller::mppi_controller::models
{
struct AckermannControlConstraints : public ControlConstraints
{
	~AckermannControlConstraints() override = default;
	void onConfigure(Parameters & parameters, const std::string & name) final;

	float vxMax;
	float vxMin;

	float wzMax;
	float wzMin;

	float axMax;
	float axMin;

	float azMax;
	float azMin;
	float minTurningRadius;
	float getMinTurningRadius() const { return minTurningRadius; }
	//////////////////////////////////////////////////////////////////////////
};

struct AckermannSamplingStd : public SamplingStd
{
	~AckermannSamplingStd() override = default;
	void onConfigure(Parameters & parameters, const std::string & name) final;

	float vx;
	float wz;
};

}  // namespace controller::mppi_controller::models

namespace models = controller::mppi_controller::models;